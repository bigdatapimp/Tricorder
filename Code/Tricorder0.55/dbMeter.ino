//------------- dB Meter code ---------------
//
// A sound level meter in the style of a phone app:
//   - DC offset removed (the PDM microphone has one; it used to count as "sound")
//   - A-weighting (dBA, how loud it sounds to a person) or flat/unweighted (dB)
//   - Fast (125 ms) or Slow (1 s) time weighting, so the reading is steady instead of jumping every 16 ms
//   - Max (highest level since reset), Leq (average level since reset) and Pk (loudest single sample)
//   - Audio goes through a queue, so no sound is lost while the screens are being redrawn
//
// Controls:   UP (short)    A-weighting <-> flat          DOWN (short)  Fast <-> Slow
//             RIGHT (short) reset Max / Leq / Pk          LEFT (long)   exit
//             UP / DOWN (long)  calibration offset +1 / -1 dB
//
// CALIBRATION: the microphone's real sensitivity isn't known exactly, so the readings are offset by
// DB_CAL_DEFAULT (from the microphone datasheet). To match a reference: hold a phone-app meter next to the
// unit, play a steady sound (pink noise or a constant tone), set the app to A-weighting + Fast, and press
// long UP / long DOWN until both read the same. The value is printed to the Serial Monitor: put it in
// DB_CAL_DEFAULT below to keep it after a power cycle.
//
// Note: very loud sounds can overload the microphone (the CLIP warning). Readings are too low while it shows.

#define DB_FS           16000.0   // PDM sample rate
#define DB_CAL_DEFAULT  94.5     // dB SPL for a full-scale signal (datasheet: -26 dBFS at 94 dB SPL, +15 dB PDM gain)
#define DB_SCALE_MIN    30.0      // left end of the bar / graph
#define DB_SCALE_MAX    120.0     // right end
#define DB_LED_YELLOW   70.0      // RGB LED turns yellow above this ...
#define DB_LED_RED      85.0      // ... and red above this (85 dBA = common hearing-protection limit)
#define DB_WARMUP_FRAMES 10       // ignore the first ~160 ms while the filters settle

// ======================= audio queue (filled by the PDM callback, emptied here) =======================
#define AUDIO_RING_FRAMES 8       // 8 x 16 ms = 128 ms of buffering
#define AUDIO_FRAME_MAX   256

short audioRing[AUDIO_RING_FRAMES][AUDIO_FRAME_MAX];
volatile int audioRingLen[AUDIO_RING_FRAMES];
volatile uint8_t audioRingHead = 0;        // advanced only by the PDM callback
volatile uint8_t audioRingTail = 0;        // advanced only by the meter
volatile unsigned long audioRingDropped = 0;

// Called from onPDMdata() (interrupt context): just copies the frame into the queue
void audioRingPush(const short* data, int n) {
  uint8_t next = (audioRingHead + 1) % AUDIO_RING_FRAMES;
  if (next == audioRingTail) {             // queue full: drop this frame
    audioRingDropped++;
    return;
  }
  if (n > AUDIO_FRAME_MAX) n = AUDIO_FRAME_MAX;
  memcpy(audioRing[audioRingHead], data, n * sizeof(short));
  audioRingLen[audioRingHead] = n;
  audioRingHead = next;
}

// ======================= DSP (no display code in this block) =======================
// ---- DSP BEGIN ----
float dbCalOffset = DB_CAL_DEFAULT;
bool  dbWeightA = true;                    // true = A-weighted, false = flat
bool  dbSlow = false;                      // false = Fast (125 ms), true = Slow (1 s)

float dbHpX = 0.0, dbHpY = 0.0;            // DC blocker state
float dbBqB[2][3], dbBqA[2][2], dbBqZ[2][2];   // two A-weighting biquads (transposed direct form II)
float dbTopB0 = 0.0, dbTopB1 = 0.0, dbTopA1 = 0.0, dbTopX = 0.0, dbTopY = 0.0;   // first-order top section
float dbAGain = 1.0;                       // normalises the A-weighting to 0 dB at 1 kHz
float dbAlpha = 0.0005;                    // time-weighting smoothing per sample
float dbPower = 0.0;                       // time-weighted mean-square level
bool  dbDesigned = false;

float dbLevel = 0.0;                       // current time-weighted level (dB)
float dbMaxLevel = 0.0;                    // highest level since reset
float dbLeqDb = 0.0;                       // average level since reset
float dbPeakDb = 0.0;                      // loudest single sample since reset (flat)
double dbEqSum = 0.0;
unsigned long dbEqFrames = 0;
int   dbWarmFrames = 0;
bool  dbClipSeen = false;                  // set when a sample hit full scale; cleared by the display code

// Analog biquad (b2 s^2 + b1 s + b0) / (a2 s^2 + a1 s + a0) -> digital biquad by the bilinear transform
void dbBiquadFromAnalog(double b2, double b1, double b0, double a2, double a1, double a0, double c,
                        double* b, double* a) {
  double nb0 = b2 * c * c + b1 * c + b0;
  double nb1 = -2.0 * b2 * c * c + 2.0 * b0;
  double nb2 = b2 * c * c - b1 * c + b0;
  double da0 = a2 * c * c + a1 * c + a0;
  double da1 = -2.0 * a2 * c * c + 2.0 * a0;
  double da2 = a2 * c * c - a1 * c + a0;
  b[0] = nb0 / da0; b[1] = nb1 / da0; b[2] = nb2 / da0;
  a[0] = da1 / da0; a[1] = da2 / da0;
}

// |H| of one digital section at angular frequency th (radians/sample)
double dbSectionMag(const double* b, double a1, double a2, double th) {
  double c1 = cos(th), s1 = sin(th), c2 = cos(2.0 * th), s2 = sin(2.0 * th);
  double nr = b[0] + b[1] * c1 + b[2] * c2;
  double ni = -(b[1] * s1 + b[2] * s2);
  double dr = 1.0 + a1 * c1 + a2 * c2;
  double di = -(a1 * s1 + a2 * s2);
  return sqrt((nr * nr + ni * ni) / (dr * dr + di * di));
}

// Design the A-weighting filter for DB_FS. Poles at 20.6 Hz (x2), 107.7 Hz and 737.9 Hz are placed exactly
// (pre-warped). The 12.2 kHz pole of the real curve is above this sample rate's limit, so it is replaced by
// a gentle first-order section (corner chosen to fit the standard curve: within 0.5 dB to 3 kHz, 1.5 dB at 7 kHz).
void dbDesignAWeighting() {
  const double fs = DB_FS;
  const double c = 2.0 * fs;
  const double f1 = 20.598997, f2 = 107.65265, f3 = 737.86223;
  const double fTop = 40200.0;
  double w1 = c * tan(PI * f1 / fs);
  double w2 = c * tan(PI * f2 / fs);
  double w3 = c * tan(PI * f3 / fs);
  double wt = 2.0 * PI * fTop;

  double b1[3], a1[2], b2[3], a2[2], bt[3], at[2];
  dbBiquadFromAnalog(1.0, 0.0, 0.0, 1.0, 2.0 * w1, w1 * w1, c, b1, a1);        // s^2 / (s+w1)^2
  dbBiquadFromAnalog(1.0, 0.0, 0.0, 1.0, w2 + w3, w2 * w3, c, b2, a2);          // s^2 / ((s+w2)(s+w3))
  double tb0 = wt / (c + wt);                                                   // first order: wt / (s+wt)
  double ta1 = (wt - c) / (c + wt);
  bt[0] = tb0; bt[1] = tb0; bt[2] = 0.0; at[0] = ta1; at[1] = 0.0;

  // Normalise so the whole chain is exactly 0 dB at 1 kHz
  double th = 2.0 * PI * 1000.0 / fs;
  double mag = dbSectionMag(b1, a1[0], a1[1], th) * dbSectionMag(b2, a2[0], a2[1], th) * dbSectionMag(bt, at[0], at[1], th);
  dbAGain = (float)(1.0 / mag);

  for (int i = 0; i < 3; i++) { dbBqB[0][i] = (float)b1[i]; dbBqB[1][i] = (float)b2[i]; }
  for (int i = 0; i < 2; i++) { dbBqA[0][i] = (float)a1[i]; dbBqA[1][i] = (float)a2[i]; }
  dbTopB0 = (float)tb0; dbTopB1 = (float)tb0; dbTopA1 = (float)ta1;
  dbDesigned = true;
}

void dbSetTimeWeighting() {
  double tau = dbSlow ? 1.0 : 0.125;
  dbAlpha = (float)(1.0 - exp(-1.0 / (DB_FS * tau)));
}

void dbResetStats() {
  dbMaxLevel = 0.0;
  dbLeqDb = 0.0;
  dbPeakDb = 0.0;
  dbEqSum = 0.0;
  dbEqFrames = 0;
}

void dbDspInit() {
  if (!dbDesigned) dbDesignAWeighting();
  dbHpX = dbHpY = 0.0;
  for (int s = 0; s < 2; s++) dbBqZ[s][0] = dbBqZ[s][1] = 0.0;
  dbTopX = dbTopY = 0.0;
  dbPower = 0.0;
  dbLevel = 0.0;
  dbSetTimeWeighting();
  dbResetStats();
  dbWarmFrames = DB_WARMUP_FRAMES;
  dbClipSeen = false;
}

// Process one frame of 16-bit samples
void dbDspProcess(const short* data, int n) {
  if (n <= 0) return;
  float sum = 0.0;
  float pk = 0.0;

  for (int i = 0; i < n; i++) {
    short sm = data[i];
    if (sm >= 32760 || sm <= -32760) dbClipSeen = true;

    float x = sm * (1.0f / 32768.0f);
    float y = x - dbHpX + 0.995f * dbHpY;         // DC blocker (high-pass at about 13 Hz)
    dbHpX = x;
    dbHpY = y;

    float ay = fabsf(y);
    if (ay > pk) pk = ay;

    if (dbWeightA) {
      for (int s = 0; s < 2; s++) {               // two biquads
        float out = dbBqB[s][0] * y + dbBqZ[s][0];
        dbBqZ[s][0] = dbBqB[s][1] * y - dbBqA[s][0] * out + dbBqZ[s][1];
        dbBqZ[s][1] = dbBqB[s][2] * y - dbBqA[s][1] * out;
        y = out;
      }
      float t = dbTopB0 * y + dbTopB1 * dbTopX - dbTopA1 * dbTopY;   // first-order top section
      dbTopX = y;
      dbTopY = t;
      y = t * dbAGain;
    }

    float p = y * y;
    dbPower += dbAlpha * (p - dbPower);           // Fast / Slow time weighting
    sum += p;
  }

  float frameMean = sum / n;

  if (dbWarmFrames > 0) {                         // filters still settling: follow the signal, don't record stats
    dbWarmFrames--;
    dbPower = frameMean;
    return;
  }

  float lv = 10.0f * log10f(dbPower + 1e-12f) + dbCalOffset;
  if (lv < 0.0f) lv = 0.0f;
  dbLevel = lv;
  if (lv > dbMaxLevel) dbMaxLevel = lv;

  dbEqSum += frameMean;
  dbEqFrames++;
  float leq = 10.0f * log10f((float)(dbEqSum / dbEqFrames) + 1e-12f) + dbCalOffset;
  dbLeqDb = (leq < 0.0f) ? 0.0f : leq;

  float pkDb = 20.0f * log10f(pk + 1e-6f) + dbCalOffset;
  if (pkDb > dbPeakDb) dbPeakDb = pkDb;
}
// ---- DSP END ----

// ======================= display =======================
#define DB_BAR_X     1               // bar interior on the TFT
#define DB_BAR_Y     20
#define DB_BAR_W     93
#define DB_BAR_H     12
#define DB_HIST_W    128             // OLED history graph
#define DB_GRAPH_TOP 19
#define DB_GRAPH_H   32

char dbShown[6][16];                 // what is currently drawn on the TFT, per text field
uint8_t dbHist[DB_HIST_W];           // OLED history, one column per refresh
int dbHistHead = 0;
int dbBarDrawn = -1;                 // bar width / marker position currently drawn
int dbMarkDrawn = -1;

int dbDbToBarPx(float dB) {
  int px = (int)((dB - DB_SCALE_MIN) * DB_BAR_W / (DB_SCALE_MAX - DB_SCALE_MIN));
  if (px < 0) px = 0;
  if (px > DB_BAR_W) px = DB_BAR_W;
  return px;
}

// Draw text, but only the characters that changed (TFT text is slow). field = which buffer to compare against.
void dbTftText(int field, int x, int y, int size, const char* text, uint16_t fg, uint16_t bg) {
  tft.setTextSize(size);
  tft.setTextColor(fg, bg);
  for (int i = 0; text[i] != '\0'; i++) {
    if (dbShown[field][i] != text[i]) {
      tft.setCursor(x + i * 6 * size, y);
      tft.write(text[i]);
      dbShown[field][i] = text[i];
    }
  }
}

// Fill bar columns [from, to) with green / yellow / red zones, or erase them
void dbFillBar(int from, int to, bool erase) {
  int greenEnd = dbDbToBarPx(DB_LED_YELLOW);
  int yellowEnd = dbDbToBarPx(DB_LED_RED);
  int a = from;
  while (a < to) {
    int zoneEnd;
    uint16_t color;
    if (a < greenEnd)       { zoneEnd = greenEnd;  color = GREEN; }
    else if (a < yellowEnd) { zoneEnd = yellowEnd; color = YELLOW; }
    else                    { zoneEnd = DB_BAR_W;  color = RED; }
    int b = (zoneEnd < to) ? zoneEnd : to;
    tft.fillRect(DB_BAR_X + a, DB_BAR_Y, b - a, DB_BAR_H, erase ? BLACK : color);
    a = b;
  }
}

// Update the bar. 'px' = current level, 'mk' = Max marker position. Only redraws what changed.
void dbDrawBar(int px, int mk, bool force) {
  if (force || mk != dbMarkDrawn) {                 // marker moved (or first draw): repaint the whole bar
    tft.fillRect(DB_BAR_X, DB_BAR_Y, DB_BAR_W + 1, DB_BAR_H, BLACK);
    dbFillBar(0, px, false);
    if (mk > 0) tft.drawFastVLine(DB_BAR_X + mk, DB_BAR_Y, DB_BAR_H, WHITE);
    dbBarDrawn = px;
    dbMarkDrawn = mk;
    return;
  }
  if (px > dbBarDrawn)      dbFillBar(dbBarDrawn, px, false);
  else if (px < dbBarDrawn) dbFillBar(px, dbBarDrawn, true);
  dbBarDrawn = px;
}

// The parts of the TFT screen that never change
void dbDrawTftStatic() {
  tft.fillScreen(BLACK);
  tft.drawRect(0, DB_BAR_Y - 1, 96, DB_BAR_H + 2, BLUE2);
  tft.setTextSize(1);
  tft.setTextColor(WHITE, BLACK);
  for (int d = 30; d <= 120; d += 10) {             // scale ticks every 10 dB
    int x = DB_BAR_X + (int)((d - DB_SCALE_MIN) * DB_BAR_W / (DB_SCALE_MAX - DB_SCALE_MIN));
    tft.drawFastVLine(x, DB_BAR_Y + DB_BAR_H + 2, (d % 30 == 0) ? 4 : 2, WHITE);
  }
  tft.setCursor(1, 36);   tft.print("30");
  tft.setCursor(26, 36);  tft.print("60");
  tft.setCursor(57, 36);  tft.print("90");
  tft.setCursor(76, 36);  tft.print("120");
  memset(dbShown, 0, sizeof(dbShown));
  dbBarDrawn = -1;
  dbMarkDrawn = -1;
}

void dbSetLed(float level) {
  if (level >= DB_LED_RED) {            // red
    digitalWrite(LEDR, LOW);  digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);
  } else if (level >= DB_LED_YELLOW) {  // yellow
    digitalWrite(LEDR, LOW);  digitalWrite(LEDG, LOW);  digitalWrite(LEDB, HIGH);
  } else {                              // green
    digitalWrite(LEDR, HIGH); digitalWrite(LEDG, LOW);  digitalWrite(LEDB, HIGH);
  }
}

void dbLedsOff() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

void DBMeter(int pressType) {
  static bool initialized = false;
  static unsigned long lastBarMs = 0;
  static unsigned long lastNumMs = 0;
  static unsigned long lastOledMs = 0;
  static unsigned long clipUntil = 0;
  static bool oledDirty = true;

  // --- Exit ---
  if (pressType == LONG_PRESS && lastButton == 5) {
    dbLedsOff();
    tft.fillScreen(BLACK);
    initialized = false;
    currentState = ENV_MENU;
    return;
  }

  // --- Buttons ---
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) {                 // Up: A-weighting <-> flat
      dbWeightA = !dbWeightA;
      dbResetStats();
      oledDirty = true;
    } else if (lastButton == 4) {          // Down: Fast <-> Slow
      dbSlow = !dbSlow;
      dbSetTimeWeighting();
      dbResetStats();
      oledDirty = true;
    } else if (lastButton == 3) {          // Right: reset Max / Leq / Pk
      dbResetStats();
      oledDirty = true;
    }
  } else if (pressType == LONG_PRESS) {
    if (lastButton == 2 || lastButton == 4) {          // Up / Down long: calibration offset
      dbCalOffset += (lastButton == 2) ? 1.0 : -1.0;
      dbResetStats();
      oledDirty = true;
      Serial.print(F("dB calibration offset: "));
      Serial.print(dbCalOffset, 1);
      Serial.println(F("  (set DB_CAL_DEFAULT to this value to keep it)"));
    }
  }

  // --- First pass after entering ---
  if (!initialized) {
    initialized = true;
    dbDspInit();
    audioRingTail = audioRingHead;                     // throw away audio queued before we started
    memset(dbHist, 0, sizeof(dbHist));
    dbHistHead = 0;
    dbDrawTftStatic();
    lastBarMs = lastNumMs = lastOledMs = 0;
    clipUntil = 0;
    oledDirty = true;
  }

  // --- Process every queued audio frame ---
  while (audioRingTail != audioRingHead) {
    uint8_t t = audioRingTail;
    dbDspProcess(audioRing[t], audioRingLen[t]);
    audioRingTail = (t + 1) % AUDIO_RING_FRAMES;
  }
  unsigned long now = millis();
  if (dbClipSeen) {
    dbClipSeen = false;
    clipUntil = now + 2000;
  }
  bool clipping = (now < clipUntil);

  // --- TFT bar + LED: about 20 times a second ---
  if (now - lastBarMs >= 50) {
    lastBarMs = now;
    dbDrawBar(dbDbToBarPx(dbLevel), dbDbToBarPx(dbMaxLevel), false);
    dbSetLed(dbLevel);
  }

  // --- TFT numbers: 4 times a second (a steady, readable display) ---
  if (now - lastNumMs >= 250) {
    lastNumMs = now;
    char buf[16];
    snprintf(buf, sizeof(buf), "%5.1f", dbLevel);
    dbTftText(0, 0, 0, 2, buf, WHITE, BLACK);
    dbTftText(1, 64, 0, 1, dbWeightA ? "dBA" : "dB ", CYAN, BLACK);
    dbTftText(2, 64, 9, 1, dbSlow ? "Slow" : "Fast", CYAN, BLACK);
    snprintf(buf, sizeof(buf), "Max %5.1f", dbMaxLevel);
    dbTftText(3, 0, 45, 1, buf, YELLOW, BLACK);
    snprintf(buf, sizeof(buf), "Leq %5.1f", dbLeqDb);
    dbTftText(4, 0, 55, 1, buf, GREEN, BLACK);
    dbTftText(5, 66, 45, 1, clipping ? "CLIP" : "    ", RED, BLACK);
  }

  // --- OLED: history graph + stats. The graph gains one column every 200 ms. ---
  bool tick = (now - lastOledMs >= 200);
  if (tick) {
    lastOledMs = now;
    int col = (int)((dbLevel - DB_SCALE_MIN) * DB_GRAPH_H / (DB_SCALE_MAX - DB_SCALE_MIN));
    if (col < 0) col = 0;
    if (col > DB_GRAPH_H) col = DB_GRAPH_H;
    dbHist[dbHistHead] = (uint8_t)col;
    dbHistHead = (dbHistHead + 1) % DB_HIST_W;
  }
  if (tick || oledDirty) {
    oledDirty = false;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 9);
    display.print(F("dB METER"));
    display.setCursor(78, 9);
    display.print(dbWeightA ? "A " : "Z ");
    display.print(dbSlow ? "Slow" : "Fast");

    // history graph, oldest on the left
    int baseY = DB_GRAPH_TOP + DB_GRAPH_H;
    for (int x = 0; x < DB_HIST_W; x++) {
      int h = dbHist[(dbHistHead + x) % DB_HIST_W];
      display.drawFastVLine(x, baseY - h, h + 1, SSD1306_WHITE);
    }

    char line[24];
    display.setCursor(0, 55);
    snprintf(line, sizeof(line), "Pk%6.1f", dbPeakDb);
    display.print(line);
    display.setCursor(66, 55);
    snprintf(line, sizeof(line), "Cal%6.1f", dbCalOffset);
    display.print(line);
    display.display();
  }
}
