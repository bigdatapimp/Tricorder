// ------------- Seismometer ---------------
//
// Measures how hard the Tricorder is being shaken, from the accelerometer, whichever way it is lying. It is a sensitive
// vibration meter, not a calibrated scientific seismometer: the sensor's own noise is about 1 mg, so it shows a nearby
// footstep, a passing truck or the shaking of a real earthquake (strong enough to feel), but not distant small ones.
//
//   accelerometer (about 100 samples a second, in g)
//     -> gravity is estimated with a slow low-pass filter (1 second) and subtracted, so only the shaking is left, in any
//        direction: shake = |acceleration - gravity|, in mg (thousandths of a g)
//     -> the colour screen draws the strongest shake of every quarter second as one column of a seismograph, sweeping from
//        left to right (a mirrored trace, drawn from the centre line up and down)
//     -> the OLED shows the numbers: shaking now (about 1 s RMS), the strongest shake since reset, the noise floor, events
//
// An EVENT is a quarter second whose strongest shake is more than SEIS_EVENT_FACTOR times the noise floor (and above
// SEIS_EVENT_MIN_MG). The noise floor is the slow average of the quiet level, so it follows slow changes in the background
// (the engine of a truck, a fan) and is not raised by an event itself. Events are counted; one lasts at least 3 seconds.
//
// Controls:  UP (short)    more sensitive (the trace's full scale gets smaller)     DOWN (short)  less sensitive
//            RIGHT (short) reset the strongest shake and the events                 ENTER (long)  settle again (after moving it)
//            LEFT (long)   exit
// The sensitivity is the full scale of the seismograph, 2 mg to 500 mg; it does not change the event rule. It also goes to the
// phone (Bluetooth.ino, Seismometer characteristic), which logs the strongest and the average shaking with every row of its data log.
//
// Needs, in the main sketch:   State enum: SEISMOMETER     loop(): case SEISMOMETER: Seismometer(pressType); break;

#define SEIS_RANGE_COUNT   8
#define SEIS_RANGE_DEFAULT 3            // 20 mg
#define SEIS_G_TAU         1.0f         // seconds: gravity is whatever stays the same for about this long
#define SEIS_RMS_TAU       1.0f         // seconds: the "shaking now" average
#define SEIS_FLOOR_TAU     30.0f        // seconds: the noise floor follows the quiet level this slowly
#define SEIS_SETTLE_MS     2000UL       // the first samples after starting (or after ENTER) only teach it gravity
#define SEIS_LEARN_MS      3000UL       // then three seconds to learn the noise floor, with no events
#define SEIS_BLOCK_MS      250UL        // one column of the seismograph, and one event check
#define SEIS_EVENT_FACTOR  5.0f
#define SEIS_EVENT_MIN_MG  5.0f
#define SEIS_EVENT_HOLD_MS 3000UL
#define SEIS_COLS          86           // columns of the seismograph: 86 x 0.25 s = 21.5 s across
#define SEIS_X0            9            // left edge of the seismograph
#define SEIS_MID_Y         41           // its centre line
#define SEIS_HALF_H        12           // pixels above and below the centre line at full scale
#define SEIS_LIVE_MS       1500UL       // "live" for the phone while the screen has produced numbers within this time

// ----------------------- DSP (no display code in this block) -----------------------
// ---- SEIS DSP BEGIN ----
const float seisRangeMg[SEIS_RANGE_COUNT] = {2.0f, 5.0f, 10.0f, 20.0f, 50.0f, 100.0f, 200.0f, 500.0f};
int   seisRange = SEIS_RANGE_DEFAULT;     // index into seisRangeMg: the full scale of the seismograph
float seisGx = 0.0f, seisGy = 0.0f, seisGz = 1.0f;   // gravity estimate, g
bool  seisHaveG = false;
unsigned long seisLastSampleMs = 0;
unsigned long seisSettleUntil = 0;        // before this the readings are not used
unsigned long seisLearnUntil = 0;         // before this the noise floor is being learned and there are no events
float seisMs = 0.0f;                      // mean-square of the shake over about 1 s, mg^2
float seisBlockPeak = 0.0f;               // strongest shake in the current quarter second, mg
float seisSincePeak = 0.0f;               // strongest shake since the last Bluetooth packet, mg
float seisSessionPeak = 0.0f;             // strongest shake since reset, mg
float seisFloor = 0.0f;                   // noise floor (quiet level), mg RMS
unsigned long seisEventUntil = 0;         // an event is active until this time
unsigned int seisEvents = 0;              // events since reset
bool  seisEverLive = false;               // the screen has produced numbers at least once
unsigned long seisLiveMs = 0;             // millis() when the screen last produced numbers

float seisRangeNow() { return seisRangeMg[seisRange]; }

bool seisSettling(unsigned long now) { return now < seisSettleUntil; }

// Still getting ready: settling (gravity) or learning the noise floor. The phone and the screens show "wait" for it.
bool seisWaiting(unsigned long now) { return now < seisLearnUntil || now < seisSettleUntil; }

// Forget what it knows about gravity and start again: used when the screen opens and on a long ENTER
void seisRestart(unsigned long now) {
  seisHaveG = false;
  seisMs = 0.0f;
  seisBlockPeak = 0.0f;
  seisSettleUntil = now + SEIS_SETTLE_MS;
  seisLearnUntil = seisSettleUntil + SEIS_LEARN_MS;
  seisFloor = 0.2f;
}

// Forget the strongest shake and the events
void seisReset() {
  seisSessionPeak = 0.0f;
  seisSincePeak = 0.0f;
  seisEvents = 0;
  seisEventUntil = 0;
}

// One accelerometer sample (in g) taken at 'nowMs'
void seisSample(float ax, float ay, float az, unsigned long nowMs) {
  if (!seisHaveG || nowMs - seisLastSampleMs > 1000UL) {    // first sample, or a long gap: learn gravity again
    seisGx = ax; seisGy = ay; seisGz = az;
    seisHaveG = true;
    seisLastSampleMs = nowMs;
    if (nowMs >= seisSettleUntil) {
      seisSettleUntil = nowMs + SEIS_SETTLE_MS;
      seisLearnUntil = seisSettleUntil + SEIS_LEARN_MS;
    }
    return;
  }
  float dt = (float)(nowMs - seisLastSampleMs) * 0.001f;
  seisLastSampleMs = nowMs;
  if (dt < 0.001f) dt = 0.001f;
  float sx = ax - seisGx, sy = ay - seisGy, sz = az - seisGz;           // shake: what gravity does not explain
  float kg = 1.0f - expf(-dt / SEIS_G_TAU);
  seisGx += kg * sx;
  seisGy += kg * sy;
  seisGz += kg * sz;
  if (seisSettling(nowMs)) return;
  float mg = sqrtf(sx * sx + sy * sy + sz * sz) * 1000.0f;
  if (mg > seisBlockPeak) seisBlockPeak = mg;
  if (mg > seisSincePeak) seisSincePeak = mg;
  if (mg > seisSessionPeak) seisSessionPeak = mg;
  seisMs += (mg * mg - seisMs) * (1.0f - expf(-dt / SEIS_RMS_TAU));
}

// The shaking now: RMS over about the last second, mg
float seisRmsMg() { return sqrtf(seisMs); }

bool seisEventActive(unsigned long now) { return now < seisEventUntil; }

// End of a quarter second: returns its strongest shake (mg) and does the event check and the noise floor
float seisBlockEnd(unsigned long now) {
  float peak = seisBlockPeak;
  seisBlockPeak = 0.0f;
  if (seisSettling(now)) return 0.0f;
  float rms = seisRmsMg();
  if (now < seisLearnUntil) {                              // getting to know the background: the floor is simply the level
    seisFloor = rms;
    if (seisFloor < 0.2f) seisFloor = 0.2f;
    return peak;
  }
  float limit = SEIS_EVENT_FACTOR * seisFloor;
  if (limit < SEIS_EVENT_MIN_MG) limit = SEIS_EVENT_MIN_MG;
  if (peak > limit) {
    if (!seisEventActive(now)) seisEvents++;               // a new event (the last one has ended)
    seisEventUntil = now + SEIS_EVENT_HOLD_MS;
  } else if (!seisEventActive(now)) {                      // the floor follows the quiet level, never an event
    seisFloor += (rms - seisFloor) * (1.0f - expf(-(float)SEIS_BLOCK_MS * 0.001f / SEIS_FLOOR_TAU));
  }
  if (seisFloor < 0.2f) seisFloor = 0.2f;
  return peak;
}

// A shorter range is more sensitive. delta -1 = more sensitive, +1 = less. Returns true if it changed.
bool seisChangeRange(int delta) {
  int r = seisRange + delta;
  if (r < 0 || r >= SEIS_RANGE_COUNT) return false;
  seisRange = r;
  return true;
}

// How many pixels above (and below) the centre line a strongest-shake of 'mg' reaches at the current range, 0 to SEIS_HALF_H
int seisColumnHeight(float mg) {
  float f = mg / seisRangeNow();
  if (f < 0.0f) f = 0.0f;
  if (f > 1.0f) f = 1.0f;
  return (int)(f * SEIS_HALF_H + 0.5f);
}
// ---- SEIS DSP END ----

// ----------------------- Bluetooth (called from Bluetooth.ino) -----------------------
// The 12-byte Seismometer packet, see Tricorder_BLE_Protocol.md:
//   0 flags: bit0 the screen has run (valid), bit1 live (the screen is open and updating), bit2 an event is active, bit3 getting ready
//   1 sensitivity: index 0-7 into the full scales 2, 5, 10, 20, 50, 100, 200, 500 mg
//   2-3 shaking now, RMS, hundredths of a mg (uint16, 655 mg at most)       4-5 strongest shake since the last packet, tenths of a mg
//   6-7 strongest shake since reset, tenths of a mg                        8-9 events since reset (uint16)
//   10-11 noise floor, hundredths of a mg
uint16_t seisU16(float v, float scale) {
  float x = v * scale + 0.5f;
  if (x < 0.0f) x = 0.0f;
  if (x > 65535.0f) x = 65535.0f;
  return (uint16_t)x;
}

void seisPacket(uint8_t* p) {
  unsigned long now = millis();
  bool live = (currentState == SEISMOMETER) && seisEverLive && (now - seisLiveMs) < SEIS_LIVE_MS;
  uint8_t flags = 0;
  if (seisEverLive) flags |= 0x01;
  if (live) flags |= 0x02;
  if (live && seisEventActive(now)) flags |= 0x04;
  if (live && seisWaiting(now)) flags |= 0x08;
  p[0] = flags;
  p[1] = (uint8_t)seisRange;
  uint16_t rms = seisU16(seisRmsMg(), 100.0f);
  uint16_t since = seisU16(seisSincePeak, 10.0f);
  uint16_t best = seisU16(seisSessionPeak, 10.0f);
  uint16_t ev = (seisEvents > 65535U) ? 65535 : (uint16_t)seisEvents;
  uint16_t fl = seisU16(seisFloor, 100.0f);
  p[2] = rms & 0xFF;   p[3] = rms >> 8;
  p[4] = since & 0xFF; p[5] = since >> 8;
  p[6] = best & 0xFF;  p[7] = best >> 8;
  p[8] = ev & 0xFF;    p[9] = ev >> 8;
  p[10] = fl & 0xFF;   p[11] = fl >> 8;
  seisSincePeak = 0.0f;                                       // "since the last packet" starts again
}

// ----------------------- colour screen -----------------------
float seisColPeak[SEIS_COLS];             // the strongest shake of each column, mg (kept so a new range can redraw them)
bool  seisColUsed[SEIS_COLS];
int   seisNextCol = 0;

// The tag in the header: the full scale, at most 4 characters
void seisRangeLabel(char* out) {         // out: at least 5 bytes
  float r = seisRangeNow();
  if (r >= 100.0f) snprintf(out, 5, "%.1fg", r / 1000.0f);   // 0.1g 0.2g 0.5g
  else             snprintf(out, 5, "%dmg", (int)r);
}

uint16_t seisColumnColor(float mg) {
  float f = mg / seisRangeNow();
  if (f >= 0.85f) return lcAlert();
  if (f >= 0.5f)  return lcGold();
  return lcBlue();
}

// One column: cleared, the centre-line pixel, and the mirrored line for its strongest shake
void seisDrawColumn(int i) {
  int x = SEIS_X0 + i;
  tft.fillRect(x, SEIS_MID_Y - SEIS_HALF_H - 1, 1, 2 * SEIS_HALF_H + 3, BLACK);
  tft.drawPixel(x, SEIS_MID_Y, lcDim(lcBlue()));
  if (!seisColUsed[i]) return;
  int h = seisColumnHeight(seisColPeak[i]);
  if (h > 0) tft.fillRect(x, SEIS_MID_Y - h, 1, 2 * h + 1, seisColumnColor(seisColPeak[i]));
  else       tft.drawPixel(x, SEIS_MID_Y, lcBlue());
}

// The sweep marker: the column where the next one will be drawn
void seisDrawSweep(int i) {
  int x = SEIS_X0 + i;
  tft.fillRect(x, SEIS_MID_Y - SEIS_HALF_H - 1, 1, 2 * SEIS_HALF_H + 3, BLACK);
  tft.drawFastVLine(x, SEIS_MID_Y - SEIS_HALF_H - 1, 2 * SEIS_HALF_H + 3, lcDim(lcPeach()));
}

void seisDrawAllColumns() {
  for (int i = 0; i < SEIS_COLS; i++) seisDrawColumn(i);
  seisDrawSweep(seisNextCol);
}

void seisDrawStatic() {
  lcFrame("SEISMO", true);
  char tag[8];
  seisRangeLabel(tag);
  lcSetTag(tag);
  tft.setTextSize(1);
  tft.setTextColor(lcLilac(), BLACK);
  tft.setCursor(72, 10);
  tft.print("mg");
  tft.setCursor(72, 19);
  tft.print("RMS");
  lcPill(9, 56, 14, lcLilac(), "PK");
  seisDrawAllColumns();
}

void Seismometer(int pressType) {
  static bool started = false;
  static bool tftReady = false;
  static unsigned long lastBlockMs = 0, lastOledMs = 0;
  static char shownRms[8], shownPeak[8];
  static int shownEvent = -1;             // -1 nothing yet, 0 QUIET, 1 EVENT, 2 SETTLING

  if (pressType == LONG_PRESS && lastButton == 5) {
    started = false;
    tftReady = false;
    tft.fillScreen(BLACK);
    currentState = GEO_MENU;
    return;
  }

  unsigned long now = millis();
  if (!started) {
    started = true;
    seisRestart(now);
    for (int i = 0; i < SEIS_COLS; i++) seisColUsed[i] = false;
    seisNextCol = 0;
    lastBlockMs = now;
    lastOledMs = 0;
    tftReady = false;
  }

  if (pressType == SHORT_PRESS) {
    if (lastButton == 2 && seisChangeRange(-1)) tftReady = false;         // UP: more sensitive; redraw at the new scale
    else if (lastButton == 4 && seisChangeRange(+1)) tftReady = false;    // DOWN: less sensitive
    else if (lastButton == 3) { seisReset(); shownEvent = -1; }          // RIGHT: reset
  } else if (pressType == LONG_PRESS && lastButton == 1) {
    seisRestart(now);                                                     // ENTER long: settle again
  }

  if (!tftReady) {
    seisDrawStatic();
    tftTextForget(shownRms, sizeof(shownRms));
    tftTextForget(shownPeak, sizeof(shownPeak));
    shownEvent = -1;
    tftReady = true;
  }

  // every accelerometer sample that is waiting
  while (IMU.accelerationAvailable()) {
    float x, y, z;
    IMU.readAcceleration(x, y, z);
    seisSample(x, y, z, millis());
  }

  now = millis();
  if (now - lastBlockMs >= SEIS_BLOCK_MS) {
    lastBlockMs = now;
    float peak = seisBlockEnd(now);
    seisEverLive = true;
    seisLiveMs = now;
    seisColPeak[seisNextCol] = peak;
    seisColUsed[seisNextCol] = !seisSettling(now);
    seisDrawColumn(seisNextCol);
    seisNextCol = (seisNextCol + 1) % SEIS_COLS;
    seisDrawSweep(seisNextCol);

    char buf[8];
    float rms = seisRmsMg();
    snprintf(buf, sizeof(buf), "%5.1f", rms > 999.9f ? 999.9f : rms);
    tftTextDiff(shownRms, 9, 10, 2, buf, seisEventActive(now) ? lcAlert() : lcPeach(), BLACK);
    snprintf(buf, sizeof(buf), "%5.1f", seisSessionPeak > 999.9f ? 999.9f : seisSessionPeak);
    tftTextDiff(shownPeak, 26, 56, 1, buf, lcPeach(), BLACK);

    int es = seisWaiting(now) ? 2 : (seisEventActive(now) ? 1 : 0);
    if (es != shownEvent) {
      shownEvent = es;
      tft.fillRect(64, 56, 32, 8, BLACK);
      if (es == 1)      lcPill(64, 56, 32, lcAlert(), "EVENT");
      else if (es == 2) lcPill(64, 56, 32, lcGold(), "WAIT");
      else              lcPill(64, 56, 32, lcBlue(), "QUIET");
      tftTextForget(shownRms, sizeof(shownRms));                           // the digits change colour with an event
    }
  }

  // the OLED, twice a second (sending a whole OLED frame takes a while, during which no accelerometer samples are read)
  if (now - lastOledMs >= 500) {
    lastOledMs = now;
    char tag[8], v[16];
    seisRangeLabel(tag);
    display.clearDisplay();
    lcoFrame("SEISMOMETER", tag);
    snprintf(v, sizeof(v), "%.1f mg", seisRmsMg());
    lcoRow(0, "NOW", v);
    snprintf(v, sizeof(v), "%.1f mg", seisSessionPeak);
    lcoRow(1, "PEAK", v);
    snprintf(v, sizeof(v), "%.1f mg", seisFloor);
    lcoRow(2, "FLOOR", v);
    snprintf(v, sizeof(v), "%u", seisEvents);
    lcoRow(3, "EVENTS", v);
    if (seisWaiting(now))           lcoBlock(54, 9, "SETTLING...", false);
    else if (seisEventActive(now))  lcoBlock(54, 9, "EVENT!", true);
    else                            lcoBlock(54, 9, "UP/DN=SENS RT=CLR", false);
    display.display();
  }
}
