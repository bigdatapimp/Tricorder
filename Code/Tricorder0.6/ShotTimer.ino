// ------------- Shot Timer ---------------
//
// ENTER starts a countdown (2-8 s, adjustable). At zero a short beep sounds and the timer starts. Every loud
// sound (a shot, or a clap for dry-fire practice) is recorded with its time. Press ENTER again to stop, then
// the results list shows, for each sound: its time, the split (time since the previous one) and the total
// (= the time of the last sound).
//
//   SETUP     UP / DOWN = countdown +/- 1 s      LEFT / RIGHT = sensitivity - / +      ENTER = start
//             (a "HIT!" flash and the Pk% reading let you test the sensitivity before starting)
//   COUNTDOWN LEFT = cancel
//   TIMING    ENTER = stop and show results
//   RESULTS   UP / DOWN = scroll      ENTER = new run      LEFT = back to setup
//   LEFT (long) = exit from anywhere
//
// TIMING ACCURACY: the audio arrives in 16 ms frames. Each frame is time-stamped to the microsecond when the
// microphone callback delivers it, and a shot's time is worked out from its exact position inside the frame,
// so slow screen updates in the main loop do NOT affect the times. Overall accuracy is about +/- 2 ms
// (microphone and decimation-filter delay, and callback timing).
//
// The start beep is loud right next to the microphone, so sound is ignored until SHOT_BLANK_MS after the beep
// starts (a real shot can't be fired that fast). After each shot there is a short dead time and the signal
// must fall quiet again before the next shot counts, so echoes and ringing aren't counted as extra shots.

#define SHOT_MAX          99        // shots stored per run
#define SHOT_DELAY_MIN    2         // countdown range, seconds
#define SHOT_DELAY_MAX    8
#define SHOT_BEEP_HZ      2000      // start beep
#define SHOT_BEEP_MS      200
#define SHOT_BLANK_MS     260       // ignore sound this long after the beep starts
#define SHOT_DEADTIME_MS  80        // minimum time between two registered shots
#define SHOT_REARM_SAMPLES 32       // signal must stay below the threshold this long (2 ms) before the next shot
#define SHOT_SENS_LEVELS  8
#define SHOT_RING_FRAMES  8         // 8 x 16 ms of audio queued between the callback and the screen code

// UI modes
#define ST_SETUP      0
#define ST_COUNTDOWN  1
#define ST_RUN        2
#define ST_RESULTS    3

// ======================= audio queue with timestamps (filled by the PDM callback) =======================
short shotRing[SHOT_RING_FRAMES][256];
volatile int shotRingLen[SHOT_RING_FRAMES];
volatile uint32_t shotRingUs[SHOT_RING_FRAMES];   // micros() when the frame was delivered (= time of its last sample)
volatile uint8_t shotRingHead = 0;                // advanced only by the PDM callback
volatile uint8_t shotRingTail = 0;                // advanced only by the shot timer
volatile bool shotCapture = false;                // only queue audio while the shot timer screen is open

// Called from onPDMdata() (interrupt context)
void shotRingPush(const short* data, int n) {
  if (!shotCapture) return;
  uint32_t us = micros();
  uint8_t next = (shotRingHead + 1) % SHOT_RING_FRAMES;
  if (next == shotRingTail) return;               // queue full: drop the frame
  if (n > 256) n = 256;
  memcpy(shotRing[shotRingHead], data, n * sizeof(short));
  shotRingLen[shotRingHead] = n;
  shotRingUs[shotRingHead] = us;
  shotRingHead = next;
}

// ======================= detector (no display code in this block) =======================
// ---- DETECTOR BEGIN ----
// Peak level (in 16-bit counts) that counts as a "loud sound". Higher sensitivity number = lower threshold.
const int shotThresholds[SHOT_SENS_LEVELS] = {28000, 22000, 16000, 12000, 8000, 5500, 3500, 2200};
int   shotSens = 4;                   // 1..8

float shotHpX = 0.0, shotHpY = 0.0;   // DC blocker state
uint32_t shotStartUs = 0;             // micros() at the start beep: time zero
bool  shotArmed = false;              // true while timing
uint32_t shotTimes[SHOT_MAX];         // each shot, in microseconds after the start beep
int   shotCount = 0;
bool  shotOverflow = false;           // more than SHOT_MAX sounds
bool  shotHaveLast = false;
int32_t shotLastRel = 0;
long  shotQuietRun = 100000;          // samples in a row below the threshold
int   shotPeakHold = 0;               // loudest sample seen since the screen last read it
bool  shotMonitorHit = false;         // a loud sound was heard while not timing (sensitivity test)

void shotResetRun() {
  shotCount = 0;
  shotOverflow = false;
  shotHaveLast = false;
  shotLastRel = 0;
}

// Process one frame. frameEndUs = micros() when the frame was delivered (time of its last sample).
void shotProcessFrame(const short* d, int n, uint32_t frameEndUs) {
  int threshold = shotThresholds[shotSens - 1];
  const int32_t blankRel = (int32_t)SHOT_BLANK_MS * 1000L;
  const int32_t deadRel = (int32_t)SHOT_DEADTIME_MS * 1000L;

  for (int i = 0; i < n; i++) {
    float x = d[i];
    float y = x - shotHpX + 0.995f * shotHpY;      // remove the microphone's DC offset
    shotHpX = x;
    shotHpY = y;

    int a = (int)fabsf(y);
    if (a > shotPeakHold) shotPeakHold = a;

    if (a >= threshold) {
      if (shotQuietRun >= SHOT_REARM_SAMPLES) {    // a new sound, not the tail of the previous one
        if (shotArmed) {
          // exact time of this sample: 62.5 us per sample, counting back from the end of the frame
          uint32_t sampleUs = frameEndUs - (((uint32_t)(n - 1 - i)) * 125UL) / 2UL;
          int32_t rel = (int32_t)(sampleUs - shotStartUs);
          if (rel >= blankRel && (!shotHaveLast || (rel - shotLastRel) >= deadRel)) {
            if (shotCount < SHOT_MAX) shotTimes[shotCount++] = (uint32_t)rel;
            else shotOverflow = true;
            shotHaveLast = true;
            shotLastRel = rel;
          }
        } else {
          shotMonitorHit = true;
        }
      }
      shotQuietRun = 0;
    } else if (shotQuietRun < 100000) {
      shotQuietRun++;
    }
  }
}
// ---- DETECTOR END ----

// ======================= screen =======================
int   shotMode = ST_SETUP;
int   shotDelaySec = 5;
unsigned long shotCountdownEndMs = 0;
int   shotShownRemain = -1;
unsigned long shotBeepOffMs = 0;
bool  shotBeepOn = false;
bool  shotUiInit = false;
bool  shotOledDirty = true;
unsigned long shotLastTftMs = 0;
unsigned long shotLastOledMs = 0;
unsigned long shotHitUntilMs = 0;
int   shotScroll = 0;
int   shotShownCount = -1;
char  shotShown[5][16];              // what is drawn on the TFT, one buffer per text field
bool  shotFrameDrawn = false;        // the frame and the blocks of the current mode are on the colour screen
int   shotShownHit = -1;             // HIT! block: 1 shown, 0 hidden

void shotSetMode(int m) {
  shotMode = m;
  shotFrameDrawn = false;              // the colour screen is redrawn (frame and blocks) by shotDrawTft() straight away
  for (int i = 0; i < 5; i++) tftTextForget(shotShown[i], 16);
  shotOledDirty = true;
  shotLastTftMs = 0;
  shotLastOledMs = 0;
  shotShownCount = -1;
  if (m == ST_SETUP) {
    shotArmed = false;
  } else if (m == ST_COUNTDOWN) {
    shotArmed = false;
    shotResetRun();
    shotShownRemain = -1;
    shotCountdownEndMs = millis() + (unsigned long)shotDelaySec * 1000UL;
  } else if (m == ST_RESULTS) {
    shotArmed = false;
    shotScroll = 0;
  }
}

void shotBeepStop() {
  if (shotBeepOn) {
    noTone(BEEPER_PIN);
    shotBeepOn = false;
  }
}

// seconds with hundredths from microseconds
double shotSecs(uint32_t us) {
  return us / 1000000.0;
}

void shotDrawOled() {
  char line[24];
  display.clearDisplay();                                      // (LCARS look, see LcarsOled.ino)

  if (shotMode == ST_SETUP) {
    int pkPct = shotPeakHold * 100 / 32768;
    shotPeakHold = 0;
    lcoFrame("SHOT TIMER", "SETUP");
    snprintf(line, sizeof(line), "%d s  U/D", shotDelaySec);
    lcoRow(0, "DELAY", line);
    snprintf(line, sizeof(line), "%d/%d  L/R", shotSens, SHOT_SENS_LEVELS);
    lcoRow(1, "SENS", line);
    snprintf(line, sizeof(line), "%d%%  PK %d%%", shotThresholds[shotSens - 1] * 100 / 32768, pkPct);
    lcoRow(2, "THR", line);
    lcoBlock(47, 10, "ENTER = START", true);

  } else if (shotMode == ST_COUNTDOWN) {
    lcoFrame("SHOT TIMER", "READY");
    lcoText(0, "GET READY...");
    snprintf(line, sizeof(line), "%d s", shotShownRemain);
    lcoRow(1, "BEEP IN", line);
    lcoBlock(47, 10, "LEFT = CANCEL", false);

  } else if (shotMode == ST_RUN) {
    lcoFrame("SHOT TIMER", "TIMING");
    snprintf(line, sizeof(line), "%d", shotCount);
    lcoRow(0, "SHOTS", line);
    if (shotCount > 0) snprintf(line, sizeof(line), "%.2f s", shotSecs(shotTimes[shotCount - 1]));
    else snprintf(line, sizeof(line), "--");
    lcoRow(1, "LAST", line);
    lcoBlock(47, 10, "ENTER = STOP", true);

  } else {   // results
    lcoFrame("SHOTS", "TIME/SPLIT");
    if (shotCount == 0) lcoText(1, "NO SHOTS REGISTERED");
    for (int r = 0; r < 4; r++) {
      int i = shotScroll + r;
      if (i >= shotCount) break;
      double t = shotSecs(shotTimes[i]);
      double s = (i == 0) ? t : shotSecs(shotTimes[i] - shotTimes[i - 1]);
      snprintf(line, sizeof(line), "%2d %7.2f %7.2f", i + 1, t, s);
      lcoText(r, line);
    }
    lcoText(4, "U/D:SCROLL E:AGAIN");
  }
  display.display();
}

// The colour screen (LCARS look, see Lcars.ino): the mode in a block, the time or the countdown in big digits, the shots and the
// last (or average) shot time; the HIT! block flashes in the setup when a loud sound is heard
void shotDrawTft() {
  char buf[16];
  if (!shotFrameDrawn) {                         // a new mode: the frame, and the blocks that only depend on the mode
    shotFrameDrawn = true;
    shotShownHit = -1;
    lcFrame("SHOT TIMER", false);
    if (shotMode == ST_SETUP) {
      lcBigPill(8, 12, 88, 18, lcOk(), "READY", 2);
    } else if (shotMode == ST_COUNTDOWN) {
      lcBigPill(8, 11, 88, 9, lcGold(), "GET READY", 1);
    } else if (shotMode == ST_RUN) {
      lcBigPill(8, 11, 88, 9, lcAlert(), "TIMING", 1);
      lcPill(8, 42, 32, lcLilac(), "SHOTS");
      lcPill(8, 52, 26, lcBlue(), "LAST");
    } else {
      lcBigPill(8, 11, 88, 9, lcOk(), "TOTAL", 1);
      lcPill(8, 42, 32, lcLilac(), "SHOTS");
      lcPill(8, 52, 26, lcBlue(), "AVG");
    }
  }

  if (shotMode == ST_SETUP) {
    bool hit = (millis() < shotHitUntilMs);
    if ((hit ? 1 : 0) != shotShownHit) {
      shotShownHit = hit ? 1 : 0;
      if (hit) lcBigPill(8, 36, 88, 18, lcAlert(), "HIT!", 2);
      else     tft.fillRect(8, 36, 88, 18, BLACK);
    }

  } else if (shotMode == ST_COUNTDOWN) {
    snprintf(buf, sizeof(buf), "%d", shotShownRemain);                // one digit (the delay is 2 to 8 seconds), 30 pixels wide
    tftTextDiff(shotShown[1], 37, 22, 5, buf, lcPeach(), BLACK);

  } else if (shotMode == ST_RUN) {
    snprintf(buf, sizeof(buf), "%6.2f", shotSecs(micros() - shotStartUs));
    tftTextDiff(shotShown[1], 9, 22, 2, buf, lcPeach(), BLACK);
    snprintf(buf, sizeof(buf), "%2d", shotCount);
    tftTextDiff(shotShown[2], 44, 42, 1, buf, lcPeach(), BLACK);
    if (shotCount > 0) snprintf(buf, sizeof(buf), "%6.2f", shotSecs(shotTimes[shotCount - 1]));
    else snprintf(buf, sizeof(buf), "   ---");
    tftTextDiff(shotShown[3], 38, 52, 1, buf, lcPeach(), BLACK);

  } else {   // results
    uint32_t total = (shotCount > 0) ? shotTimes[shotCount - 1] : 0;
    snprintf(buf, sizeof(buf), "%6.2f", shotSecs(total));
    tftTextDiff(shotShown[1], 9, 22, 2, buf, lcPeach(), BLACK);
    snprintf(buf, sizeof(buf), "%2d%s", shotCount, shotOverflow ? "+" : " ");
    tftTextDiff(shotShown[2], 44, 42, 1, buf, lcPeach(), BLACK);
    if (shotCount > 0) snprintf(buf, sizeof(buf), "%6.2f", shotSecs(total / shotCount));
    else snprintf(buf, sizeof(buf), "   ---");
    tftTextDiff(shotShown[3], 38, 52, 1, buf, lcPeach(), BLACK);
  }
}

void ShotTimer(int pressType) {
  // --- Exit ---
  if (pressType == LONG_PRESS && lastButton == 5) {
    shotBeepStop();
    shotCapture = false;
    shotArmed = false;
    shotUiInit = false;
    tft.fillScreen(BLACK);
    currentState = UTIL_MENU;
    return;
  }

  // --- First pass after entering ---
  if (!shotUiInit) {
    shotUiInit = true;
    shotCapture = true;
    shotRingTail = shotRingHead;                    // start with an empty queue
    shotHpX = shotHpY = 0.0;
    shotQuietRun = 100000;
    shotPeakHold = 0;
    shotMonitorHit = false;
    shotHitUntilMs = 0;
    shotSetMode(ST_SETUP);
  }

  // --- Process all queued audio first, so a shot just before a button press is never missed ---
  while (shotRingTail != shotRingHead) {
    uint8_t t = shotRingTail;
    shotProcessFrame(shotRing[t], shotRingLen[t], shotRingUs[t]);
    shotRingTail = (t + 1) % SHOT_RING_FRAMES;
  }
  unsigned long now = millis();

  // --- Start beep off ---
  if (shotBeepOn && now >= shotBeepOffMs) shotBeepStop();

  // --- Countdown reaching zero: this is the time-critical moment, so it comes before any drawing ---
  if (shotMode == ST_COUNTDOWN && (long)(shotCountdownEndMs - now) <= 0) {
    shotStartUs = micros();                         // time zero
    shotArmed = true;
    tone(BEEPER_PIN, SHOT_BEEP_HZ);
    shotBeepOn = true;
    shotBeepOffMs = millis() + SHOT_BEEP_MS;
    shotSetMode(ST_RUN);
    shotArmed = true;                               // shotSetMode(ST_RUN) leaves this alone, but be explicit
    now = millis();
  }

  // --- Buttons ---
  if (pressType == SHORT_PRESS) {
    if (shotMode == ST_SETUP) {
      if (lastButton == 2 && shotDelaySec < SHOT_DELAY_MAX) { shotDelaySec++; shotOledDirty = true; }
      else if (lastButton == 4 && shotDelaySec > SHOT_DELAY_MIN) { shotDelaySec--; shotOledDirty = true; }
      else if (lastButton == 3 && shotSens < SHOT_SENS_LEVELS) { shotSens++; shotOledDirty = true; }
      else if (lastButton == 5 && shotSens > 1) { shotSens--; shotOledDirty = true; }
      else if (lastButton == 1) { shotSetMode(ST_COUNTDOWN); }

    } else if (shotMode == ST_COUNTDOWN) {
      if (lastButton == 5) shotSetMode(ST_SETUP);

    } else if (shotMode == ST_RUN) {
      if (lastButton == 1) {                        // stop
        shotArmed = false;
        shotBeepStop();
        shotSetMode(ST_RESULTS);
      }

    } else {                                        // results
      int maxScroll = (shotCount > 4) ? shotCount - 4 : 0;
      if (lastButton == 2 && shotScroll > 0) { shotScroll--; shotOledDirty = true; }
      else if (lastButton == 4 && shotScroll < maxScroll) { shotScroll++; shotOledDirty = true; }
      else if (lastButton == 1) shotSetMode(ST_COUNTDOWN);   // another run straight away
      else if (lastButton == 5) shotSetMode(ST_SETUP);
    }
  }
  now = millis();

  // --- Sensitivity test: flash HIT! when a loud sound is heard in setup ---
  if (shotMonitorHit) {
    shotMonitorHit = false;
    shotHitUntilMs = now + 300;
  }

  // --- Screen updates ---
  if (shotMode == ST_SETUP) {
    if (now - shotLastTftMs >= 50) { shotLastTftMs = now; shotDrawTft(); }
    if (shotOledDirty || now - shotLastOledMs >= 250) { shotOledDirty = false; shotLastOledMs = now; shotDrawOled(); }

  } else if (shotMode == ST_COUNTDOWN) {
    int remain = (int)((shotCountdownEndMs - now + 999UL) / 1000UL);
    if (remain < 0) remain = 0;
    if (remain != shotShownRemain) {
      shotShownRemain = remain;
      shotDrawTft();
      shotDrawOled();
    }

  } else if (shotMode == ST_RUN) {
    if (now - shotLastTftMs >= 100) { shotLastTftMs = now; shotDrawTft(); }
    if (shotOledDirty || shotCount != shotShownCount) {
      shotShownCount = shotCount;
      shotOledDirty = false;
      shotDrawOled();
    }

  } else {
    if (shotOledDirty) {
      shotOledDirty = false;
      shotDrawTft();
      shotDrawOled();
    }
  }
}
