// ------------- Countdown Timer ---------------
//
// SETUP     UP / DOWN = change the selected field      LEFT / RIGHT = select minutes / seconds
//           ENTER = start
// RUNNING   ENTER = pause      LEFT = cancel (back to setup)
// PAUSED    ENTER = resume     LEFT = reset (back to setup)
// TIME UP   the alarm beeps for 20 s and the red LED blinks until you stop it: any short press (or the phone
//           app's "Stop alarm") stops both and returns to setup. The beeping stops by itself after 20 s,
//           the LED keeps blinking until the alarm is stopped.
// LEFT (long) = exit from anywhere  (leaving the screen cancels a running countdown)
//
// The time you set is remembered while the sketch is running, so you can restart the same countdown.

#define CD_SET     0
#define CD_RUN     1
#define CD_PAUSED  2
#define CD_DONE    3

#define CD_MAX_MIN     99
#define CD_ALARM_HZ    2000
#define CD_ALARM_MS    20000          // how long the alarm keeps beeping
#define CD_BEEP_PERIOD 500            // one beep every 500 ms ...
#define CD_BEEP_LEN    200            // ... lasting 200 ms
#define CD_LED_PERIOD  500            // the red LED blinks once every 500 ms (250 ms on, 250 ms off)

int   cdMode = CD_SET;
int   cdSetMin = 5;                   // the time that was set
int   cdSetSecs = 0;
int   cdField = 0;                    // 0 = minutes, 1 = seconds
unsigned long cdEndMs = 0;            // millis() when the countdown reaches zero (while running)
unsigned long cdRemainMs = 0;         // time left (while paused)
unsigned long cdAlarmStartMs = 0;
bool  cdBeepOn = false;
bool  cdLedActive = false;            // this screen is using the RGB LED (from the alarm until it is stopped)
bool  cdLedLit = false;               // the red LED is lit at the moment
bool  cdUiInit = false;
bool  cdFrameDrawn = false;           // the frame and the blocks of the current mode are on the colour screen
int   cdShownFlash = -1;              // TIME UP block: 1 shown, 0 hidden
bool  cdOledDirty = true;
unsigned long cdLastTftMs = 0;
int   cdShownField = -1;
char  cdShown[4][16];

// Whole seconds left, rounded up so the display shows 00:01 until the very end
unsigned long cdRemainingSec() {
  unsigned long ms;
  if (cdMode == CD_RUN) {
    long left = (long)(cdEndMs - millis());
    ms = (left > 0) ? (unsigned long)left : 0UL;
  } else if (cdMode == CD_PAUSED) {
    ms = cdRemainMs;
  } else {
    return (unsigned long)cdSetMin * 60UL + (unsigned long)cdSetSecs;
  }
  return (ms + 999UL) / 1000UL;
}

void cdFormat(unsigned long totalSec, char* buf, size_t n) {
  snprintf(buf, n, "%02lu:%02lu", totalSec / 60UL, totalSec % 60UL);
}

void cdBeepStop() {
  if (cdBeepOn) {
    noTone(BEEPER_PIN);
    cdBeepOn = false;
  }
}

// The RGB LED (LEDR / LEDG / LEDB) is active LOW on the Nano 33 BLE: LOW = lit
void cdLedWrite(bool redOn) {
  digitalWrite(LEDR, redOn ? LOW : HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  cdLedLit = redOn;
}

void cdLedStart() {                   // the alarm begins: red on
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  cdLedActive = true;
  cdLedWrite(true);
}

void cdLedStop() {                    // the alarm was stopped, or the screen was left: LED off
  if (cdLedActive) {
    cdLedWrite(false);
    cdLedActive = false;
  }
}

void cdSetMode(int m) {
  cdMode = m;
  cdOledDirty = true;
  cdLastTftMs = 0;
  cdShownField = -1;
  for (int i = 0; i < 4; i++) tftTextForget(cdShown[i], 16);
  cdFrameDrawn = false;                // the colour screen is redrawn (frame and blocks) by cdDrawTft() straight away
  if (m != CD_DONE) {
    cdBeepStop();
    cdLedStop();
  }
  if (m == CD_DONE) {
    cdAlarmStartMs = millis();
    cdLedStart();
  }
}

void cdDrawOled() {
  display.clearDisplay();                                      // (LCARS look, see LcarsOled.ino)
  lcoFrame("COUNTDOWN", cdMode == CD_SET ? "SET" : (cdMode == CD_RUN ? "RUN" : (cdMode == CD_PAUSED ? "PAUSED" : "DONE")));
  if (cdMode == CD_SET) {
    lcoRow(0, "U/D", "CHANGE");
    lcoRow(1, "L/R", cdField == 0 ? "[MIN] SEC" : "MIN [SEC]");
    lcoRow(2, "ENTER", "START");
  } else if (cdMode == CD_RUN) {
    lcoText(0, "RUNNING...");
    lcoRow(2, "ENTER", "PAUSE");
    lcoRow(3, "LEFT", "CANCEL");
  } else if (cdMode == CD_PAUSED) {
    lcoText(0, "PAUSED");
    lcoRow(2, "ENTER", "RESUME");
    lcoRow(3, "LEFT", "RESET");
  } else {
    lcoBlock(24, 14, "TIME UP!", true);
    lcoText(4, "ANY BUTTON = STOP");
  }
  display.display();
}

// The colour screen (LCARS look, see Lcars.ino): the time in big digits with the field being edited underlined, the mode in a block
// along the bottom, the time that was set while it runs, and a flashing TIME UP block when it is over
void cdDrawTft() {
  char buf[16];
  if (!cdFrameDrawn) {                           // a new mode: the frame, and the parts that only depend on the mode
    cdFrameDrawn = true;
    cdShownField = -1;
    cdShownFlash = -1;
    for (int i = 0; i < 4; i++) tftTextForget(cdShown[i], 16);
    lcFrame("COUNTDOWN", false);
    if (cdMode != CD_DONE) {
      tft.fillRect(47, 20, 4, 4, lcPeach());     // the colon between the minutes and the seconds
      tft.fillRect(47, 28, 4, 4, lcPeach());
      if (cdMode == CD_RUN)         lcBigPill(8, 44, 88, 8, lcOk(), "RUNNING", 1);
      else if (cdMode == CD_PAUSED) lcBigPill(8, 44, 88, 8, lcGold(), "PAUSED", 1);
      else                          lcBigPill(8, 44, 88, 8, lcBlue(), "SET TIME", 1);
      if (cdMode != CD_SET) lcPill(8, 54, 20, lcLilac(), "SET");
    }
  }

  if (cdMode == CD_DONE) {
    // flash "TIME UP" twice a second
    bool on = (((millis() - cdAlarmStartMs) / 500UL) % 2UL) == 0UL;
    if ((on ? 1 : 0) != cdShownFlash) {
      cdShownFlash = on ? 1 : 0;
      if (on) lcBigPill(8, 22, 88, 24, lcAlert(), "TIME UP", 2);
      else    tft.fillRect(8, 22, 88, 24, BLACK);
    }
    return;
  }

  unsigned long sec = cdRemainingSec();
  snprintf(buf, sizeof(buf), "%02lu", sec / 60UL);
  tftTextDiff(cdShown[1], 9, 12, 3, buf, lcPeach(), BLACK);          // minutes and seconds, 36 pixels wide each
  snprintf(buf, sizeof(buf), "%02lu", sec % 60UL);
  tftTextDiff(cdShown[2], 55, 12, 3, buf, lcPeach(), BLACK);

  // underline the field being edited
  if (cdMode == CD_SET && cdField != cdShownField) {
    tft.fillRect(8, 38, 88, 3, BLACK);
    if (cdField == 0) tft.fillRect(9, 38, 36, 3, lcOrange());        // minutes digits
    else              tft.fillRect(55, 38, 36, 3, lcOrange());       // seconds digits
    cdShownField = cdField;
  }

  if (cdMode != CD_SET) {
    snprintf(buf, sizeof(buf), "%02d:%02d", cdSetMin, cdSetSecs);
    tftTextDiff(cdShown[3], 32, 54, 1, buf, lcPeach(), BLACK);
  }
}

void CountdownTimer(int pressType) {
  // --- Exit ---
  if (pressType == LONG_PRESS && lastButton == 5) {
    cdBeepStop();
    cdLedStop();
    cdUiInit = false;
    cdMode = CD_SET;
    tft.fillScreen(BLACK);
    currentState = UTIL_MENU;
    return;
  }

  // --- First pass after entering ---
  if (!cdUiInit) {
    cdUiInit = true;
    cdField = 0;
    cdSetMode(CD_SET);
  }

  unsigned long now = millis();

  // --- Reached zero? ---
  if (cdMode == CD_RUN && (long)(cdEndMs - now) <= 0) {
    cdSetMode(CD_DONE);
  }

  // --- Alarm beeping (no delay() calls, so the buttons stay responsive) ---
  if (cdMode == CD_DONE) {
    unsigned long t = now - cdAlarmStartMs;
    bool shouldBeep = (t < CD_ALARM_MS) && ((t % CD_BEEP_PERIOD) < CD_BEEP_LEN);
    if (shouldBeep && !cdBeepOn) { tone(BEEPER_PIN, CD_ALARM_HZ); cdBeepOn = true; }
    if (!shouldBeep && cdBeepOn) cdBeepStop();

    // red LED: blinks until the alarm is stopped (the beeping stops by itself after 20 s, the LED does not)
    bool ledOn = (t % CD_LED_PERIOD) < (CD_LED_PERIOD / 2);
    if (ledOn != cdLedLit) cdLedWrite(ledOn);
  }

  // --- Buttons ---
  if (pressType == SHORT_PRESS) {
    if (cdMode == CD_SET) {
      if (lastButton == 5) { cdField = 0; cdOledDirty = true; }          // Left: minutes
      else if (lastButton == 3) { cdField = 1; cdOledDirty = true; }     // Right: seconds
      else if (lastButton == 2 || lastButton == 4) {                     // Up / Down: change the field
        int step = (lastButton == 2) ? 1 : -1;
        if (cdField == 0) cdSetMin = (cdSetMin + step + CD_MAX_MIN + 1) % (CD_MAX_MIN + 1);
        else cdSetSecs = (cdSetSecs + step + 60) % 60;
        cdLastTftMs = 0;
      } else if (lastButton == 1) {                                      // Enter: start
        if (cdSetMin > 0 || cdSetSecs > 0) {
          cdEndMs = millis() + ((unsigned long)cdSetMin * 60UL + (unsigned long)cdSetSecs) * 1000UL;
          cdSetMode(CD_RUN);
        }
      }

    } else if (cdMode == CD_RUN) {
      if (lastButton == 1) {                                             // Enter: pause
        long left = (long)(cdEndMs - millis());
        cdRemainMs = (left > 0) ? (unsigned long)left : 0UL;
        cdSetMode(CD_PAUSED);
      } else if (lastButton == 5) {                                      // Left: cancel
        cdSetMode(CD_SET);
      }

    } else if (cdMode == CD_PAUSED) {
      if (lastButton == 1) {                                             // Enter: resume
        cdEndMs = millis() + cdRemainMs;
        cdSetMode(CD_RUN);
      } else if (lastButton == 5) {                                      // Left: reset
        cdSetMode(CD_SET);
      }

    } else {                                                             // time up: any button stops the alarm
      cdSetMode(CD_SET);
    }
  }

  // --- Screen updates ---
  now = millis();
  if (now - cdLastTftMs >= 100 || cdOledDirty) {
    cdLastTftMs = now;
    cdDrawTft();
  }
  if (cdOledDirty) {
    cdOledDirty = false;
    cdDrawOled();
  }
}
