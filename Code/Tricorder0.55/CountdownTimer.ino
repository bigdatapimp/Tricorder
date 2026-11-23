// ------------- Countdown Timer ---------------
//
// SETUP     UP / DOWN = change the selected field      LEFT / RIGHT = select minutes / seconds
//           ENTER = start
// RUNNING   ENTER = pause      LEFT = cancel (back to setup)
// PAUSED    ENTER = resume     LEFT = reset (back to setup)
// TIME UP   the alarm beeps for 20 s (or until you press a button); any short press returns to setup
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

int   cdMode = CD_SET;
int   cdSetMin = 5;                   // the time that was set
int   cdSetSecs = 0;
int   cdField = 0;                    // 0 = minutes, 1 = seconds
unsigned long cdEndMs = 0;            // millis() when the countdown reaches zero (while running)
unsigned long cdRemainMs = 0;         // time left (while paused)
unsigned long cdAlarmStartMs = 0;
bool  cdBeepOn = false;
bool  cdUiInit = false;
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

void cdSetMode(int m) {
  cdMode = m;
  cdOledDirty = true;
  cdLastTftMs = 0;
  cdShownField = -1;
  for (int i = 0; i < 4; i++) tftTextForget(cdShown[i], 16);
  tft.fillScreen(BLACK);
  if (m != CD_DONE) cdBeepStop();
  if (m == CD_DONE) cdAlarmStartMs = millis();
}

void cdDrawOled() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 9);
  display.print(F("COUNTDOWN"));
  if (cdMode == CD_SET) {
    display.setCursor(0, 24); display.print(F("U/D = Change"));
    display.setCursor(0, 34); display.print(cdField == 0 ? "L/R = [Min]  Sec" : "L/R =  Min  [Sec]");
    display.setCursor(0, 52); display.print(F("ENTER = Start"));
  } else if (cdMode == CD_RUN) {
    display.setCursor(0, 24); display.print(F("Running..."));
    display.setCursor(0, 44); display.print(F("ENTER = Pause"));
    display.setCursor(0, 54); display.print(F("LEFT  = Cancel"));
  } else if (cdMode == CD_PAUSED) {
    display.setCursor(0, 24); display.print(F("Paused"));
    display.setCursor(0, 44); display.print(F("ENTER = Resume"));
    display.setCursor(0, 54); display.print(F("LEFT  = Reset"));
  } else {
    display.setCursor(0, 24); display.print(F("TIME UP!"));
    display.setCursor(0, 54); display.print(F("Any button = Stop"));
  }
  display.display();
}

void cdDrawTft() {
  char buf[16];
  if (cdMode == CD_DONE) {
    // flash "TIME UP" twice a second
    bool on = (((millis() - cdAlarmStartMs) / 500UL) % 2UL) == 0UL;
    tftTextDiff(cdShown[1], 6, 20, 2, on ? "TIME UP" : "       ", RED, BLACK);
    return;
  }

  const char* label = "SET TIME";
  uint16_t color = CYAN;
  if (cdMode == CD_RUN) { label = "RUNNING "; color = GREEN; }
  else if (cdMode == CD_PAUSED) { label = "PAUSED  "; color = YELLOW; }
  tftTextDiff(cdShown[0], 0, 0, 1, label, color, BLACK);

  cdFormat(cdRemainingSec(), buf, sizeof(buf));
  tftTextDiff(cdShown[1], 3, 14, 3, buf, WHITE, BLACK);        // "MM:SS" at text size 3 is 90 px wide

  // underline the field being edited
  if (cdMode == CD_SET && cdField != cdShownField) {
    tft.fillRect(0, 40, 96, 3, BLACK);
    if (cdField == 0) tft.fillRect(3, 40, 36, 3, CYAN);        // minutes digits
    else              tft.fillRect(57, 40, 36, 3, CYAN);       // seconds digits
    cdShownField = cdField;
  }

  if (cdMode == CD_SET) snprintf(buf, sizeof(buf), "         ");
  else snprintf(buf, sizeof(buf), "Set %02d:%02d", cdSetMin, cdSetSecs);
  tftTextDiff(cdShown[2], 0, 50, 1, buf, CYAN, BLACK);
}

void CountdownTimer(int pressType) {
  // --- Exit ---
  if (pressType == LONG_PRESS && lastButton == 5) {
    cdBeepStop();
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
