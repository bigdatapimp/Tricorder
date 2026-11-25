// ------------- Stopwatch ---------------
//
//   ENTER = start / stop        RIGHT = lap (while running)        DOWN = reset (while stopped)
//   LEFT (long) = exit  (leaving the screen clears the stopwatch)
//
// The TFT shows the running time; the OLED lists the most recent laps (lap number, lap time, total).
// Up to SW_MAX_LAPS laps are kept. Times use millis(), accurate to a few milliseconds.

#define SW_MAX_LAPS 20

bool swRunning = false;
unsigned long swStartMs = 0;           // millis() when the current run segment began
unsigned long swAccumMs = 0;           // time accumulated before that (from earlier start/stop segments)
unsigned long swLapTotals[SW_MAX_LAPS];// total elapsed time at each lap press
int   swLapCount = 0;
bool  swUiInit = false;
bool  swOledDirty = true;
unsigned long swLastTftMs = 0;
char  swShown[4][16];

unsigned long swElapsed() {
  return swAccumMs + (swRunning ? (millis() - swStartMs) : 0UL);
}

// "MM:SS.hh" normally (8 characters, fits the TFT at text size 2); "H:MM:SS" once past an hour
void swFormat(unsigned long ms, char* buf, size_t n) {
  unsigned long totalSec = ms / 1000UL;
  unsigned long h = totalSec / 3600UL;
  unsigned long m = (totalSec / 60UL) % 60UL;
  unsigned long s = totalSec % 60UL;
  unsigned long hh = (ms % 1000UL) / 10UL;
  if (h == 0) snprintf(buf, n, "%02lu:%02lu.%02lu", m, s, hh);
  else snprintf(buf, n, "%lu:%02lu:%02lu", h, m, s);
}

void swReset() {
  swRunning = false;
  swAccumMs = 0;
  swStartMs = 0;
  swLapCount = 0;
}

void swDrawOled() {
  char line[28];
  char a[12], b[12];
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 9);
  display.print(F("STOPWATCH"));
  display.setCursor(84, 9);
  display.print(swRunning ? "RUNNING" : (swElapsed() > 0 ? "STOPPED" : "READY"));

  // the most recent 4 laps, newest at the bottom
  int first = (swLapCount > 4) ? swLapCount - 4 : 0;
  for (int i = first; i < swLapCount; i++) {
    unsigned long split = swLapTotals[i] - (i > 0 ? swLapTotals[i - 1] : 0UL);
    swFormat(split, a, sizeof(a));
    swFormat(swLapTotals[i], b, sizeof(b));
    snprintf(line, sizeof(line), "L%02d %s %s", i + 1, a, b);
    display.setCursor(0, 20 + (i - first) * 8);
    display.print(line);
  }
  if (swLapCount == 0) {
    display.setCursor(0, 28);
    display.print(F("No laps yet"));
  }
  display.setCursor(0, 55);
  display.print(F("E:Run/Stop R:Lap D:Clr"));
  display.display();
}

// The colour screen (LCARS look, see Lcars.ino): the time in big digits (MM:SS with the hundredths small beside it, H:MM:SS once
// past an hour), the number of laps, the last lap, and the state in a block along the bottom
int swShownState = -1;                 // which state block is on the screen: 0 READY, 1 RUNNING, 2 STOPPED (-1 none)
int swShownFmt = -1;                   // 0 = MM:SS + hundredths, 1 = H:MM:SS

void swDrawTftStatic() {
  lcFrame("STOPWATCH", false);
  tft.fillRoundRect(8, 29, 88, 2, 1, lcOrange());
  lcPill(8, 34, 26, lcLilac(), "LAPS");
  lcPill(8, 44, 26, lcBlue(), "LAST");
  swShownState = -1;
  swShownFmt = -1;
}

void swDrawTft(bool force) {
  char buf[16];
  char t[12];
  if (force) {
    for (int i = 0; i < 4; i++) tftTextForget(swShown[i], 16);
  }

  // the state block
  int state = swRunning ? 1 : (swElapsed() > 0 ? 2 : 0);
  if (state != swShownState) {
    swShownState = state;
    if (state == 1)      lcBigPill(8, 54, 88, 8, lcOk(), "RUNNING", 1);
    else if (state == 2) lcBigPill(8, 54, 88, 8, lcAlert(), "STOPPED", 1);
    else                 lcBigPill(8, 54, 88, 8, lcBlue(), "READY", 1);
  }

  // the time
  unsigned long ms = swElapsed();
  unsigned long totalSec = ms / 1000UL;
  unsigned long h = totalSec / 3600UL;
  unsigned long m = (totalSec / 60UL) % 60UL;
  unsigned long s = totalSec % 60UL;
  int fmt = (h > 0) ? 1 : 0;
  if (fmt != swShownFmt) {                       // the layout changes at one hour: wipe the old one
    swShownFmt = fmt;
    tft.fillRect(8, 11, 88, 16, BLACK);
    tftTextForget(swShown[0], 16);
    tftTextForget(swShown[1], 16);
  }
  if (fmt == 0) {
    snprintf(buf, sizeof(buf), "%02lu:%02lu", m, s);
    tftTextDiff(swShown[1], 9, 11, 2, buf, lcPeach(), BLACK);
    snprintf(buf, sizeof(buf), ".%02lu", (ms % 1000UL) / 10UL);
    tftTextDiff(swShown[0], 70, 19, 1, buf, lcPeach(), BLACK);
  } else {
    if (h > 9) { h = 9; m = 59; s = 59; }        // the colour screen has room for one digit of hours
    snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", h, m, s);
    tftTextDiff(swShown[1], 9, 11, 2, buf, lcPeach(), BLACK);
  }

  // the laps
  snprintf(buf, sizeof(buf), "%2d", swLapCount);
  tftTextDiff(swShown[2], 38, 34, 1, buf, lcPeach(), BLACK);
  if (swLapCount > 0) {
    unsigned long split = swLapTotals[swLapCount - 1] - (swLapCount > 1 ? swLapTotals[swLapCount - 2] : 0UL);
    swFormat(split, t, sizeof(t));
    snprintf(buf, sizeof(buf), "%-8s", t);
  } else {
    snprintf(buf, sizeof(buf), "%-8s", "--");
  }
  tftTextDiff(swShown[3], 38, 44, 1, buf, lcPeach(), BLACK);
}

void Stopwatch(int pressType) {
  // --- Exit ---
  if (pressType == LONG_PRESS && lastButton == 5) {
    swReset();
    swUiInit = false;
    tft.fillScreen(BLACK);
    currentState = UTIL_MENU;
    return;
  }

  // --- First pass after entering ---
  if (!swUiInit) {
    swUiInit = true;
    swReset();
    swDrawTftStatic();
    swOledDirty = true;
    swLastTftMs = 0;
    swDrawTft(true);
  }

  // --- Buttons ---
  if (pressType == SHORT_PRESS) {
    if (lastButton == 1) {                       // Enter: start / stop
      if (swRunning) {
        swAccumMs += millis() - swStartMs;
        swRunning = false;
      } else {
        swStartMs = millis();
        swRunning = true;
      }
      swOledDirty = true;
    } else if (lastButton == 3 && swRunning) {   // Right: lap
      if (swLapCount < SW_MAX_LAPS) {
        swLapTotals[swLapCount++] = swElapsed();
        swOledDirty = true;
      }
    } else if (lastButton == 4 && !swRunning) {  // Down: reset
      swReset();
      swOledDirty = true;
    }
  }

  // --- Screen updates ---
  unsigned long now = millis();
  if (now - swLastTftMs >= 50 || swOledDirty) {
    swLastTftMs = now;
    swDrawTft(false);
  }
  if (swOledDirty) {
    swOledDirty = false;
    swDrawOled();
  }
}
