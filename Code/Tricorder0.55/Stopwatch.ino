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

void swDrawTft(bool force) {
  char buf[16];
  char t[12];
  if (force) {                                   // fixed text and layout
    for (int i = 0; i < 4; i++) tftTextForget(swShown[i], 16);
  }
  const char* label = swRunning ? "RUNNING " : (swElapsed() > 0 ? "STOPPED " : "READY   ");
  uint16_t labelColor = swRunning ? GREEN : (swElapsed() > 0 ? RED : CYAN);
  tftTextDiff(swShown[0], 0, 0, 1, label, labelColor, BLACK);

  swFormat(swElapsed(), t, sizeof(t));
  snprintf(buf, sizeof(buf), "%-8s", t);         // pad so a shorter string fully overwrites a longer one
  tftTextDiff(swShown[1], 0, 14, 2, buf, WHITE, BLACK);

  snprintf(buf, sizeof(buf), "Laps %2d  ", swLapCount);
  tftTextDiff(swShown[2], 0, 38, 1, buf, CYAN, BLACK);
  if (swLapCount > 0) {
    unsigned long split = swLapTotals[swLapCount - 1] - (swLapCount > 1 ? swLapTotals[swLapCount - 2] : 0UL);
    swFormat(split, t, sizeof(t));
    snprintf(buf, sizeof(buf), "Last %-8s", t);
  } else {
    snprintf(buf, sizeof(buf), "Last %-8s", "--");
  }
  tftTextDiff(swShown[3], 0, 48, 1, buf, YELLOW, BLACK);
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
    tft.fillScreen(BLACK);
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
