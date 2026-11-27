// --- The colour screen in the menus (LCARS look, see Lcars.ino) ---
// Every menu (and the Time Zone screen) shows the same panel: the menu's name in the header, the selected item as a big
// block with its place in the list (one segment per item), and the GPS clock: UTC, local time with the zone's label (CST,
// CDT, ...) and the local date. The OLED keeps the full list.
//
// Text on the SSD1331 is slow (every pixel is a separate SPI write). The old version repainted the whole clock on every
// pass through loop(), which made each pass take long enough that button presses were missed. Now the panel is drawn once,
// the clock repaints only when the GPS time changes, and only the characters that actually changed (normally just the
// seconds digit(s)); the selected item repaints only when the cursor moves.

bool clockCacheValid = false;      // false = the whole panel must be drawn from scratch
char clockShownUtc[16];            // what is currently on screen, one buffer per line
char clockShownCt[16];
char clockShownDate[16];

void invalidateClockCache() {
  clockCacheValid = false;
}

// The long names shown for the items of each menu (the OLED lists the short ones)
const char* const lcMainNames[5]  = {"GEOLOGICAL", "ENVIRONMENTAL", "COMMUNICATIONS", "UTILITY", "SYSTEM"};
const char* const lcGeoNames[4]   = {"METAL DETECTOR", "EMF METER", "NAVIGATION", "SEISMOMETER"};
const char* const lcEnvNames[3]   = {"ATMOSPHERIC", "dB METER", "SPECTRUM"};
const char* const lcCommsNames[3] = {"BLUETOOTH", "IR SEND", "IR RECEIVE"};
const char* const lcUtilNames[3]  = {"SHOT TIMER", "STOPWATCH", "COUNTDOWN"};
const char* const lcSysNames[5]   = {"BATTERY", "SYSTEM INFO", "TIME ZONE", "DECLINATION", "REBOOT"};

// Which menu is showing: its header title, the selected item's name, its place in the list, and whether to highlight it
// (the zone in use, in the Time Zone screen). False if the current screen is not one of the menus.
bool lcMenuInfo(const char** title, const char** item, int* index, int* count, bool* highlight) {
  *highlight = false;
  switch (currentState) {
    case MAIN_MENU:  *title = "MAIN MENU"; *count = 5; *index = mainMenuCursor; break;
    case GEO_MENU:   *title = "GEO";       *count = 4; *index = subMenuCursor;  break;
    case ENV_MENU:   *title = "ENV";       *count = 3; *index = subMenuCursor;  break;
    case COMMS_MENU: *title = "COMMS";     *count = 3; *index = subMenuCursor;  break;
    case UTIL_MENU:  *title = "UTIL";      *count = 3; *index = subMenuCursor;  break;
    case SYS_MENU:   *title = "SYS";       *count = 5; *index = subMenuCursor;  break;
    case TIMEZONE:   *title = "TIME ZONE"; *count = tzCountForPanel(); *index = tzCursorForPanel(); break;
    case DECLINATION_SETUP: *title = "COMPASS"; *count = 1; *index = 0; break;
    default:         *title = "CLOCK";     *count = 0; *index = 0; *item = ""; return false;
  }
  if (*index < 0) *index = 0;
  if (*index >= *count) *index = *count - 1;
  switch (currentState) {
    case MAIN_MENU:  *item = lcMainNames[*index];  break;
    case GEO_MENU:   *item = lcGeoNames[*index];   break;
    case ENV_MENU:   *item = lcEnvNames[*index];   break;
    case COMMS_MENU: *item = lcCommsNames[*index]; break;
    case UTIL_MENU:  *item = lcUtilNames[*index];  break;
    case SYS_MENU:   *item = lcSysNames[*index];   break;
    case DECLINATION_SETUP: *item = declinationPanelText(); *highlight = declinationPanelInUse(); break;
    default:         *item = tzPanelName(*index); *highlight = tzInUseForPanel(*index); break;
  }
  return true;
}

// The selected item (a block, green if it is the zone in use) and the strip that shows where it is in the list
void lcDrawMenuItem(const char* item, int index, int count, bool highlight) {
  char name[16];
  strncpy(name, item, 14);
  name[14] = '\0';
  tft.fillRect(8, 11, 88, 16, BLACK);
  lcBigPill(8, 11, 88, 11, highlight ? lcOk() : lcBlue(), name, 1);
  int w = (88 - 2 * (count - 1)) / count;
  for (int i = 0; i < count; i++) {
    tft.fillRect(8 + i * (w + 2), 24, w, 3, (i == index) ? lcOrange() : lcDim(lcLilac()));
  }
}

// Print only the characters of 'text' that differ from what is already drawn ('shown')
void drawClockLine(int x, int y, const char* text, char* shown, uint16_t fg, uint16_t bg) {
  tft.setTextSize(1);
  tft.setTextColor(fg, bg);
  for (int i = 0; text[i] != '\0'; i++) {
    if (shown[i] != text[i]) {
      tft.setCursor(x + i * 6, y);
      tft.write(text[i]);
      shown[i] = text[i];
    }
  }
}

void displayGPSClock() {
  static bool lastValid = false;
  static uint32_t lastTimeValue = 0xFFFFFFFF;
  static uint32_t lastDateValue = 0xFFFFFFFF;
  static int shownIndex = -1, shownCount = -1;
  static bool shownHighlight = false;
  static char shownItem[16];
  static char shownTz[8];

  const char* title;
  const char* item;
  int index, count;
  bool highlight;
  bool haveMenu = lcMenuInfo(&title, &item, &index, &count, &highlight);
  bool valid = gps.time.isValid() && gps.date.isValid();

  // Draw the whole panel from scratch on first use and after a screen change
  if (!clockCacheValid) {
    lcFrame(title, false);
    memset(clockShownUtc, 0, sizeof(clockShownUtc));
    memset(clockShownCt, 0, sizeof(clockShownCt));
    memset(clockShownDate, 0, sizeof(clockShownDate));
    memset(shownTz, 0, sizeof(shownTz));
    shownIndex = -1;
    shownItem[0] = '\0';
    lastValid = !valid;                 // so that the clock part below is drawn too
    lastTimeValue = 0xFFFFFFFF;
    lastDateValue = 0xFFFFFFFF;
    clockCacheValid = true;
  }

  // The selected item, when the cursor moves
  if (haveMenu && (index != shownIndex || count != shownCount || highlight != shownHighlight ||
                   strncmp(item, shownItem, sizeof(shownItem) - 1) != 0)) {
    shownIndex = index;
    shownCount = count;
    shownHighlight = highlight;
    strncpy(shownItem, item, sizeof(shownItem) - 1);
    shownItem[sizeof(shownItem) - 1] = '\0';
    lcDrawMenuItem(item, index, count, highlight);
  }

  // The GPS fix came or went: redraw the clock part
  if (valid != lastValid) {
    lastValid = valid;
    lastTimeValue = 0xFFFFFFFF;
    lastDateValue = 0xFFFFFFFF;
    memset(clockShownUtc, 0, sizeof(clockShownUtc));
    memset(clockShownCt, 0, sizeof(clockShownCt));
    memset(clockShownDate, 0, sizeof(clockShownDate));
    memset(shownTz, 0, sizeof(shownTz));
    tft.fillRect(8, 29, 88, 35, BLACK);        // also removes the old "searching" block
    if (!valid) {
      lcBigPill(8, 38, 88, 11, lcAlert(), "SEARCHING SAT", 1);
    } else {
      lcPill(8, 31, 26, lcLilac(), "UTC");
      lcPill(8, 51, 26, lcBlue(), "DATE");
    }
  }

  if (!valid) return;

  // Nothing new from the GPS? Then nothing to draw.
  uint32_t timeValue = gps.time.value();
  uint32_t dateValue = gps.date.value();
  if (timeValue == lastTimeValue && dateValue == lastDateValue) return;
  lastTimeValue = timeValue;
  lastDateValue = dateValue;

  // Build a tmElements_t structure (the TimeLib way)
  tmElements_t tm;
  tm.Year   = gps.date.year() - 1970; // years since 1970
  tm.Month  = gps.date.month();
  tm.Day    = gps.date.day();
  tm.Hour   = gps.time.hour();
  tm.Minute = gps.time.minute();
  tm.Second = gps.time.second();
  time_t utc = makeTime(tm);           // UTC as a Unix timestamp

  // Convert UTC -> the selected local time zone (including DST)
  const char* tzAbbrev = "";
  time_t local = tzToLocal(utc, &tzAbbrev);

  char line[16];

  // --- UTC time ---
  snprintf(line, sizeof(line), "%02d:%02d:%02d", (int)gps.time.hour(), (int)gps.time.minute(), (int)gps.time.second());
  drawClockLine(38, 31, line, clockShownUtc, lcPeach(), BLACK);

  // --- Local time, with the zone's label (CST, CDT, ...) in its block ---
  if (strncmp(shownTz, tzAbbrev, sizeof(shownTz) - 1) != 0) {
    strncpy(shownTz, tzAbbrev, sizeof(shownTz) - 1);
    shownTz[sizeof(shownTz) - 1] = '\0';
    lcPill(8, 41, 26, lcOk(), tzAbbrev);
  }
  snprintf(line, sizeof(line), "%02d:%02d:%02d", hour(local), minute(local), second(local));
  drawClockLine(38, 41, line, clockShownCt, lcPeach(), BLACK);

  // --- Local date ---
  snprintf(line, sizeof(line), "%02d/%02d/%02d", month(local), day(local), year(local) - 2000);
  drawClockLine(38, 51, line, clockShownDate, lcPeach(), BLACK);
}




// --- Beeper Helper Functions ---

void playTone(int frequency, int duration) {
  // Play a tone on the beeper pin
  tone(BEEPER_PIN, frequency);
  delay(duration);
  noTone(BEEPER_PIN);
  delay(50); // Short pause between notes
}

void playStartupTone() {
  // Play a nice major chord arpeggio (C -> E -> G -> C)
  playTone(65, 100);  // C2 - Click
  playTone(65, 100);  // C2 - Click
  playTone(65, 100);  // C2 - Click
  playTone(294, 400); // D4
  playTone(392, 100);  // G4
  playTone(523, 400);  // C5
  playTone(494, 200);  // B4
  playTone(392, 100); // G4
  playTone(330, 100);  // E4
  playTone(440, 100);  // A4
  playTone(587, 600); // D5
}
