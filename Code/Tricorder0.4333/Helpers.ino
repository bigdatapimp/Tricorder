// --- GPS Time Display Helper ---
// Text on the SSD1331 is slow (every pixel is a separate SPI write). The old version repainted the
// whole clock on every pass through loop(), which made each pass take long enough that button
// presses were missed. Now it repaints only when the GPS time changes, and only the characters
// that actually changed (normally just the seconds digit(s)).

bool clockCacheValid = false;      // false = TFT clock area must be repainted from scratch
char clockShownUtc[16];            // what is currently on screen, one buffer per line
char clockShownCt[16];
char clockShownDate[16];

void invalidateClockCache() {
  clockCacheValid = false;
}

// Print only the characters of 'text' that differ from what is already drawn ('shown')
void drawClockLine(int x, int y, const char* text, char* shown, uint16_t fg, uint16_t bg) {
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

  bool valid = gps.time.isValid() && gps.date.isValid();

  // Repaint from scratch on first use, after a screen change, or when fix status changes
  if (!clockCacheValid || valid != lastValid) {
    tft.fillRect(0, 8, 96, 40, BLACK);   // also removes the old "Searching" text
    memset(clockShownUtc, 0, sizeof(clockShownUtc));
    memset(clockShownCt, 0, sizeof(clockShownCt));
    memset(clockShownDate, 0, sizeof(clockShownDate));
    lastTimeValue = 0xFFFFFFFF;
    lastDateValue = 0xFFFFFFFF;
    lastValid = valid;
    clockCacheValid = true;
    tft.setTextSize(1);

    if (!valid) {
      tft.setTextColor(RED, BLACK);
      tft.setCursor(0, 10);
      tft.print("Searching Sat...");
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

  // Convert UTC -> US Central (including DST)
  time_t local = US_Central.toLocal(utc);

  char line[16];

  // --- UTC Time (top line) ---
  snprintf(line, sizeof(line), "UTC %02d:%02d:%02d",
           (int)gps.time.hour(), (int)gps.time.minute(), (int)gps.time.second());
  drawClockLine(10, 15, line, clockShownUtc, CYAN, BLACK);

  // --- US Central Time (middle line) ---
  snprintf(line, sizeof(line), "CT %02d:%02d:%02d", hour(local), minute(local), second(local));
  drawClockLine(10, 25, line, clockShownCt, GREEN, BLACK);

  // --- Local Date (bottom line) ---
  snprintf(line, sizeof(line), "%02d/%02d/%02d", month(local), day(local), year(local) - 2000);
  drawClockLine(20, 35, line, clockShownDate, BLACK, CYAN);
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

void playClimbingTones() {
  // Play 3 short beeps climbing in pitch
  playTone(440, 80);  // A4
  playTone(554, 80);  // C#5
  playTone(659, 100); // E5
}