// --- GPS Time Display Helper ---
void displayGPSClock() {
  tft.setTextSize(1); // Small text fits best on 96x64 screen
  
  // Check if time and date are valid
  if (gps.time.isValid() && gps.date.isValid()) {
    
    // Build a tmElements_t structure (the TimeLib way)
    tmElements_t tm;
    tm.Year   = gps.date.year() - 1970; // years since 1970
    tm.Month  = gps.date.month();
    tm.Day    = gps.date.day();
    tm.Hour   = gps.time.hour();
    tm.Minute = gps.time.minute();
    tm.Second = gps.time.second();
    time_t utc = makeTime(tm);           // ← UTC as a Unix timestamp

    // Convert UTC → US Central (including DST)
    time_t local = US_Central.toLocal(utc);

    // Extract the local time/date components
    int locHour   = hour(local);
    int locMinute = minute(local);
    int locSecond = second(local);
    int locDay    = day(local);
    int locMonth  = month(local);
    int locYear   = year(local);

    // --- UTC Time (top line) ---
    tft.setCursor(10, 15);
    tft.setTextColor(CYAN, BLACK);
    tft.print("UTC ");
    if (gps.time.hour() < 10) tft.print('0');
    tft.print(gps.time.hour());
    tft.print(':');
    if (gps.time.minute() < 10) tft.print('0');
    tft.print(gps.time.minute());
    tft.print(':');
    if (gps.time.second() < 10) tft.print('0');
    tft.println(gps.time.second());

    // --- US Central Time (middle line) ---
    tft.setCursor(10, 25);
    tft.setTextColor(GREEN, BLACK); // Green text for CT
    tft.print("CT ");
    if (locHour < 10) tft.print('0');
    tft.print(locHour);
    tft.print(':');
    if (locMinute < 10) tft.print('0');
    tft.print(locMinute);
    tft.print(':');
    if (locSecond < 10) tft.print('0');
    tft.println(locSecond);

    // --- Local Date (bottom line) ---
    tft.setCursor(20, 35);
    tft.setTextColor(BLACK, CYAN);
    if (locMonth < 10) tft.print('0');
    tft.print(locMonth);
    tft.print('/');
    if (locDay < 10) tft.print('0');
    tft.print(locDay);
    tft.print('/');
    tft.print(locYear - 2000); // Show 2-digit year

  } else {
    // If GPS hasn't fixed yet
    tft.setCursor(10, 10);
    tft.setTextColor(RED, BLACK);
    tft.print("Searching Sat...");
  }
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