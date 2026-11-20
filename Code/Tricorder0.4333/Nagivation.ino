// ----------- Navigation code ---------
//
// Heading = calibrated magnetometer, tilt-compensated using the accelerometer, then smoothed.
//
// What was wrong with the old version (why the compass "lagged and returned to the same direction"):
//  1. The magnetometer calibration was computed but never applied. A hard-iron offset bigger than the
//     horizontal part of the Earth's field makes the measured direction barely change as you turn.
//  2. When tilted, the gyro/magnetometer blend used alpha = 1.0, i.e. ZERO magnetometer correction,
//     so heading was pure gyro integration (which drifts), and when level it only corrected 2% per pass.
//  3. The gyro was integrated from a single sample per (slow) loop pass, and with a sign that doesn't
//     match a clockwise heading.
//  4. The smoothing filter averaged the raw numbers, so crossing 359 -> 0 swept the needle the long way round.
//  5. Every pass redrew the whole TFT and OLED, so the loop was slow and the filter lagged for seconds.

void Navigation(int pressType) {
  static bool initialized = false;            // false = (re)draw the static parts of the screen
  static float ax = 0.0, ay = 0.0, az = 1.0;  // low-pass filtered accelerometer (g)
  static float displayHeading = 0.0;
  static bool haveHeading = false;
  static unsigned long lastHeadingMs = 0;
  static int drawnHeading = -1;
  static unsigned long lastOledMs = 0;
  static unsigned long lastSpeedMs = 0;
#ifdef COMPASS_DEBUG
  static unsigned long lastDebugMs = 0;
#endif

  // --- Handle Button Inputs ---
  if (pressType == LONG_PRESS) {
    if (lastButton == 5) { // Left: Exit to Menu
      tft.fillScreen(BLACK);
      initialized = false;
      haveHeading = false;
      currentState = GEO_MENU;
      return;
    }
    if (lastButton == 1) { // Enter: Calibrate Magnetometer
      calibrateMagnetometer();
      initialized = false;   // calibration took over the OLED, redraw everything
      haveHeading = false;
      return;
    }
    if (lastButton == 3) { // Right: Toggle DMS / Decimal Degrees
      useDMS = !useDMS;
      lastOledMs = 0;        // refresh the OLED right away
    }
    if (lastButton == 2) { // UP: Toggle MPH/KPH
      useMPH = !useMPH;
      lastSpeedMs = 0;
    }
    if (lastButton == 4) { // DOWN: Toggle Feet/Meters
      useFeet = !useFeet;
      lastOledMs = 0;
    }
  }

  // --- Read GPS data ---
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // --- First pass after entering: draw the parts of the TFT that never change ---
  if (!initialized) {
    initialized = true;
    haveHeading = false;
    drawnHeading = -1;
    lastOledMs = 0;
    lastSpeedMs = 0;

    tft.fillScreen(BLACK);
    tft.drawCircle(CENTER_X, CENTER_Y, COMPASS_RADIUS, MAGENTA);
    tft.setTextSize(1);
    tft.setCursor(CENTER_X - 3, CENTER_Y - COMPASS_RADIUS - 8);
    tft.setTextColor(RED);
    tft.print("N");
    tft.setTextColor(BLUE2);
    tft.setCursor(CENTER_X - 3, CENTER_Y + COMPASS_RADIUS + 3);
    tft.print("S");
    tft.setCursor(CENTER_X - COMPASS_RADIUS - 8, CENTER_Y - 3);
    tft.print("W");
    tft.setCursor(CENTER_X + COMPASS_RADIUS + 2, CENTER_Y - 3);
    tft.print("E");
  }

  // --- Accelerometer: drain everything that's waiting into a smoothed estimate of "up" ---
  while (IMU.accelerationAvailable()) {
    float x, y, z;
    IMU.readAcceleration(x, y, z);
    ax += ACCEL_ALPHA * (x - ax);
    ay += ACCEL_ALPHA * (y - ay);
    az += ACCEL_ALPHA * (z - az);
  }

  // Level check (same thresholds as before)
  float totalAccel = sqrt(ax * ax + ay * ay + az * az);
  float pitchDeg = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float rollDeg  = atan2(ay, az) * 180.0 / PI;
  isLevel = (totalAccel > 0.9 && totalAccel < 1.1) && (fabs(pitchDeg) < 20.0) && (fabs(rollDeg) < 20.0);

  // --- Magnetometer: new sample? ---
  float mx, my, mz;
  if (readMagAxes(mx, my, mz) && totalAccel > 0.3) {
    applyMagCal(mx, my, mz);

    // "Up" unit vector. The accelerometer reads +1 g pointing up when the device is at rest.
    float ux = ax / totalAccel, uy = ay / totalAccel, uz = az / totalAccel;

    // East = magnetic field x up   (horizontal, perpendicular to magnetic north)
    float ex = my * uz - mz * uy;
    float ey = mz * ux - mx * uz;
    float ez = mx * uy - my * ux;
    float eLen = sqrt(ex * ex + ey * ey + ez * ez);

    if (eLen > 1.0) {   // field must have a usable horizontal part
      ex /= eLen; ey /= eLen; ez /= eLen;

      // North = up x East
      float nx = uy * ez - uz * ey;
      float ny = uz * ex - ux * ez;
      float nz = ux * ey - uy * ex;

      // Heading of the device's "front" axis, clockwise from magnetic north
      float fE = FWD_X * ex + FWD_Y * ey + FWD_Z * ez;
      float fN = FWD_X * nx + FWD_Y * ny + FWD_Z * nz;
      float heading = atan2(fE, fN) * 180.0 / PI;
      heading += DECLINATION;
      while (heading < 0) heading += 360.0;
      while (heading >= 360.0) heading -= 360.0;

      // --- Smoothing: shortest-way-round, time based (so it doesn't depend on how fast the loop runs) ---
      unsigned long now = millis();
      if (!haveHeading) {
        displayHeading = heading;
        haveHeading = true;
      } else {
        float dt = (now - lastHeadingMs) / 1000.0;
        float gain = 1.0 - exp(-dt / COMPASS_TAU);
        float diff = heading - displayHeading;
        if (diff > 180.0) diff -= 360.0;
        if (diff < -180.0) diff += 360.0;
        displayHeading += gain * diff;
        while (displayHeading < 0) displayHeading += 360.0;
        while (displayHeading >= 360.0) displayHeading -= 360.0;
      }
      lastHeadingMs = now;

#ifdef COMPASS_DEBUG
      // Consistency test: with a correct frame + calibration, "field . up" stays about constant and
      // NEGATIVE (field points down in the northern hemisphere) and "|B|" stays constant,
      // however you rotate or tilt the device. Turning clockwise must make the heading increase.
      if (now - lastDebugMs > 300) {
        lastDebugMs = now;
        Serial.print(F("raw-heading ")); Serial.print(heading, 1);
        Serial.print(F("  B.up ")); Serial.print(mx * ux + my * uy + mz * uz, 1);
        Serial.print(F("  |B| "));  Serial.print(sqrt(mx * mx + my * my + mz * mz), 1);
        Serial.print(F("  tilt p/r ")); Serial.print(pitchDeg, 0); Serial.print(F("/")); Serial.println(rollDeg, 0);
      }
#endif
    }
  }

  // --- Draw needle + heading text, only when the whole-degree heading changed ---
  if (haveHeading) {
    int h = ((int)(displayHeading + 0.5)) % 360;
    if (h != drawnHeading) {
      drawnHeading = h;

      // Clear just the needle area, then draw the needle
      tft.fillRect(CENTER_X - NEEDLE_LENGTH, CENTER_Y - NEEDLE_LENGTH,
                   NEEDLE_LENGTH * 2 + 1, NEEDLE_LENGTH * 2 + 1, BLACK);
      float rad = displayHeading * PI / 180.0;
      int endX = CENTER_X + NEEDLE_LENGTH * sin(rad);
      int endY = CENTER_Y - NEEDLE_LENGTH * cos(rad);
      tft.drawLine(CENTER_X, CENTER_Y, endX, endY, ORANGE);

      // Heading text: fixed width so shorter numbers fully overwrite longer ones
      char buffer[12];
      snprintf(buffer, sizeof(buffer), "Hdg:%3d", h);
      tft.setTextSize(1);
      tft.setTextColor(BLUE, BLACK);
      tft.setCursor(0, 45);
      tft.print(buffer);
      tft.print((char)247); // Degree symbol
    }
  }

  // --- GPS speed (TFT): GPS only updates once a second, so redraw once a second ---
  if (millis() - lastSpeedMs >= 1000) {
    lastSpeedMs = millis();
    char buffer[16];
    if (gps.speed.isValid()) {
      snprintf(buffer, sizeof(buffer), "Spd:%5.1f %s",
               useMPH ? gps.speed.mph() : gps.speed.kmph(), useMPH ? "Mph" : "Kph");
    } else {
      snprintf(buffer, sizeof(buffer), "Spd:%-9s", "---");
    }
    tft.setTextSize(1);
    tft.setTextColor(CYAN, BLACK);
    tft.setCursor(0, 56);
    tft.print(buffer);
  }

  // --- OLED: position/altitude text. 4 times a second is plenty and keeps the loop fast. ---
  if (millis() - lastOledMs >= 250) {
    lastOledMs = millis();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // --- Level Indicator ---
    display.setCursor(105, 20);
    display.println(isLevel ? "LVL" : "TLT");

    // --- GPS Coordinates ---
    display.setCursor(0, 16);
    if (gps.location.isValid()) {
      if (useDMS) {
        // --- DMS Format ---
        int lat_deg = abs(gps.location.rawLat().deg);
        double lat_min_full = gps.location.rawLat().billionths / 1000000000.0 * 60.0;
        int lat_min = (int)lat_min_full;
        double lat_sec = (lat_min_full - lat_min) * 60.0;
        char lat_dir = gps.location.rawLat().negative ? 'S' : 'N';

        display.print("Lat:");
        display.print(lat_deg);
        display.print((char)247);
        display.print(lat_min);
        display.print("'");
        display.print(lat_sec, 1);
        display.print("''");
        display.println(lat_dir);

        display.setCursor(0, 26);
        int lon_deg = abs(gps.location.rawLng().deg);
        double lon_min_full = gps.location.rawLng().billionths / 1000000000.0 * 60.0;
        int lon_min = (int)lon_min_full;
        double lon_sec = (lon_min_full - lon_min) * 60.0;
        char lon_dir = gps.location.rawLng().negative ? 'W' : 'E';

        display.print("Lon:");
        display.print(lon_deg);
        display.print((char)247);
        display.print(lon_min);
        display.print("'");
        display.print(lon_sec, 1);
        display.print("''");
        display.println(lon_dir);
      } else {
        // --- Decimal Degrees Format ---
        display.print("Lat: ");
        display.println(gps.location.lat(), 5);

        display.setCursor(0, 26);
        display.print("Lon: ");
        display.println(gps.location.lng(), 5);
      }

      // --- GPS Altitude ---
      display.setCursor(0, 36);
      display.print("GPS Alt:");
      if (useFeet) {
        display.print((int)gps.altitude.feet());
        display.println("ft");
      } else {
        display.print((int)gps.altitude.meters());
        display.println("m");
      }
    } else {
      display.println("GPS: No Fix");
    }

    // --- Barometric Altitude ---
    display.setCursor(0, 46);
    display.print("Baro Alt:");
    float pressure = BARO.readPressure(MILLIBAR);
    float baro_alt_ft = 44330.0 * (1.0 - pow(pressure / 1013.25, 1.0 / 5.255)) * 3.28084;

    if (useFeet) {
      display.print((int)baro_alt_ft);
      display.print("ft");
    } else {
      display.print((int)(baro_alt_ft / 3.28084));
      display.print("m");
    }

    display.display();
  }
}
