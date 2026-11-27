// ----------- Navigation code ---------
//
// 
// The heading shown is the compass's magnetic heading plus the declination (east is positive, west negative; the pure magnetic
// declination for where you are). It is set at SYSTEM > DECLINATION (Declination.ino), which starts from DECLINATION in the main
// sketch when nothing has been saved. COMPASS_TRIM is for anything left over after that, such as
// the board not sitting exactly square in its case: positive turns the heading clockwise. It stays 0.0 unless a check
// against a known direction (for example the GPS course on a straight road) shows the compass is still off.
#ifndef COMPASS_TRIM
#define COMPASS_TRIM 0.0
#endif

// ---- the colour screen (LCARS look, see Lcars.ino) ----
//
// The compass is on the right (a ring, N in red, the needle); down the left side: the heading in big digits with its
// compass-point name, a rail, the GPS speed with its unit, and LVL / TLT (the heading is only reliable when the device is
// level). The header's tag block shows the coordinate format used on the OLED: DEC, DMS or MGRS.
#define NAV_CX      74       // compass centre and size on the colour screen
#define NAV_CY      34
#define NAV_R       13
#define NAV_NEEDLE  9
#define NAV_HDG_X   9        // the heading digits (size 2, 3 digits) ...
#define NAV_HDG_Y   11
#define NAV_NAME_Y  28       // ... the name of the compass point under them,
#define NAV_RAIL_Y  38       // a rail,
#define NAV_SPD_Y   41       // the speed (label block and value),
#define NAV_UNIT_Y  50       // and its unit with the LVL / TLT block

// The name of the compass point for a heading in degrees (16 points, the same names as the phone app)
const char* navCardinal(int heading) {
  const char* names[16] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int i = (int)((heading / 22.5) + 0.5) % 16;
  if (i < 0) i += 16;
  return names[i];
}

// The header tag shows the coordinate format that long RIGHT cycles through
void navDrawTag() {
  lcSetTag(coordFormat == 0 ? "DEC" : (coordFormat == 1 ? "DMS" : "MGRS"));
}

// The parts of the colour screen that do not change: the frame, the compass ring with N / S / W / E, the rail, the labels
void navDrawStatic() {
  lcFrame("NAVIGATE", true);
  navDrawTag();
  tft.drawCircle(NAV_CX, NAV_CY, NAV_R, lcLilac());
  tft.setTextSize(1);
  tft.setTextColor(lcAlert(), BLACK);
  tft.setCursor(NAV_CX - 3, NAV_CY - NAV_R - 8);
  tft.print("N");
  tft.setTextColor(lcBlue(), BLACK);
  tft.setCursor(NAV_CX - 3, NAV_CY + NAV_R + 3);
  tft.print("S");
  tft.setCursor(NAV_CX - NAV_R - 8, NAV_CY - 3);
  tft.print("W");
  tft.setCursor(NAV_CX + NAV_R + 2, NAV_CY - 3);
  tft.print("E");
  tft.setTextColor(lcPeach(), BLACK);                         // the degree sign after the heading digits
  tft.setCursor(NAV_HDG_X + 36 + 1, NAV_HDG_Y);
  tft.print((char)247);
  tft.fillRoundRect(8, NAV_RAIL_Y, 44, 2, 1, lcOrange());
  lcPill(8, NAV_SPD_Y, 20, lcLilac(), "SPD");
}

// The GPS speed in the unit chosen with long UP (a block of 5 characters, and the unit)
void navDrawSpeed(bool valid, double speed, const char* unit) {
  char buf[8];
  if (valid) snprintf(buf, sizeof(buf), "%5.1f", speed);
  else       snprintf(buf, sizeof(buf), "%5s", "---");
  tft.setTextSize(1);
  tft.setTextColor(lcPeach(), BLACK);
  tft.setCursor(31, NAV_SPD_Y);
  tft.print(buf);
  tft.setTextColor(lcLilac(), BLACK);
  tft.setCursor(8, NAV_UNIT_Y);
  tft.print(unit);
}

// LVL (green) or TLT (amber) next to the unit
void navDrawLevel(bool level) {
  if (level) lcPill(31, NAV_UNIT_Y, 20, lcOk(), "LVL");
  else       lcPill(31, NAV_UNIT_Y, 20, lcGold(), "TLT");
}

// While the magnetometer is calibrated (15 seconds, the OLED explains how) the colour screen says so too
void navDrawCalibrating() {
  tft.fillRoundRect(8, 28, 88, 22, 4, lcGold());
  tft.setTextSize(1);
  tft.setTextColor(BLACK, lcGold());
  tft.setCursor(8 + (88 - 66) / 2, 31);
  tft.print("CALIBRATING");
  tft.setCursor(8 + (88 - 54) / 2, 40);
  tft.print("TUMBLE IT");
}

// One row of the OLED: a label block 30 pixels wide at row 0 to 4 (9 pixels apart from y = 18) and the cursor placed for its value
// in a column at x = 43 (LCARS look, see LcarsOled.ino)
void navOledLabel(int row, const char* label) {
  lcoPill(9, 18 + 9 * row, 30, 8, label);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(43, 18 + 9 * row);
}

void Navigation(int pressType) {
  static bool initialized = false;            // false = (re)draw the static parts of the screen
  static float ax = 0.0, ay = 0.0, az = 1.0;  // low-pass filtered accelerometer (g)
  static float displayHeading = 0.0;
  static bool haveHeading = false;
  static unsigned long lastHeadingMs = 0;
  static int drawnHeading = -1;
  static int drawnPoint = -1;                 // the compass-point name currently on the colour screen
  static int drawnLevel = -1;                 // 1 = LVL shown, 0 = TLT shown
  static char shownHdg[6];                    // the heading digits currently on the colour screen
  static unsigned long lastOledMs = 0;
  static unsigned long lastSpeedMs = 0;
  // MGRS is recomputed only when the GPS position changes (about once a second), not on every refresh
  static double mgrsLastLat = 1000.0, mgrsLastLng = 1000.0;
  static bool mgrsValid = false;
  static int mgrsZone = 0;
  static char mgrsBand = ' ', mgrsCol = ' ', mgrsRow = ' ';
  static long mgrsE = 0, mgrsN = 0;

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
      navDrawCalibrating();
      calibrateMagnetometer();
      initialized = false;   // calibration took over the OLED, redraw everything
      haveHeading = false;
      return;
    }
    if (lastButton == 3) { // Right: cycle coordinates  Decimal Degrees -> DMS -> MGRS
      coordFormat = (coordFormat + 1) % 3;
      lastOledMs = 0;        // refresh the OLED right away
      if (initialized) navDrawTag();
    }
    if (lastButton == 2) { // UP: cycle speed units  MPH -> KPH -> knots
      speedUnit = (speedUnit + 1) % 3;
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

    drawnPoint = -1;
    drawnLevel = -1;
    tftTextForget(shownHdg, sizeof(shownHdg));
    navDrawStatic();
    tftTextDiff(shownHdg, NAV_HDG_X, NAV_HDG_Y, 2, "---", lcPeach(), BLACK);     // until there is a heading
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
      heading += declinationNow() + COMPASS_TRIM;     // true heading: magnetic heading + declination (east positive)
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

      // publish for Bluetooth (see Bluetooth.ino)
      telHeading = displayHeading;
      telPitch = pitchDeg;
      telRoll = rollDeg;
      telField = sqrt(mx * mx + my * my + mz * mz);
      telLevel = isLevel;
      telMotionMs = millis();

    }
  }

  // --- Draw needle + heading text, only when the whole-degree heading changed ---
  if (haveHeading) {
    int h = ((int)(displayHeading + 0.5)) % 360;
    if (h != drawnHeading) {
      drawnHeading = h;

      // Clear just the needle area, then draw the needle
      tft.fillRect(NAV_CX - NAV_NEEDLE, NAV_CY - NAV_NEEDLE,
                   NAV_NEEDLE * 2 + 1, NAV_NEEDLE * 2 + 1, BLACK);
      float rad = displayHeading * PI / 180.0;
      int endX = NAV_CX + NAV_NEEDLE * sin(rad);
      int endY = NAV_CY - NAV_NEEDLE * cos(rad);
      tft.drawLine(NAV_CX, NAV_CY, endX, endY, lcOrange());

      // Heading digits: fixed width so shorter numbers fully overwrite longer ones
      char buffer[8];
      snprintf(buffer, sizeof(buffer), "%3d", h);
      tftTextDiff(shownHdg, NAV_HDG_X, NAV_HDG_Y, 2, buffer, lcPeach(), BLACK);

      // The name of the compass point, in a block (redrawn only when it changes)
      int point = ((int)((h / 22.5) + 0.5)) % 16;
      if (point != drawnPoint) {
        drawnPoint = point;
        tft.fillRect(8, NAV_NAME_Y, 24, 8, BLACK);
        lcPill(8, NAV_NAME_Y, 20, lcLilac(), navCardinal(h));
      }
    }
  }

  // --- GPS speed (TFT): GPS only updates once a second, so redraw once a second ---
  if (millis() - lastSpeedMs >= 1000) {
    lastSpeedMs = millis();
    double spd = 0.0;
    const char* unit = (speedUnit == 1) ? "KPH" : ((speedUnit == 2) ? "KTS" : "MPH");
    bool spdValid = gps.speed.isValid();
    if (spdValid) {
      if (speedUnit == 1)      spd = gps.speed.kmph();
      else if (speedUnit == 2) spd = gps.speed.knots();
      else                     spd = gps.speed.mph();
    }
    navDrawSpeed(spdValid, spd, unit);
  }

  // --- LVL / TLT on the colour screen, whenever the state changes ---
  if ((isLevel ? 1 : 0) != drawnLevel) {
    drawnLevel = isLevel ? 1 : 0;
    navDrawLevel(isLevel);
  }

  // --- OLED: position/altitude text. 4 times a second is plenty and keeps the loop fast. ---
  if (millis() - lastOledMs >= 250) {
    lastOledMs = millis();

    display.clearDisplay();                                        // (LCARS look, see LcarsOled.ino)
    lcoFrame("NAVIGATE", coordFormat == 0 ? "DEC" : (coordFormat == 1 ? "DMS" : "MGRS"));

    // --- GPS Coordinates ---
    if (gps.location.isValid()) {
      if (coordFormat == 2) {
        // --- MGRS Format (WGS84, 1 m) ---
        double lat = gps.location.lat();
        double lng = gps.location.lng();
        if (lat != mgrsLastLat || lng != mgrsLastLng) {
          mgrsLastLat = lat;
          mgrsLastLng = lng;
          mgrsValid = mgrsFromLatLon(lat, lng, &mgrsZone, &mgrsBand, &mgrsCol, &mgrsRow, &mgrsE, &mgrsN);
        }
        navOledLabel(0, "GRID");
        if (mgrsValid) {
          display.print(mgrsZone);
          display.print(mgrsBand);
          display.print(" ");
          display.print(mgrsCol);
          display.print(mgrsRow);

          navOledLabel(1, "E/N");
          char mgrsLine[16];
          snprintf(mgrsLine, sizeof(mgrsLine), "%05ld %05ld", mgrsE, mgrsN);
          display.print(mgrsLine);
        } else {
          display.print("n/a (polar)");
        }
      } else if (coordFormat == 1) {
        // --- DMS Format ---
        char dms[20];
        int lat_deg = abs(gps.location.rawLat().deg);
        double lat_min_full = gps.location.rawLat().billionths / 1000000000.0 * 60.0;
        int lat_min = (int)lat_min_full;
        double lat_sec = (lat_min_full - lat_min) * 60.0;
        char lat_dir = gps.location.rawLat().negative ? 'S' : 'N';
        snprintf(dms, sizeof(dms), "%d%c%d'%.1f\"%c", lat_deg, (char)247, lat_min, lat_sec, lat_dir);
        navOledLabel(0, "LAT");
        display.print(dms);

        int lon_deg = abs(gps.location.rawLng().deg);
        double lon_min_full = gps.location.rawLng().billionths / 1000000000.0 * 60.0;
        int lon_min = (int)lon_min_full;
        double lon_sec = (lon_min_full - lon_min) * 60.0;
        char lon_dir = gps.location.rawLng().negative ? 'W' : 'E';
        snprintf(dms, sizeof(dms), "%d%c%d'%.1f\"%c", lon_deg, (char)247, lon_min, lon_sec, lon_dir);
        navOledLabel(1, "LON");
        display.print(dms);
      } else {
        // --- Decimal Degrees Format ---
        navOledLabel(0, "LAT");
        display.print(gps.location.lat(), 5);
        navOledLabel(1, "LON");
        display.print(gps.location.lng(), 5);
      }

      // --- GPS Altitude ---
      navOledLabel(2, "GPS");
      if (useFeet) {
        display.print((int)gps.altitude.feet());
        display.print(" ft");
      } else {
        display.print((int)gps.altitude.meters());
        display.print(" m");
      }
    } else {
      lcoOutline(9, 18, 118, 8, "NO GPS FIX");
    }

    // --- Barometric Altitude ---
    navOledLabel(3, "BARO");
    float pressure = BARO.readPressure(MILLIBAR);
    float baro_alt_ft = 44330.0 * (1.0 - pow(pressure / 1013.25, 1.0 / 5.255)) * 3.28084;

    if (useFeet) {
      display.print((int)baro_alt_ft);
      display.print(" ft");
    } else {
      display.print((int)(baro_alt_ft / 3.28084));
      display.print(" m");
    }

    // --- Level indicator: a solid LVL block, or a hollow TLT one when the device is tilted ---
    if (isLevel) lcoPill(9, 54, 30, 8, "LVL");
    else         lcoOutline(9, 54, 30, 8, "TLT");

    display.display();
  }
}
