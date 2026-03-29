// ----------- Navigation code ---------
void Navigation(int pressType) {
  // --- Handle Button Inputs ---
  if (pressType == LONG_PRESS) {
    if (lastButton == 5) { // Left: Exit to Menu
      tft.fillScreen(BLACK);
      currentState = GEO_MENU;
      return;
    }
    if (lastButton == 1) { // Enter: Calibrate Magnetometer
      calibrateMagnetometer();
      return;
    }
    if (lastButton == 3) { // Right: Toggle DMS / Decimal Degrees
      useDMS = !useDMS;
    }
    // --- New Button Logic ---
    if (lastButton == 2) { // UP: Toggle MPH/KPH
      useMPH = !useMPH;
    }
    if (lastButton == 4) { // DOWN: Toggle Feet/Meters
      useFeet = !useFeet;
    }
  }


  // --- Read GPS data ---
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // --- Read IMU data ---
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    // --- Calculate total acceleration magnitude ---
    float totalAccel = sqrt(ax * ax + ay * ay + az * az);
    
    // --- Calculate pitch and roll from accelerometer ---
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    float roll = atan2(ay, az);

    // --- Check if device is level ---
    float pitchDeg = pitch * 180.0 / PI;
    float rollDeg = roll * 180.0 / PI;
    
    isLevel = (totalAccel > 0.9 && totalAccel < 1.1) && 
              (fabs(pitchDeg) < 20.0) && 
              (fabs(rollDeg) < 20.0);

    // --- TILT-COMPENSATED MAGNETOMETER READING ---
    float cos_pitch = cos(pitch);
    float sin_pitch = sin(pitch);
    float cos_roll = cos(roll);
    float sin_roll = sin(roll);

    float mx_comp = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
    float my_comp = my * cos_roll - mz * sin_roll;

    float mag_heading = atan2(-my_comp, mx_comp) * 180.0 / PI;
    mag_heading += DECLINATION;
    
    while (mag_heading < 0) mag_heading += 360;
    while (mag_heading >= 360) mag_heading -= 360;

    // --- Sensor Fusion ---
    if (lastFusionTime == 0) {
      fusedHeading = mag_heading;
      lastFusionTime = millis();
    } else {
      unsigned long now = millis();
      float dt = (now - lastFusionTime) / 1000.0; 
      lastFusionTime = now;
      fusedHeading += gz * dt;
      while (fusedHeading < 0) fusedHeading += 360;
      while (fusedHeading >= 360) fusedHeading -= 360;

      float alpha = isLevel ? 0.98 : 1.0; 
      float diff = mag_heading - fusedHeading;
      if (diff > 180) diff -= 360;
      if (diff < -180) diff += 360;
      fusedHeading += (1 - alpha) * diff;
      while (fusedHeading < 0) fusedHeading += 360;
      while (fusedHeading >= 360) fusedHeading -= 360;
    }

    // --- Smoothing Filter ---
    static float displayHeading = 0;
    if (displayHeading == 0) {
      displayHeading = fusedHeading;
    }
    float smoothing = isLevel ? 0.80 : 0.95;
    displayHeading = displayHeading * smoothing + fusedHeading * (1.0 - smoothing);
    while (displayHeading < 0) displayHeading += 360;
    while (displayHeading >= 360) displayHeading -= 360;

    // --- Draw to OLEDs ---
    display.clearDisplay();
    

    // --- Draw Compass Circle ---
    tft.drawCircle(CENTER_X, CENTER_Y, COMPASS_RADIUS, MAGENTA);

    // --- Draw Cardinal Direction Labels ---
    tft.setTextSize(1);
    tft.setCursor(CENTER_X - 3, CENTER_Y - COMPASS_RADIUS - 8);
    tft.setTextColor(RED);
    tft.println("N");
    tft.setCursor(CENTER_X - 3, CENTER_Y + COMPASS_RADIUS + 3);
    tft.setTextColor(BLUE2);
    tft.println("S");
    tft.setCursor(CENTER_X - COMPASS_RADIUS - 8, CENTER_Y - 3);
    tft.println("W");
    tft.setCursor(CENTER_X + COMPASS_RADIUS + 2, CENTER_Y - 3);
    tft.println("E");

    // Clear just the needle area first
    tft.fillRect(CENTER_X - NEEDLE_LENGTH, CENTER_Y - NEEDLE_LENGTH, NEEDLE_LENGTH * 2, NEEDLE_LENGTH * 2, BLACK);

    // --- Draw Compass Needle ---
    float rad = displayHeading * PI / 180.0;
    int endX = CENTER_X + NEEDLE_LENGTH * sin(rad);
    int endY = CENTER_Y - NEEDLE_LENGTH * cos(rad);
    tft.drawLine(CENTER_X, CENTER_Y, endX, endY, ORANGE);

    // --- Draw Level Indicator ---
    display.setCursor(105, 20);
    display.setTextSize(1);
    if (isLevel) {
      display.println("LVL");
    } else {
      display.println("TLT");
    }

    // --- Display Heading Text (TFT) ---
    tft.fillRect(42, 45, 10, 10, BLACK);  
    tft.setTextSize(1);
    tft.setTextColor(BLUE, BLACK);
    tft.setCursor(0, 45);
    tft.print("Hdg:");
    char buffer[10];
    sprintf(buffer, "%.0f", displayHeading);
    tft.print(buffer);
    tft.print((char)247); // Degree symbol

    // --- Display GPS Speed (TFT - Below Heading) ---
    //tft.fillRect(23, 56, 80, 8, BLACK); // Clear speed line
    tft.setCursor(0, 56);
    tft.setTextColor(CYAN, BLACK);
    tft.print("Spd:");
    if (gps.speed.isValid()) {
        if (useMPH) {
            tft.print(gps.speed.mph(), 1);
            tft.print(" Mph");
        } else {
            tft.print(gps.speed.kmph(), 1);
            tft.print(" Kph");
        }
    } else {
        tft.print("---");
    }

    // --- Display GPS Coordinates (OLED) ---
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

      // --- GPS Altitude (OLED) ---
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
      display.println("");
      display.println("");
    }

    // --- Barometric Altitude (OLED) ---
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

  delay(50);
}