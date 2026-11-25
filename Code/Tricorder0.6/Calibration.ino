// ---------- Calibration ----------

// Read one magnetometer sample (if a new one is ready) with the MAG_SIGN_* fix-ups applied.
// Returns false when no new sample is available.
bool readMagAxes(float &x, float &y, float &z) {
  if (!IMU.magneticFieldAvailable()) return false;
  IMU.readMagneticField(x, y, z);
  x *= MAG_SIGN_X;
  y *= MAG_SIGN_Y;
  z *= MAG_SIGN_Z;
  return true;
}

// Remove the hard-iron offset and scale out the soft-iron distortion found by calibrateMagnetometer()
void applyMagCal(float &x, float &y, float &z) {
  x = (x - magOffX) * magScaleX;
  y = (y - magOffY) * magScaleY;
  z = (z - magOffZ) * magScaleZ;
}

void calibrateMagnetometer() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 9);
  display.println(F("Calibrating..."));
  display.println(F("Slowly rotate the"));
  display.println(F("device in ALL"));
  display.println(F("directions (tumble"));
  display.println(F("it, figure-8)"));
  display.display();

  float minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9, minZ = 1e9, maxZ = -1e9;
  unsigned long startTime = millis();
  unsigned long lastCountdown = 0;

  while (millis() - startTime < MAG_CAL_MS) {
    while (Serial1.available()) gps.encode(Serial1.read());   // keep the GPS buffer from overflowing

    float x, y, z;
    if (readMagAxes(x, y, z)) {
      minX = min(minX, x); maxX = max(maxX, x);
      minY = min(minY, y); maxY = max(maxY, y);
      minZ = min(minZ, z); maxZ = max(maxZ, z);
    }

    // Countdown, once a second
    if (millis() - lastCountdown >= 1000) {
      lastCountdown = millis();
      display.fillRect(0, 50, 128, 14, SSD1306_BLACK);
      display.setCursor(0, 52);
      display.print(F("Time left: "));
      unsigned long elapsed = millis() - startTime;
      display.print(elapsed < MAG_CAL_MS ? (MAG_CAL_MS - elapsed) / 1000 + 1 : 0);
      display.print(F("s"));
      display.display();
    }
    delay(5);
  }

  // Half-range on each axis. For a perfect sphere these would all be equal to the field strength.
  float rX = (maxX - minX) / 2.0;
  float rY = (maxY - minY) / 2.0;
  float rZ = (maxZ - minZ) / 2.0;

  display.clearDisplay();
  display.setCursor(0, 15);

  // If any axis barely moved, the device wasn't rotated through enough orientations: keep the old values
  if (rX < 10.0 || rY < 10.0 || rZ < 10.0) {
    display.println(F("Cal FAILED"));
    display.println(F("Rotate through ALL"));
    display.println(F("orientations & retry"));
    display.display();
    Serial.println(F("Mag cal failed - not enough rotation, previous values kept"));
    delay(2500);
    return;
  }

  magOffX = (maxX + minX) / 2.0;
  magOffY = (maxY + minY) / 2.0;
  magOffZ = (maxZ + minZ) / 2.0;

  float avgR = (rX + rY + rZ) / 3.0;
  magScaleX = avgR / rX;
  magScaleY = avgR / rY;
  magScaleZ = avgR / rZ;

  // Print values so they can be pasted into the sketch as permanent defaults
  Serial.println(F("--- Magnetometer calibration (paste into Tricorder0_4332.ino) ---"));
  Serial.print(F("float magOffX = "));   Serial.print(magOffX, 2);   Serial.print(F(", magOffY = "));   Serial.print(magOffY, 2);
  Serial.print(F(", magOffZ = "));       Serial.print(magOffZ, 2);   Serial.println(F(";"));
  Serial.print(F("float magScaleX = ")); Serial.print(magScaleX, 3); Serial.print(F(", magScaleY = ")); Serial.print(magScaleY, 3);
  Serial.print(F(", magScaleZ = "));     Serial.print(magScaleZ, 3); Serial.println(F(";"));
  Serial.print(F("Field strength ~ ")); Serial.print(avgR, 1); Serial.println(F(" uT"));

  display.println(F("Calibration Done"));
  display.print(F("Field: "));
  display.print(avgR, 1);
  display.println(F(" uT"));
  display.display();
  delay(2000);
}

void calibrateMetalDetector() {
  float sum = 0.0;
  int count = 0;
  Serial.println("Calibrating baseline... Keep away from metal.");
  display.clearDisplay();
  display.setCursor(0, 16);
  display.println(F("Calibrating..."));
  display.display();

  while (count < CALIBRATION_SAMPLES) {
    if (IMU.magneticFieldAvailable()) {
      float x, y, z;
      IMU.readMagneticField(x, y, z);
      float magnitude = sqrt(x * x + y * y + z * z);
      sum += magnitude;
      count++;
      delay(100);
    }
  }

  baselineMagnitude = sum / CALIBRATION_SAMPLES;
  Serial.print("Baseline calibrated: ");
  Serial.print(baselineMagnitude);
  Serial.println(" uT");
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
  audioRingPush(sampleBuffer, bytesAvailable / 2);   // NEW: queue audio for the dB meter
  shotRingPush(sampleBuffer, bytesAvailable / 2);

}
