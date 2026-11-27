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
  char left[20];                                             // the OLED in the LCARS look (see LcarsOled.ino)
  snprintf(left, sizeof(left), "TIME LEFT: %lus", (unsigned long)(MAG_CAL_MS / 1000UL));
  lcoCalibrating("MAG", "TUMBLE IT SLOWLY IN", "ALL DIRECTIONS", "(FIGURE-8)", 0, left);

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
      unsigned long elapsed = millis() - startTime;
      snprintf(left, sizeof(left), "TIME LEFT: %lus", elapsed < MAG_CAL_MS ? (unsigned long)((MAG_CAL_MS - elapsed) / 1000 + 1) : 0UL);
      lcoCalibrating("MAG", "TUMBLE IT SLOWLY IN", "ALL DIRECTIONS", "(FIGURE-8)", (int)(elapsed * 100UL / MAG_CAL_MS), left);
    }
    delay(5);
  }

  // Half-range on each axis. For a perfect sphere these would all be equal to the field strength.
  float rX = (maxX - minX) / 2.0;
  float rY = (maxY - minY) / 2.0;
  float rZ = (maxZ - minZ) / 2.0;

  // If any axis barely moved, the device wasn't rotated through enough orientations: keep the old values
  if (rX < 10.0 || rY < 10.0 || rZ < 10.0) {
    lcoMessage("CAL FAILED", "MAG", "ROTATE THROUGH ALL", "ORIENTATIONS AND", "RETRY", NULL, NULL);
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

  char field[24];
  snprintf(field, sizeof(field), "FIELD: %.1f uT", avgR);
  lcoMessage("CALIBRATED", "MAG", "CALIBRATION DONE", field, NULL, NULL, NULL);
  delay(2000);
}

void calibrateMetalDetector() {
  float sum = 0.0;
  int count = 0;
  Serial.println("Calibrating baseline... Keep away from metal.");
  lcoCalibrating("METAL", "KEEP AWAY FROM", "METAL", NULL, 0, "SETTING BASELINE...");   // the OLED in the LCARS look (see LcarsOled.ino)

  while (count < CALIBRATION_SAMPLES) {
    if (IMU.magneticFieldAvailable()) {
      float x, y, z;
      IMU.readMagneticField(x, y, z);
      float magnitude = sqrt(x * x + y * y + z * z);
      sum += magnitude;
      count++;
      if (count % 10 == 0) lcoCalibrating("METAL", "KEEP AWAY FROM", "METAL", NULL, count * 100 / CALIBRATION_SAMPLES, "SETTING BASELINE...");
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
  micStreamPush(sampleBuffer, bytesAvailable / 2);   // live audio to the phone (only queues while streaming)

}
