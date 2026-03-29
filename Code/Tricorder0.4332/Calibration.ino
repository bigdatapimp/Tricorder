// ---------- Calibration ----------
void calibrateMagnetometer() {
  //Serial.println("Calibrating magnetometer... Rotate the device in all directions.");
  display.clearDisplay();
  display.setCursor(0, 15);
  display.println(F("Calibrating..."));
  display.println(F("Rotate device"));
  display.display();

  float minX = 1000, maxX = -1000, minY = 1000, maxY = -1000, minZ = 1000, maxZ = -1000;
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    if (IMU.magneticFieldAvailable()) {
      float x, y, z;
      IMU.readMagneticField(x, y, z);
      minX = min(minX, x); maxX = max(maxX, x);
      minY = min(minY, y); maxY = max(maxY, y);
      minZ = min(minZ, z); maxZ = max(maxZ, z);
    }
    delay(100);
  }

  float offsetX = (maxX + minX) / 2.0;
  float offsetY = (maxY + minY) / 2.0;
  float offsetZ = (maxZ + minZ) / 2.0;

  /*Serial.print("Offsets: X=");
  Serial.print(offsetX);
  Serial.print(" Y=");
  Serial.print(offsetY);
  Serial.print(" Z=");
  Serial.println(offsetZ);*/

  display.clearDisplay();
  display.setCursor(0, 15);
  display.println(F("Calibration Done"));
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
}
