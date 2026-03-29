// ------------- Metal Detector code ----------
void MetalDetector(int pressType) {

  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = GEO_MENU;
    return;
  }
  if (pressType == LONG_PRESS && lastButton == 1) {
    calibrateMetalDetector();
    return;
  }

      
  float x, y, z;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    float currentMagnitude = sqrt(x * x + y * y + z * z);
    float deviation = abs(currentMagnitude - baselineMagnitude);
    bool metalDetected = (deviation > DETECTION_THRESHOLD);

    display.clearDisplay();
    display.setCursor(0, 9);
    display.println(F("Metal Detector"));
    display.setCursor(0, 20);
    display.print(F("Mag: "));
    display.print(currentMagnitude, 1);
    display.println(F(" uT"));
    display.setCursor(0, 30);
    display.print(F("Dev: "));
    display.print(deviation, 1);
    display.println(F(" uT"));
    int barWidth = map(deviation, 0, 200, 0, SCREEN_WIDTH - 20);
    display.fillRect(0, 40, barWidth, 10, SSD1306_WHITE);
    display.setCursor(0, 55);
    if (metalDetected) {
      display.println(F("METAL DETECTED!"));
    } else {
      display.println(F("No metal"));
    }
    display.display();
  }

  delay(500);
}