// ------------- Metal Detector code ----------
#define METAL_UPDATE_MS 200    // the screen updates 5 times a second
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

      
  // Update at most every METAL_UPDATE_MS, but never wait: return at once in between, so the buttons and
  // Bluetooth keep running (the old delay(500) froze everything for half a second at a time).
  static unsigned long lastUpdateMs = 0;
  if (millis() - lastUpdateMs < METAL_UPDATE_MS) return;
  lastUpdateMs = millis();

  float x, y, z;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    float currentMagnitude = sqrt(x * x + y * y + z * z);
    float deviation = abs(currentMagnitude - baselineMagnitude);
    bool metalDetected = (deviation > DETECTION_THRESHOLD);

    telMetalMag = currentMagnitude;                // published for Bluetooth (see Bluetooth.ino)
    telMetalDev = deviation;
    telMetalDetected = metalDetected;
    telMetalMs = millis();

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
}