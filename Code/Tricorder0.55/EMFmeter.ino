// ---------- EMF Meter code ----------
#define EMF_UPDATE_MS 200      // the screen updates 5 times a second
void EMFMeter(int pressType) {

  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = GEO_MENU;
    return;
  }


  // Update at most every EMF_UPDATE_MS, but never wait: return at once in between, so the buttons and
  // Bluetooth keep running (the old delay(500) froze everything for half a second at a time).
  static unsigned long lastUpdateMs = 0;
  if (millis() - lastUpdateMs < EMF_UPDATE_MS) return;
  lastUpdateMs = millis();

  float x, y, z;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    float emfMagnitude = sqrt(x * x + y * y + z * z);

    telEmfMag = emfMagnitude;                      // published for Bluetooth (see Bluetooth.ino)
    telEmfMs = millis();

    
    display.clearDisplay();
    display.setCursor(0, 9);
    display.println(F("EMF Meter"));
    display.setCursor(0, 20);
    display.print(F("EMF: "));
    display.print(emfMagnitude, 1);
    display.println(F(" uT"));
    int barWidth = map(emfMagnitude, 0, MAX_EMF, 0, SCREEN_WIDTH - 20);
    if (barWidth > 0) {
      display.fillRect(0, 30, barWidth, 10, SSD1306_WHITE);
    }
    display.setCursor(0, 45);
    display.print(F("0"));
    display.setCursor(SCREEN_WIDTH - 50, 45);
    display.print(MAX_EMF, 0);
    display.print(F("uT"));
    display.display();
  }
}

