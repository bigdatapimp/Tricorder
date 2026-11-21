// ---------- EMF Meter code ----------
void EMFMeter(int pressType) {

  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = GEO_MENU;
    return;
  }


  float x, y, z;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    float emfMagnitude = sqrt(x * x + y * y + z * z);

    
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

  delay(500);

}

