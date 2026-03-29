
// --------- Battery Monitor code ---------
void Battery(int pressType) {
  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = SYS_MENU;
    return;
  }

  // Update every 2 seconds, similar to atmospheric
  if (millis() - lastBatteryUpdate > 2000) {
    // Read voltage from VIn pin (assumed A0 with divider)
    int adcValue = analogRead(VOLTAGE_PIN);
    float voltage = (adcValue / 1023.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;

    // Estimate battery percentage for 1S LiPo (3.0V = 0%, 4.2V = 100%)
    float percentage = ((voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V)) * 100.0;
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;

    
    display.clearDisplay();
    display.setCursor(0, 9);
    display.println(F("Battery Status"));
    display.setCursor(0, 20);
    display.print(F("Voltage: "));
    display.print(voltage, 2);
    display.println(F(" V"));
    display.setCursor(0, 35);
    display.print(F("Battery: "));
    display.print(percentage, 1);
    display.println(F(" %"));

    // Optional: Add a battery bar
    int barWidth = map(percentage, 0, 100, 0, SCREEN_WIDTH - 20);
    display.fillRect(0, 50, barWidth, 10, SSD1306_WHITE);

    display.display();
    lastBatteryUpdate = millis();
  }
  
  

}