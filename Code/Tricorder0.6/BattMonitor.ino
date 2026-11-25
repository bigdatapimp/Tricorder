
// --------- Battery Monitor code ---------
//
// The colour screen (LCARS look, see Lcars.ino) shows the charge in big digits, a 22-segment bar whose segments are red up to
// 20 %, amber up to 40 % and green above, the voltage and a one-word verdict. The OLED keeps its own text and bar.
#define BATT_SEGS 22

// How many bar segments are lit for a charge in percent (rounded to the nearest)
int battLitSegments(float percentage) {
  if (!(percentage > 0.0)) return 0;                       // also catches NaN
  int n = (int)(percentage * BATT_SEGS / 100.0 + 0.5);
  if (n > BATT_SEGS) n = BATT_SEGS;
  return n;
}

// A segment has the colour of the charge at its bottom edge
uint16_t battSegColor(int i) {
  return lcPercentColor(i * 100 / BATT_SEGS);
}

void battDrawTft(float voltage, float percentage, bool first) {
  char buf[12];
  if (first) {
    lcFrame("BATTERY", false);
    tft.setTextSize(2);
    tft.setTextColor(lcPeach(), BLACK);
    tft.setCursor(69, 10);
    tft.print("%");
    lcPill(8, 40, 26, lcLilac(), "VOLT");
  }
  int pct = (int)(percentage + 0.5);
  uint16_t color = lcPercentColor(pct);
  snprintf(buf, sizeof(buf), "%5.1f", percentage);
  tft.setTextSize(2);
  tft.setTextColor(color, BLACK);
  tft.setCursor(9, 10);
  tft.print(buf);
  int lit = battLitSegments(percentage);
  for (int i = 0; i < BATT_SEGS; i++) lcSeg(8 + i * 4, 27, 3, 9, battSegColor(i), i < lit);
  snprintf(buf, sizeof(buf), "%4.2fV", voltage);
  tft.setTextSize(1);
  tft.setTextColor(lcPeach(), BLACK);
  tft.setCursor(38, 40);
  tft.print(buf);
  lcBigPill(8, 50, 88, 8, color, pct > 40 ? "GOOD" : (pct > 20 ? "LOW" : "CRITICAL"), 1);
}

void Battery(int pressType) {
  static bool tftReady = false;       // is the frame on the colour screen?

  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    tftReady = false;
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
    battDrawTft(voltage, percentage, !tftReady);
    tftReady = true;
    lastBatteryUpdate = millis();
  }
  
  

}