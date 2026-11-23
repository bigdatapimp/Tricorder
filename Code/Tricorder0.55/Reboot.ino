// ------------- Reboot ---------------
//
// Asks for confirmation (so a stray press can't restart the unit), then does a full software reset.
//   ENTER (short) = reboot now        LEFT (short or long) = cancel and go back to the System menu
//
// A reboot runs setup() again, so you'll see the startup sequence and the metal detector
// baseline calibration (keep the unit away from metal while it runs).

void RebootDevice(int pressType) {
  static bool drawn = false;

  // --- Cancel ---
  if ((pressType == SHORT_PRESS || pressType == LONG_PRESS) && lastButton == 5) {
    drawn = false;
    currentState = SYS_MENU;
    return;
  }

  // --- Confirm ---
  if (pressType == SHORT_PRESS && lastButton == 1) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 9);
    display.println(F("REBOOTING..."));
    display.display();
    tft.fillScreen(BLACK);
    Serial.println(F("Rebooting by request"));
    Serial.flush();
    delay(600);              // let the screens update and the serial output finish
    NVIC_SystemReset();      // full software reset (does not return)
  }

  // --- Draw the confirmation screen once ---
  if (!drawn) {
    drawn = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 9);
    display.println(F("REBOOT"));
    display.setCursor(0, 24);
    display.println(F("Restart the device?"));
    display.setCursor(0, 38);
    display.println(F("ENTER = Yes"));
    display.setCursor(0, 48);
    display.println(F("LEFT  = Cancel"));
    display.display();
  }
}
