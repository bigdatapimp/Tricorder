// ------------- Reboot ---------------
//
// Asks for confirmation (so a stray press can't restart the unit), then does a full software reset.
//   ENTER (short) = reboot now        LEFT (short or long) = cancel and go back to the System menu
//
// A reboot runs setup() again, so you'll see the startup sequence and the metal detector
// baseline calibration (keep the unit away from metal while it runs).

// Restart now: say so on both screens, let the Serial output finish, then do a full software reset (does not return).
// Used by the confirmation below and by the phone's reboot command (Remote.ino).
// The colour screen (LCARS look, see Lcars.ino): the question and the two keys
void rebootDrawConfirm() {
  lcFrame("REBOOT", false);
  lcBigPill(8, 12, 88, 11, lcGold(), "RESTART NOW?", 1);
  lcBigPill(8, 28, 88, 14, lcAlert(), "ENTER = YES", 1);
  lcBigPill(8, 46, 88, 14, lcBlue(), "LEFT = CANCEL", 1);
}

void rebootDrawBanner() {
  lcFrame("REBOOT", false);
  lcBigPill(8, 25, 88, 14, lcGold(), "REBOOTING...", 1);
}

void rebootNow(bool fromPhone) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 9);
  display.println(F("REBOOTING..."));
  display.display();
  rebootDrawBanner();
  Serial.println(fromPhone ? F("Rebooting by request from the phone") : F("Rebooting by request"));
  Serial.flush();
  delay(600);              // let the screens update and the serial output finish
  NVIC_SystemReset();      // full software reset (does not return)
}

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
    rebootNow(false);
  }

  // --- Draw the confirmation screen once ---
  if (!drawn) {
    drawn = true;
    rebootDrawConfirm();
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
