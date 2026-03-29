// ---------- Button Press Handling ----------
int getPressedButton(int value) {
  if (value >= BUTTON_E_MIN && value <= BUTTON_E_MAX) return 1;
  if (value >= BUTTON_UP_MIN && value <= BUTTON_UP_MAX) return 2;
  if (value >= BUTTON_RT_MIN && value <= BUTTON_RT_MAX) return 3;
  if (value >= BUTTON_DN_MIN && value <= BUTTON_DN_MAX) return 4;
  if (value >= BUTTON_LT_MIN && value <= BUTTON_LT_MAX) return 5;
  return 0;
}

int handleButtonPress(int currentButton) {
  if (currentButton != 0) {
    if (stableButton == 0) {
      stableButton = currentButton;
      stableStart = millis();
    } else if (stableButton == currentButton && millis() - stableStart >= DEBOUNCE_DURATION) {
      // Button is stable, now handle press
      if (!buttonPressed) {
        buttonPressed = true;
        buttonPressTime = millis();
        lastButton = currentButton;
      } else {
        if (millis() - buttonPressTime >= LONG_PRESS_DURATION) {
          buttonPressed = false;
          stableButton = 0;
          return LONG_PRESS;
        }
      }
    } else if (stableButton != currentButton) {
      // Button changed, reset debounce
      stableButton = currentButton;
      stableStart = millis();
    }
  } else {
    if (buttonPressed) {
      buttonPressed = false;
      unsigned long duration = millis() - buttonPressTime;
      if (duration < LONG_PRESS_DURATION) {
        stableButton = 0;
        return SHORT_PRESS;
      }
    }
    stableButton = 0;
  }
  return NO_PRESS;
}
