// ---------- Button Press Handling ----------
int getPressedButton(int value) {
  if (value >= BUTTON_E_MIN && value <= BUTTON_E_MAX) return 1;
  if (value >= BUTTON_UP_MIN && value <= BUTTON_UP_MAX) return 2;
  if (value >= BUTTON_RT_MIN && value <= BUTTON_RT_MAX) return 3;
  if (value >= BUTTON_DN_MIN && value <= BUTTON_DN_MAX) return 4;
  if (value >= BUTTON_LT_MIN && value <= BUTTON_LT_MAX) return 5;
  return 0;
}

// Returns SHORT_PRESS (reported on release), LONG_PRESS (reported once, while still held) or NO_PRESS.
// lastButton always holds the button the returned press belongs to.
//
// Changes from the old version:
//  - The release is debounced too, so contact bounce can't produce a false short press.
//  - After a long press fires, nothing more is reported until the button is released. (Before, the
//    button re-armed while still held, so releasing right after a long press produced a phantom
//    SHORT_PRESS in the next screen, and holding for 2 s fired a second LONG_PRESS.)
int handleButtonPress(int rawButton) {
  static int candidate = 0;               // what the ADC currently says
  static unsigned long candidateSince = 0;
  static int held = 0;                    // debounced button (0 = none)
  static unsigned long heldSince = 0;
  static bool longReported = false;

  unsigned long now = millis();
  int result = NO_PRESS;

  // 1. A reading must stay the same for DEBOUNCE_DURATION before we believe it
  if (rawButton != candidate) {
    candidate = rawButton;
    candidateSince = now;
  }

  // 2. Accept a debounced change
  if (candidate != held && (now - candidateSince) >= DEBOUNCE_DURATION) {
    if (held != 0 && candidate == 0 && !longReported) {
      result = SHORT_PRESS;               // clean release of a short press (lastButton is still 'held')
    }
    held = candidate;
    heldSince = now;
    longReported = false;
    if (held != 0) lastButton = held;
  }

  // 3. Long press fires once, while the button is still down
  if (result == NO_PRESS && held != 0 && !longReported && (now - heldSince) >= LONG_PRESS_DURATION) {
    longReported = true;
    result = LONG_PRESS;
  }

  return result;
}
