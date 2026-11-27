// ------------- Compass Declination ---------------
//
// The angle between magnetic north and true north where you are, in degrees: east is positive, west is negative. The Navigation
// screen adds it to the compass's magnetic heading. SYSTEM > DECLINATION lets you set it by hand.
//   UP / DOWN = +/- 0.1 degree     long UP / long DOWN = +/- 1 degree     ENTER = use this value     LEFT (long) = exit
// Leaving without pressing ENTER throws the change away.
//
// The value the sketch starts with is DECLINATION in the main sketch. If this core provides mbed's flash key-value store
// (kvstore_global_api.h, the same as the Time Zone screen uses) a value set here is saved and comes back after power-off; the
// screen says "SAVED". If not, it says "UNSAVED": the value is used until the next reboot, then DECLINATION is used again.
//
// Needs one line in setup():   declinationLoad();     State enum: DECLINATION_SETUP     loop(): case DECLINATION_SETUP: DeclinationSetup(pressType);

#if defined(__has_include)
  #if __has_include("kvstore_global_api.h")
    #include "kvstore_global_api.h"
    #define DECL_HAVE_KV 1
  #endif
#endif

#ifndef DECLINATION
  #define DECLINATION 0.0
#endif

#define DECL_LIMIT   45.0                   // the setting can go from -45.0 to +45.0 degrees
#define DECL_KV_KEY  "/kv/decl_x100"        // stored as a whole number of hundredths of a degree

float declinationDeg = DECLINATION;         // the declination in use (east positive)
bool declinationSavedOk = false;            // true if the value in use came from flash or was saved there
float declEdit = DECLINATION;               // the value being edited on the screen

// The declination in use: the Navigation screen adds this to the magnetic heading
float declinationNow() { return declinationDeg; }

// Text for a declination, e.g. "+3.4 E", "-12.5 W" or "0.0". 'withDegree' adds the degree sign of the OLED font after the number.
void declinationFormat(float v, char* out, size_t n, bool withDegree) {
  float a = fabsf(v);
  const char* deg = withDegree ? "\xF7" : "";
  if (a < 0.05f) {
    snprintf(out, n, "0.0%s", deg);
  } else {
    snprintf(out, n, "%c%.1f%s %c", v < 0 ? '-' : '+', a, deg, v < 0 ? 'W' : 'E');
  }
}

float declinationClamp(float v) {
  if (v > DECL_LIMIT) return DECL_LIMIT;
  if (v < -DECL_LIMIT) return -DECL_LIMIT;
  return v;
}

// Save the value in use. Returns true only if it really was stored.
bool declinationSave() {
#ifdef DECL_HAVE_KV
  int16_t v = (int16_t)lroundf(declinationDeg * 100.0f);
  int r = kv_set(DECL_KV_KEY, &v, sizeof(v), 0);
  if (r != 0) {
    Serial.print(F("Declination save failed, code "));
    Serial.println(r);
    declinationSavedOk = false;
    return false;
  }
  declinationSavedOk = true;
  return true;
#else
  declinationSavedOk = false;
  return false;
#endif
}

// Call once from setup(): restores the saved value if there is one, otherwise keeps DECLINATION
void declinationLoad() {
  declinationDeg = DECLINATION;
  declinationSavedOk = false;
#ifdef DECL_HAVE_KV
  int16_t v = 0;
  size_t actual = 0;
  int r = kv_get(DECL_KV_KEY, &v, sizeof(v), &actual);
  if (r == 0 && actual == sizeof(v) && v >= -(int)(DECL_LIMIT * 100.0) && v <= (int)(DECL_LIMIT * 100.0)) {
    declinationDeg = v / 100.0f;
    declinationSavedOk = true;
    Serial.print(F("Declination restored: "));
    Serial.println(declinationDeg, 2);
  } else {
    Serial.println(F("No saved declination, using DECLINATION"));
  }
#else
  Serial.println(F("Declination saving not available on this core, using DECLINATION"));
#endif
  declEdit = declinationDeg;
}

// What the phone is told (Bluetooth.ino, Motion packet): the value in hundredths of a degree, and whether it is saved
int declinationX100ForPhone() { return (int)lroundf(declinationDeg * 100.0f); }
bool declinationSavedForPhone() { return declinationSavedOk; }

// The phone set a value (Remote.ino), in hundredths of a degree: use it and try to save it, exactly as ENTER does on the
// Declination screen. False if it is outside the range.
bool declinationApplyFromPhone(int x100) {
  if (x100 < -(int)(DECL_LIMIT * 100.0) || x100 > (int)(DECL_LIMIT * 100.0)) return false;
  declinationDeg = x100 / 100.0f;
  declEdit = declinationDeg;
  bool saved = declinationSave();
  Serial.print(F("Declination set from the phone: "));
  Serial.print(declinationDeg, 2);
  Serial.println(saved ? F(" (saved)") : F(" (this session only)"));
  return true;
}

// What the colour screen shows while the Declination screen is open (the panel in Helpers.ino): the value being edited as the
// selected item, green when it is the value in use
const char* declinationPanelText() {
  static char buf[16];
  declinationFormat(declEdit, buf, sizeof(buf), false);
  return buf;
}
bool declinationPanelInUse() { return fabsf(declEdit - declinationDeg) < 0.005f; }

// The OLED (LCARS look, see LcarsOled.ino): the value in use, the value being edited in big digits, and the buttons. The tag
// says whether the last save worked.
void declinationDrawOled(bool hasMessage, bool savedFlag) {
  char buf[16];
  display.clearDisplay();
  lcoFrame("COMPASS", hasMessage ? (savedFlag ? "SAVED" : "UNSAVED") : NULL);

  declinationFormat(declinationDeg, buf, sizeof(buf), true);
  lcoRow(0, "IN USE", buf);

  lcoOutline(9, 28, 118, 18, "");
  declinationFormat(declEdit, buf, sizeof(buf), true);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(9 + (118 - 12 * (int)strlen(buf)) / 2, 31);
  display.print(buf);

  lcoTextAt(9, 48, "UP/DN 0.1  HOLD 1.0", 19);
  lcoTextAt(9, 56, "ENTER = USE THIS", 19);
  display.display();
}

void DeclinationSetup(int pressType) {
  static bool initialized = false;
  static bool dirty = true;
  static unsigned long messageUntil = 0;
  static bool messageSaved = false;
  static float drawnEdit = 1000.0f;                // what the OLED shows now (so a change from the phone is noticed too)
  static float drawnInUse = 1000.0f;

  // --- Exit ---
  if (pressType == LONG_PRESS && lastButton == 5) {
    initialized = false;
    messageUntil = 0;
    drawnEdit = 1000.0f;
    drawnInUse = 1000.0f;
    declEdit = declinationDeg;                     // an unsent change is dropped
    currentState = SYS_MENU;
    return;
  }

  // --- First pass after entering: start from the value in use ---
  if (!initialized) {
    initialized = true;
    declEdit = declinationDeg;
    dirty = true;
    messageUntil = 0;
  }

  // --- Buttons ---
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) {                         // Up: +0.1
      declEdit = declinationClamp(roundf((declEdit + 0.1f) * 10.0f) / 10.0f);
      dirty = true;
    } else if (lastButton == 4) {                  // Down: -0.1
      declEdit = declinationClamp(roundf((declEdit - 0.1f) * 10.0f) / 10.0f);
      dirty = true;
    } else if (lastButton == 1) {                  // Enter: use this value
      declinationDeg = declEdit;
      messageSaved = declinationSave();
      if (!messageSaved) {
        Serial.print(F("Declination set for this session only. To keep it, set DECLINATION to "));
        Serial.println(declinationDeg, 2);
      }
      messageUntil = millis() + 2000;
      dirty = true;
    }
  } else if (pressType == LONG_PRESS) {
    if (lastButton == 2) {                         // long Up: +1
      declEdit = declinationClamp(roundf((declEdit + 1.0f) * 10.0f) / 10.0f);
      dirty = true;
    } else if (lastButton == 4) {                  // long Down: -1
      declEdit = declinationClamp(roundf((declEdit - 1.0f) * 10.0f) / 10.0f);
      dirty = true;
    }
  }

  // Take the "Saved" message away after two seconds
  if (messageUntil != 0 && millis() > messageUntil) {
    messageUntil = 0;
    dirty = true;
  }

  // The phone can change the value while this screen is open (Remote.ino): redraw when either number moved
  if (declEdit != drawnEdit || declinationDeg != drawnInUse) dirty = true;

  // --- OLED: only when something changed ---
  if (dirty) {
    dirty = false;
    drawnEdit = declEdit;
    drawnInUse = declinationDeg;
    declinationDrawOled(messageUntil != 0, messageSaved);
  }

  displayGPSClock();   // the colour screen keeps the live clock and shows the value being edited
}
