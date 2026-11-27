// ------------- Time Zone Selection ---------------
//
// Lets you pick the local time zone used by the GPS clock (US zones, with automatic daylight saving).
//   UP / DOWN = move     ENTER = select this zone     LEFT (long) = exit
// The GPS clock on the colour TFT stays visible while you do this, so you see the change straight away.
//
// Remembering the choice across power-off:
//   If this core provides mbed's flash key-value store (kvstore_global_api.h), the choice is saved there
//   automatically and the screen says "Saved". If not (or if the save fails) the screen says "Not saved":
//   the zone still changes, but returns to TZ_DEFAULT_INDEX after a reboot. Change TZ_DEFAULT_INDEX
//   below to make a zone permanent that way.
//
// Needs one line in setup():   tzLoad();     and the GPS clock in Helpers.ino to call tzToLocal().

#if defined(__has_include)
  #if __has_include("kvstore_global_api.h")
    #include "kvstore_global_api.h"
    #define TZ_HAVE_KV 1
  #endif
#endif

#define TZ_COUNT          7
#define TZ_DEFAULT_INDEX  1                 // 0=Eastern 1=Central 2=Mountain 3=Arizona 4=Pacific 5=Alaska 6=Hawaii
#define TZ_KV_KEY         "/kv/tz_index"
#define TZ_LIST_ROWS      5                 // rows of the list visible on the OLED at once

// US daylight saving: starts 2nd Sunday in March 02:00, ends 1st Sunday in November 02:00 (local time).
// Offsets are minutes from UTC.
TimeChangeRule tzRuleEDT = {"EDT", Second, Sun, Mar, 2, -240};
TimeChangeRule tzRuleEST = {"EST", First,  Sun, Nov, 2, -300};
TimeChangeRule tzRuleCDT = {"CDT", Second, Sun, Mar, 2, -300};   // (same values as the old US_Central rules in the main file)
TimeChangeRule tzRuleCST = {"CST", First,  Sun, Nov, 2, -360};
TimeChangeRule tzRuleMDT = {"MDT", Second, Sun, Mar, 2, -360};
TimeChangeRule tzRuleMST = {"MST", First,  Sun, Nov, 2, -420};
TimeChangeRule tzRulePDT = {"PDT", Second, Sun, Mar, 2, -420};
TimeChangeRule tzRulePST = {"PST", First,  Sun, Nov, 2, -480};
TimeChangeRule tzRuleAKDT = {"AKDT", Second, Sun, Mar, 2, -480};
TimeChangeRule tzRuleAKST = {"AKST", First,  Sun, Nov, 2, -540};
TimeChangeRule tzRuleAZST = {"MST", First, Sun, Nov, 2, -420};   // Arizona: MST all year (no DST)
TimeChangeRule tzRuleHST  = {"HST", First, Sun, Nov, 2, -600};   // Hawaii: HST all year (no DST)

Timezone tzEastern(tzRuleEDT, tzRuleEST);
Timezone tzCentral(tzRuleCDT, tzRuleCST);
Timezone tzMountain(tzRuleMDT, tzRuleMST);
Timezone tzArizona(tzRuleAZST);
Timezone tzPacific(tzRulePDT, tzRulePST);
Timezone tzAlaska(tzRuleAKDT, tzRuleAKST);
Timezone tzHawaii(tzRuleHST);

// Same order as the names below and as TZ_DEFAULT_INDEX
Timezone* const tzTable[TZ_COUNT] = {&tzEastern, &tzCentral, &tzMountain, &tzArizona, &tzPacific, &tzAlaska, &tzHawaii};
const char* const tzNames[TZ_COUNT] = {"Eastern", "Central", "Mountain", "Arizona (no DST)", "Pacific", "Alaska", "Hawaii (no DST)"};

int tzIndex = TZ_DEFAULT_INDEX;   // zone currently in use
bool tzSavedOk = false;           // true if the zone in use came from flash or was saved there (false: it is lost at power-off)

// UTC -> local time for the selected zone. 'abbrev' (optional) receives e.g. "CDT" or "CST".
time_t tzToLocal(time_t utc, const char** abbrev) {
  TimeChangeRule* tcr = NULL;
  time_t local = tzTable[tzIndex]->toLocal(utc, &tcr);
  if (abbrev != NULL) *abbrev = (tcr != NULL) ? tcr->abbrev : "";
  return local;
}

// Save the selected zone. Returns true only if it really was stored.
bool tzSave() {
#ifdef TZ_HAVE_KV
  uint8_t v = (uint8_t)tzIndex;
  int r = kv_set(TZ_KV_KEY, &v, sizeof(v), 0);
  if (r != 0) {
    Serial.print(F("Time zone save failed, code "));
    Serial.println(r);
    tzSavedOk = false;
    return false;
  }
  tzSavedOk = true;
  return true;
#else
  tzSavedOk = false;
  return false;
#endif
}

// Call once from setup(): restores the saved zone if there is one, otherwise keeps TZ_DEFAULT_INDEX
void tzLoad() {
  tzIndex = TZ_DEFAULT_INDEX;
#ifdef TZ_HAVE_KV
  uint8_t v = 0;
  size_t actual = 0;
  int r = kv_get(TZ_KV_KEY, &v, sizeof(v), &actual);
  if (r == 0 && actual == sizeof(v) && v < TZ_COUNT) {
    tzIndex = v;
    tzSavedOk = true;
    Serial.print(F("Time zone restored: "));
    Serial.println(tzNames[tzIndex]);
  } else {
    Serial.println(F("No saved time zone, using default"));
  }
#else
  Serial.println(F("Time zone saving not available on this core, using default"));
#endif
}

// The zone in use and whether it will survive a power-off: sent to the phone in the Device packet (Bluetooth.ino)
uint8_t tzIndexForPhone() { return (uint8_t)tzIndex; }
bool tzSavedForPhone() { return tzSavedOk; }

// The phone chose a zone (Remote.ino): use it and try to save it, exactly as ENTER does on the Time Zone screen.
// False if there is no such zone.
bool tzApplyFromPhone(int index) {
  if (index < 0 || index >= TZ_COUNT) return false;
  tzIndex = index;
  bool saved = tzSave();
  Serial.print(F("Time zone set from the phone: "));
  Serial.print(tzNames[tzIndex]);
  Serial.println(saved ? F(" (saved)") : F(" (this session only)"));
  invalidateClockCache();                          // repaint the TFT clock with the new zone label
  return true;
}

// What the colour screen shows in the Time Zone screen (the menu panel in Helpers.ino): short names, which zone is
// highlighted by the cursor and which one is in use
int tzPanelCursor = 0;                             // set by TimeZoneSetup on every pass
int tzCursorForPanel() { return tzPanelCursor; }
int tzCountForPanel() { return TZ_COUNT; }
const char* tzPanelName(int i) {
  static const char* const names[TZ_COUNT] = {"EASTERN", "CENTRAL", "MOUNTAIN", "ARIZONA", "PACIFIC", "ALASKA", "HAWAII"};
  return (i >= 0 && i < TZ_COUNT) ? names[i] : "";
}
bool tzInUseForPanel(int i) { return i == tzIndex; }

// The OLED (LCARS look, see LcarsOled.ino): the zone list with the cursor as a solid block and * on the zone in use; the tag says
// whether the last save worked
void tzDrawOled(int cursor, int inUse, bool hasMessage, bool savedFlag) {
  display.clearDisplay();                                  // (LCARS look, see LcarsOled.ino)
  lcoFrame("TIME ZONE", hasMessage ? (savedFlag ? "SAVED" : "UNSAVED") : NULL);

  // Scrolling window of TZ_LIST_ROWS rows that always contains the cursor
  int first = cursor - (TZ_LIST_ROWS - 1) / 2;
  if (first < 0) first = 0;
  if (first > TZ_COUNT - TZ_LIST_ROWS) first = TZ_COUNT - TZ_LIST_ROWS;

  display.setTextSize(1);
  for (int r = 0; r < TZ_LIST_ROWS; r++) {
    int i = first + r;
    int y = lcoRowY(r);
    if (i == cursor) display.fillRoundRect(9, y, 118, 8, 3, SSD1306_WHITE);   // the cursor: a solid block
    else             display.drawRoundRect(9, y, 118, 8, 3, SSD1306_WHITE);
    display.setTextColor(i == cursor ? SSD1306_BLACK : SSD1306_WHITE);
    display.setCursor(9 + 6, y);
    display.print(tzNames[i]);
    if (i == inUse) {                                    // * marks the zone currently in use
      display.setCursor(9 + 108, y);
      display.print("*");
    }
  }
  display.display();
}

void TimeZoneSetup(int pressType) {
  static bool initialized = false;
  static bool dirty = true;
  static int cursor = 0;
  static unsigned long messageUntil = 0;
  static bool messageSaved = false;

  // --- Exit ---
  if (pressType == LONG_PRESS && lastButton == 5) {
    initialized = false;
    messageUntil = 0;
    currentState = SYS_MENU;
    return;
  }

  // --- First pass after entering: start with the cursor on the zone in use ---
  if (!initialized) {
    initialized = true;
    cursor = tzIndex;
    dirty = true;
    messageUntil = 0;
  }

  // --- Buttons ---
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) {                         // Up
      cursor = (cursor - 1 + TZ_COUNT) % TZ_COUNT;
      dirty = true;
    } else if (lastButton == 4) {                  // Down
      cursor = (cursor + 1) % TZ_COUNT;
      dirty = true;
    } else if (lastButton == 1) {                  // Enter: use this zone
      tzIndex = cursor;
      messageSaved = tzSave();
      if (!messageSaved) {
        Serial.print(F("Time zone set for this session only. To keep it, set TZ_DEFAULT_INDEX to "));
        Serial.println(tzIndex);
      }
      invalidateClockCache();                      // repaint the TFT clock with the new zone label
      messageUntil = millis() + 2000;
      dirty = true;
    }
  }

  // Take the "Saved" message away after two seconds
  if (messageUntil != 0 && millis() > messageUntil) {
    messageUntil = 0;
    dirty = true;
  }

  // --- OLED: only when something changed ---
  if (dirty) {
    dirty = false;
    tzDrawOled(cursor, tzIndex, messageUntil != 0, messageSaved);
  }

  tzPanelCursor = cursor;
  displayGPSClock();   // TFT keeps showing the live clock in the active zone
}
