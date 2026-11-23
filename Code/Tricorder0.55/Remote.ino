// ------------- Remote control from the phone ---------------
//
// The phone app can press the Tricorder's buttons and open its screens. This works by feeding "virtual button
// presses" into the very same code that handles the real buttons (see remoteApply(), called from loop()), so
// every screen behaves exactly as if you had pressed the button yourself.
//
// Commands arrive over Bluetooth (Bluetooth.ino passes them to remoteHandleCommand()):
//   [1, button, kind, 0]   press a button: button 1 ENTER, 2 UP, 3 RIGHT, 4 DOWN, 5 LEFT;  kind 0 short, 1 long
//   [2, screen id, 0, 0]   open a screen (ids as in the "active screen" field; 0 = the main menu). The Tricorder backs
//                          out to the main menu and walks to the screen with a few presses, like you would.
//
// SAFETY
//   - Remote control is OFF every time the Tricorder starts. You switch it on yourself: COMMS > Bluetooth, then
//     RIGHT. Commands are ignored while it is off, and it switches off when Bluetooth is turned off.
//   - The phone can never operate the Reboot, Time Zone, IR Send and IR Receive screens or open them; on those
//     screens only LEFT (back out) is accepted. On the Bluetooth screen it may press UP, DOWN and LEFT but not
//     ENTER or RIGHT. (So the phone cannot restart the unit, send IR codes, or switch Bluetooth or remote
//     control off from under itself.)
//   - A real button press always wins: it cancels whatever the phone was doing.
//
// DIAGNOSIS: every command received is printed on the Serial Monitor with the reason it was accepted or ignored.
// If loop() does not call remoteApply() (the line "pressType = remoteApply(pressType, button);" is missing),
// remoteHookMissing() turns true and the phone app says so.
//
// If you change the menus, update remoteLocate() below (menu position of each screen) to match.

#define REMOTE_CMD_BUTTON   1
#define REMOTE_CMD_GOTO     2
#define REMOTE_MAX_STEPS    8
#define REMOTE_STEP_MS      150     // pause between the steps of a script, so each screen has time to react
#define REMOTE_OFF_WHEN_PHONE_DISCONNECTS 0   // set to 1 to switch remote control off whenever the phone disconnects

bool remoteEnabled = false;

// A script is a few steps run one at a time. Step kinds: 0 short press, 1 long press (arg = button 1-5),
// 2 put the main menu cursor on item 'arg', 3 put the submenu cursor on item 'arg'.
uint8_t remoteStepKind[REMOTE_MAX_STEPS];
uint8_t remoteStepArg[REMOTE_MAX_STEPS];
int remoteStepCount = 0;
int remoteStepIndex = 0;
unsigned long remoteNextMs = 0;
unsigned long remoteLastApplyMs = 0;     // when loop() last called remoteApply(): used to notice a missing hook

// True if remote control is on but loop() isn't calling remoteApply(), i.e. this line is missing from loop():
//     pressType = remoteApply(pressType, button);
bool remoteHookMissing() {
  return remoteEnabled && (millis() - remoteLastApplyMs) > 1000UL;
}

// Print a received command and what was done with it (Serial Monitor)
void remoteLog(const char* msg, const uint8_t* d, int n) {
  Serial.print("Remote: command");
  for (int i = 0; i < n; i++) {
    Serial.print(' ');
    if (d[i] < 16) Serial.print('0');
    Serial.print(d[i], HEX);
  }
  Serial.print(" - ");
  Serial.println(msg);
}

bool remoteBusy() {
  return remoteStepIndex < remoteStepCount;
}

void remoteClear() {
  remoteStepCount = 0;
  remoteStepIndex = 0;
}

void remoteAddStep(uint8_t kind, uint8_t arg) {
  if (remoteStepCount < REMOTE_MAX_STEPS) {
    remoteStepKind[remoteStepCount] = kind;
    remoteStepArg[remoteStepCount] = arg;
    remoteStepCount++;
  }
}

void remoteSetEnabled(bool on) {
  remoteEnabled = on;
  if (!on) remoteClear();
}

// May the phone press this button (1 ENTER, 2 UP, 3 RIGHT, 4 DOWN, 5 LEFT) on the screen that is open now?
//  - Reboot, Time Zone, IR Send, IR Receive: only LEFT (to back out). They restart the unit, change settings or
//    send IR codes.
//  - Bluetooth screen: UP, DOWN and LEFT. ENTER and RIGHT would switch Bluetooth / remote control off.
bool remoteButtonAllowed(int button) {
  if (currentState == REBOOT || currentState == TIMEZONE || currentState == IRSEND || currentState == IRRECEIVER) {
    return button == 5;
  }
  if (currentState == BLUETOOTH_SCREEN) {
    return button == 2 || button == 4 || button == 5;
  }
  return true;
}

// 0 = the main menu, 1 = a category menu, 2 = a tool screen
int remoteLevel() {
  switch (currentState) {
    case MAIN_MENU:
      return 0;
    case GEO_MENU: case ENV_MENU: case COMMS_MENU: case UTIL_MENU: case SYS_MENU:
      return 1;
    default:
      return 2;
  }
}

// Where each screen the phone may open sits in the menus: main menu item (GEO 0, ENV 1, COMMS 2, UTIL 3, SYS 4)
// and the item inside that category's menu. Screen ids are the ones in bleScreenId() (Bluetooth.ino).
bool remoteLocate(int screenId, int* category, int* item) {
  switch (screenId) {
    case 2:  *category = 0; *item = 0; return true;   // Metal Detector
    case 3:  *category = 0; *item = 1; return true;   // EMF Meter
    case 1:  *category = 0; *item = 2; return true;   // Navigation
    case 4:  *category = 1; *item = 0; return true;   // Atmospheric
    case 5:  *category = 1; *item = 1; return true;   // dB Meter
    case 6:  *category = 1; *item = 2; return true;   // Spectrum
    case 13: *category = 3; *item = 0; return true;   // Shot Timer
    case 14: *category = 3; *item = 1; return true;   // Stopwatch
    case 15: *category = 3; *item = 2; return true;   // Countdown
    case 7:  *category = 4; *item = 0; return true;   // Battery
    case 8:  *category = 4; *item = 1; return true;   // System Info
    default: return false;                            // Time Zone, Reboot, IR, Bluetooth: never opened remotely
  }
}

// Back out to the main menu (one long LEFT per level), then walk down to the screen
void remoteBuildGoto(int screenId) {
  remoteClear();
  int category = 0;
  int item = 0;
  if (screenId != 0) {
    if (!remoteLocate(screenId, &category, &item)) return;
    if (bleScreenId() == (uint8_t)screenId) return;        // already there
  } else if (currentState == MAIN_MENU) {
    return;
  }
  int levels = remoteLevel();
  for (int i = 0; i < levels; i++) remoteAddStep(1, 5);    // long LEFT
  if (screenId != 0) {
    remoteAddStep(2, (uint8_t)category);                   // main menu cursor on the category
    remoteAddStep(0, 1);                                   // ENTER
    remoteAddStep(3, (uint8_t)item);                       // submenu cursor on the item
    remoteAddStep(0, 1);                                   // ENTER
  }
}

// Called by Bluetooth.ino when the phone disconnects
void remotePhoneDisconnected() {
#if REMOTE_OFF_WHEN_PHONE_DISCONNECTS
  remoteSetEnabled(false);
#endif
}

// A command received over Bluetooth
void remoteHandleCommand(const uint8_t* d, int n) {
  if (!remoteEnabled) {
    remoteLog("IGNORED: remote control is switched off (COMMS > Bluetooth > RIGHT)", d, n);
    return;
  }
  if (n < 2) {
    remoteLog("ignored: too short", d, n);
    return;
  }
  if (d[0] == REMOTE_CMD_BUTTON) {
    if (n < 3) { remoteLog("ignored: too short", d, n); return; }
    int button = d[1];
    if (button < 1 || button > 5) { remoteLog("ignored: no such button", d, n); return; }
    if (d[2] > 1) { remoteLog("ignored: press kind must be 0 (short) or 1 (long)", d, n); return; }
    if (!remoteButtonAllowed(button)) { remoteLog("ignored: the phone may not press that button on this screen", d, n); return; }
    remoteClear();
    remoteAddStep(d[2], (uint8_t)button);                  // kind 0 = short, 1 = long
    remoteNextMs = millis();
    remoteLog("accepted: button press", d, n);
  } else if (d[0] == REMOTE_CMD_GOTO) {
    remoteBuildGoto(d[1]);
    remoteNextMs = millis();
    if (remoteStepCount == 0) remoteLog("nothing to do: already on that screen, or the phone may not open it", d, n);
    else remoteLog("accepted: opening a screen", d, n);
  } else {
    remoteLog("ignored: unknown command", d, n);
  }
}

// Called from loop() with the real button state. Returns the press type to use: the real one, or the next
// step of a remote script.
int remoteApply(int pressType, int rawButton) {
  remoteLastApplyMs = millis();                              // loop() is calling us: the hook is in place
  if (!remoteBusy()) {
    remoteClear();
    return pressType;
  }
  if (!remoteEnabled || pressType != NO_PRESS) {            // switched off, or a real button press: stop the script
    remoteClear();
    return pressType;
  }
  if (rawButton != 0) return pressType;                     // a real button is being held: wait
  unsigned long now = millis();
  if ((long)(now - remoteNextMs) < 0) return pressType;     // not time for the next step yet

  uint8_t kind = remoteStepKind[remoteStepIndex];
  uint8_t arg = remoteStepArg[remoteStepIndex];
  remoteStepIndex++;
  remoteNextMs = now + REMOTE_STEP_MS;

  if (kind == 0 || kind == 1) {
    if (!remoteButtonAllowed(arg)) {                        // the screen changed to one that button is blocked on
      remoteClear();
      return pressType;
    }
    lastButton = arg;
    return (kind == 0) ? SHORT_PRESS : LONG_PRESS;
  }
  if (kind == 2) {
    mainMenuCursor = arg;
    menuDirty = true;
  } else if (kind == 3) {
    subMenuCursor = arg;
    menuDirty = true;
  }
  return pressType;
}
