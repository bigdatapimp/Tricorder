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
//   [5, zone, 0, 0]        set the time zone (0 Eastern, 1 Central, 2 Mountain, 3 Arizona, 4 Pacific, 5 Alaska,
//                          6 Hawaii), exactly as ENTER does on the Time Zone screen; it is saved if flash allows
//   [7, lo, hi, 0]         set the compass declination: a signed 16-bit number (little endian) of hundredths of a degree,
//                          east positive, -4500 to 4500; used and saved exactly as ENTER does on the Declination screen
//   [8, 1, q, s] / [8, 0, 0, 0]   start / stop the live microphone stream, quality q = 0 (8 kHz), 1 (5.33 kHz), 2 (4 kHz) or 3 (16 kHz), packets
//                          of 4 + 8 x s bytes (s 2 to 30, so 20 to 244): the
//                          Tricorder opens its LIVE AUDIO screen and sends the
//                          microphone on the Audio characteristic (MicStream.ino); it stops when the phone says so, LEFT is
//                          pressed on the Tricorder, the phone disconnects or remote control is switched off
//   [6, 0x52, 0x42, 0x54]  reboot the Tricorder. The three extra bytes ("RBT") are a confirmation: a command that
//                          lacks them is ignored, so a stray write can never restart the unit
//   [4, action, 0, 0]      IR Receive screen only: action 1 start a clone, 2 open the IR data view, 3 back to the IR
//                          Receive menu (from the data view or a result). Everything else on that screen (skip, back,
//                          cancel, yes / no) is an ordinary BUTTON command.
//   [3, code, 0, 0]        send one IR code: 'code' is the number of the option on the IR Send screen (0 PWR, 1 VUP,
//                          2 VDN, 3 OK, 4 BACK, 5 EXIT, 6 Lft, 7 Rt, 8 Dn, 9 Up, 10 Amz, 11 YT, 12 Ejct, 13 Play,
//                          14 Stop, 15 Pause, 16 Menu). The Tricorder opens the IR Send screen first if it isn't open,
//                          then "presses ENTER" on that option, so exactly the same code runs as when you do it by
//                          hand. A code that has not been assigned (skipped when cloning a remote, or one of 12-16 that
//                          has no default) is ignored; the Device packet tells the phone which codes exist.
//
// SAFETY
//   - Remote control is OFF every time the Tricorder starts. You switch it on yourself: COMMS > Bluetooth, then
//     RIGHT. Commands are ignored while it is off, and it switches off when Bluetooth is turned off.
//   - The phone can never operate the Reboot, Time Zone, Declination and IR Send screens with button presses, and cannot open
//     Reboot, Time Zone, Declination or Bluetooth; on those screens only LEFT (back out) is accepted. Changing the time zone,
//     the declination and rebooting have their own commands (5, 7 and 6 above); the app asks before it sends a reboot. It CAN open and operate IR
//     Receive (cloning a remote transmits nothing), but it cannot leave that screen while a clone or a question
//     ("discard the new codes?") is on it: it has to answer or cancel first.
//     On the Bluetooth screen it may press UP, DOWN and LEFT but not ENTER or RIGHT. (So the phone cannot restart
//     the unit, or switch Bluetooth or remote control off from under itself.)
//   - The ONLY way the phone makes the Tricorder transmit is the IR command above, which can only choose one of
//     the codes already on the IR Send screen.
//   - A real button press always wins: it cancels whatever the phone was doing.
//
// DIAGNOSIS: every command received is printed on the Serial Monitor with the reason it was accepted or ignored.
// If loop() does not call remoteApply() (the line "pressType = remoteApply(pressType, button);" is missing),
// remoteHookMissing() turns true and the phone app says so.
//
// If you change the menus, update remoteLocate() below (menu position of each screen) to match.

#define REMOTE_CMD_BUTTON   1
#define REMOTE_CMD_GOTO     2
#define REMOTE_CMD_IR       3
#define REMOTE_CMD_IRRX     4
#define REMOTE_CMD_TZ       5
#define REMOTE_CMD_REBOOT   6
#define REMOTE_CMD_DECL     7
#define REMOTE_CMD_AUDIO    8
#define REMOTE_IR_CODES     17      // number of options on the IR Send screen (see IRRec.ino and IRSend.ino)
#define REMOTE_MAX_STEPS    8
#define REMOTE_STEP_MS      150     // pause between the steps of a script, so each screen has time to react
#define REMOTE_OFF_WHEN_PHONE_DISCONNECTS 0   // set to 1 to switch remote control off whenever the phone disconnects

bool remoteEnabled = false;
bool remoteIsOn() { return remoteEnabled; }     // for the tabs that sort before this one

// A script is a few steps run one at a time. Step kinds: 0 short press, 1 long press (arg = button 1-5),
// 2 put the main menu cursor on item 'arg', 3 put the submenu cursor on item 'arg', 4 send IR code 'arg' (select
// that option on the IR Send screen and press ENTER; only while that screen is open), 5 open the Live Audio screen (arg = quality + 4 x packet size).
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
//  - Reboot, Time Zone, Declination, Live Audio, IR Send: only LEFT (to back out). They restart the unit, change settings or send IR codes.
//    (IR Receive allows every button: it only listens, and cloning is how the phone's IR keys get their codes.)
//  - Bluetooth screen: UP, DOWN and LEFT. ENTER and RIGHT would switch Bluetooth / remote control off.
bool remoteButtonAllowed(int button) {
  if (currentState == REBOOT || currentState == TIMEZONE || currentState == DECLINATION_SETUP || currentState == AUDIO_STREAM || currentState == IRSEND) {
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
    case 19: *category = 0; *item = 3; return true;   // Seismometer
    case 4:  *category = 1; *item = 0; return true;   // Atmospheric
    case 5:  *category = 1; *item = 1; return true;   // dB Meter
    case 6:  *category = 1; *item = 2; return true;   // Spectrum
    case 13: *category = 3; *item = 0; return true;   // Shot Timer
    case 14: *category = 3; *item = 1; return true;   // Stopwatch
    case 15: *category = 3; *item = 2; return true;   // Countdown
    case 11: *category = 2; *item = 1; return true;   // IR Send (opening it sends nothing)
    case 12: *category = 2; *item = 2; return true;   // IR Receive (only listens)
    case 7:  *category = 4; *item = 0; return true;   // Battery
    case 8:  *category = 4; *item = 1; return true;   // System Info
    default: return false;                            // Time Zone, Declination, Reboot, Bluetooth: never opened remotely
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

// True if the phone must not walk away from the current screen: a clone or a question is open on IR Receive
bool remoteLeaveBlocked() {
  return currentState == IRRECEIVER && !irRxCanLeave();
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
    if (remoteLeaveBlocked()) { remoteLog("ignored: finish or cancel what is open on IR Receive first", d, n); return; }
    remoteBuildGoto(d[1]);
    remoteNextMs = millis();
    if (remoteStepCount == 0) remoteLog("nothing to do: already on that screen, or the phone may not open it", d, n);
    else remoteLog("accepted: opening a screen", d, n);
  } else if (d[0] == REMOTE_CMD_IR) {
    if (d[1] >= REMOTE_IR_CODES) { remoteLog("ignored: no such IR code", d, n); return; }
    if (!irSlotAvailable(d[1])) { remoteLog("ignored: that IR code has not been assigned", d, n); return; }
    if (remoteLeaveBlocked()) { remoteLog("ignored: finish or cancel what is open on IR Receive first", d, n); return; }
    remoteBuildGoto(11);                                   // open the IR Send screen (does nothing if it is already open)
    remoteAddStep(4, d[1]);                                // then send the chosen code
    remoteNextMs = millis();
    remoteLog("accepted: sending an IR code", d, n);
  } else if (d[0] == REMOTE_CMD_TZ) {
    if (!tzApplyFromPhone(d[1])) { remoteLog("ignored: no such time zone", d, n); return; }
    remoteLog("accepted: time zone changed", d, n);
  } else if (d[0] == REMOTE_CMD_DECL) {
    if (n < 3) { remoteLog("ignored: too short", d, n); return; }
    int x100 = (int)(int16_t)(uint16_t)(d[1] | ((uint16_t)d[2] << 8));
    if (!declinationApplyFromPhone(x100)) { remoteLog("ignored: declination out of range (-45 to +45 degrees)", d, n); return; }
    remoteLog("accepted: declination changed", d, n);
  } else if (d[0] == REMOTE_CMD_AUDIO) {
    if (d[1] == 0) {
      if (currentState != AUDIO_STREAM) { remoteLog("nothing to do: the live audio is not on", d, n); return; }
      micStreamStopFromPhone();
      remoteLog("accepted: live audio stopping", d, n);
    } else {
      if (remoteLeaveBlocked()) { remoteLog("ignored: finish or cancel what is open on IR Receive first", d, n); return; }
      if (currentState == AUDIO_STREAM) { remoteLog("nothing to do: the live audio is already on", d, n); return; }
      remoteBuildGoto(0);                                  // back out to the main menu (does nothing if it is there already)
      // then open the Live Audio screen at the quality asked for (d[2]) and the packet size (d[3], in 8-byte units, 0 = 20 bytes)
      uint8_t quality = (n >= 3 && d[2] <= 3) ? d[2] : 0;
      uint8_t size8 = (n >= 4 && d[3] >= 2 && d[3] <= 30) ? d[3] : 2;
      remoteAddStep(5, (uint8_t)(quality | (size8 << 2)));
      remoteNextMs = millis();
      remoteLog("accepted: live audio starting", d, n);
    }
  } else if (d[0] == REMOTE_CMD_REBOOT) {
    if (n < 4 || d[1] != 0x52 || d[2] != 0x42 || d[3] != 0x54) { remoteLog("ignored: a reboot needs the confirmation bytes 52 42 54", d, n); return; }
    remoteLog("accepted: rebooting now", d, n);
    rebootNow(true);
  } else if (d[0] == REMOTE_CMD_IRRX) {
    if (currentState != IRRECEIVER) { remoteLog("ignored: the IR Receive screen is not open", d, n); return; }
    if (!irRxRemoteAction(d[1])) { remoteLog("ignored: that IR Receive action is not possible on this screen", d, n); return; }
    remoteClear();
    remoteLog("accepted: IR Receive action", d, n);
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
  if (kind == 4) {                                          // send an IR code: only while the IR Send screen is open
    if (currentState != IRSEND) {
      remoteClear();
      return pressType;
    }
    irSendCursor = arg;                                     // select the option ...
    lastButton = 1;                                         // ... and press ENTER, in one step so that quick
    return SHORT_PRESS;                                     //     successive taps can't cut each other short
  }
  if (kind == 5) {                                          // open the Live Audio screen (arg = quality + 4 x packet size in 8-byte units)
    micStreamStartFromPhone(arg & 3, arg >> 2);
    return pressType;
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
