// --------- IR Sender code ----------
//
// The IR Send screen sends the codes in the table kept by IRRec.ino (17 buttons: the built-in NEC codes, or a cloned
// remote). ENTER sends the selected button; a button with no code is dimmed and sends nothing.
//
// HOW A CODE GOES OUT
//   The IR LED is wired straight from pin D3 through a 220 ohm resistor. This file makes the 38 kHz carrier itself:
//   it writes directly to the pin's GPIO set / clear registers and times every edge against the CPU's cycle counter.
//   Three things make that work well on this board:
//   - HIGH DRIVE. The nRF52840 normally drives a pin at "standard" strength (about 2 mA); "high drive" (about 10 mA)
//     lets more current through the LED, so it is brighter and reaches further. With 220 ohm the current cannot exceed
//     (3.3 V - LED voltage) / 220 ohm = 9-10 mA, inside the 15 mA per pin limit Arduino lists for this board.
//     The pin is rebuilt by pinMode() at the start of every send, which resets the drive, so it is raised afterwards,
//     and nothing touches the pin again while the frame goes out.
//   - NO PWM DRIVER. The IR library makes its carrier with the chip's PWM hardware and leaves a PWM unit connected to
//     the pin afterwards, which would hide this file's pulses; irPwmRelease() lets go of it first. (Claiming a PWM
//     output of our own froze the Tricorder, so this file never does.)
//   - PRIORITY BOOST. With Bluetooth running, the RTOS threads of the Bluetooth stack can interrupt the main thread for
//     several ms, which ruins the timing. For the ~120 ms a send takes, the thread that runs loop() is raised above them
//     (nothing may be printed to the Serial Monitor while it is boosted: the USB code needs the lower threads).
//   If this build cannot do it (the pin registers or the cycle counter are not reachable) the same codes are sent
//   with the IR library's own mark() and space() instead, still on high drive when possible.

// The nRF52840's pin registers, by name
#if defined(__has_include)
#if __has_include(<nrf.h>)
#include <nrf.h>
#endif
#endif

#if defined(NRF_P0) && defined(NRF_P1)
#define IR_DRIVE_AVAILABLE 1
#else
#define IR_DRIVE_AVAILABLE 0
#endif
#define IR_DRIVE_MASK  (7UL << 8)        // PIN_CNF bits 10:8 hold the drive strength
#define IR_DRIVE_H0H1  (3UL << 8)        // 3 = high drive for both '0' and '1'

// Where an Arduino pin (D2..D12) sits on the nRF52840 of the Nano 33 BLE: port 0 or 1, and the bit within it
bool irPinLocate(int arduinoPin, int* port, int* bit) {
  static const int8_t irPinTable[11][2] = {    // D2 ... D12, as {port, bit}
    {1, 11}, {1, 12}, {1, 15}, {1, 13}, {1, 14}, {0, 23}, {0, 21}, {0, 27}, {1, 2}, {1, 1}, {1, 8}
  };
  if (arduinoPin < 2 || arduinoPin > 12) return false;
  *port = irPinTable[arduinoPin - 2][0];
  *bit = irPinTable[arduinoPin - 2][1];
  return true;
}

// Switch the IR LED's pin to high drive. Only the drive bits are changed; the rest of the pin's setup is kept.
bool irSetHighDrive() {
#if IR_DRIVE_AVAILABLE
  int port, bit;
  if (!irPinLocate(IR_SEND_PIN, &port, &bit)) return false;
  NRF_GPIO_Type* gpio = (port == 0) ? NRF_P0 : NRF_P1;
  gpio->PIN_CNF[bit] = (gpio->PIN_CNF[bit] & ~IR_DRIVE_MASK) | IR_DRIVE_H0H1;
  return true;
#else
  return false;
#endif
}

// NEC timing, in microseconds (one NEC "unit" is 560 us)
#define IR_NEC_UNIT                560UL
#define IR_NEC_HEADER_MARK         (16UL * IR_NEC_UNIT)
#define IR_NEC_HEADER_SPACE        (8UL * IR_NEC_UNIT)
#define IR_NEC_BIT_MARK            IR_NEC_UNIT
#define IR_NEC_ONE_SPACE           (3UL * IR_NEC_UNIT)
#define IR_NEC_ZERO_SPACE          IR_NEC_UNIT
#define IR_NEC_REPEAT_HEADER_SPACE (4UL * IR_NEC_UNIT)
#define IR_NEC_REPEAT_PERIOD_MS    110UL      // a repeat frame starts 110 ms after the start of the frame

// The 32 bits of a NEC frame, sent least significant bit first: address, then its inverse (or the high byte of a
// 16 bit address), then the command, then its inverse
uint32_t irNecRaw(uint16_t address, uint8_t command) {
  uint32_t raw = (address < 0x100) ? (((uint32_t)(uint8_t)~address << 8) | address) : (uint32_t)address;
  return raw | ((uint32_t)command << 16) | ((uint32_t)(uint8_t)~command << 24);
}

// ---- priority boost ----
#if defined(__has_include) && !defined(IR_RTOS_AVAILABLE)
#if __has_include(<cmsis_os2.h>)
#include <cmsis_os2.h>
#define IR_RTOS_AVAILABLE 1
#endif
#endif
#ifndef IR_RTOS_AVAILABLE
#define IR_RTOS_AVAILABLE 0                 // the RTOS header isn't visible to this build: no boost
#endif

bool irBoosted = false;                     // is this thread boosted right now?
#if IR_RTOS_AVAILABLE
osThreadId_t irThreadId = 0;
osPriority_t irSavedPriority = osPriorityNormal;
#endif

// Raise (or put back) the priority of the thread running loop()
void irBoostSet(bool on) {
#if IR_RTOS_AVAILABLE
  if (on && !irBoosted) {
    irThreadId = osThreadGetId();
    irSavedPriority = osThreadGetPriority(irThreadId);
    osThreadSetPriority(irThreadId, osPriorityRealtime);
    irBoosted = true;
  } else if (!on && irBoosted) {
    osThreadSetPriority(irThreadId, irSavedPriority);
    irBoosted = false;
  }
#else
  (void)on;
#endif
}
void irBoostBegin() { irBoostSet(true); }
void irBoostEnd()   { irBoostSet(false); }

// ---- the IR library's PWM unit ----
#if defined(NRF_PWM0) && defined(NRF_PWM1) && defined(NRF_PWM2) && defined(NRF_PWM3)
#define IR_PWM_AVAILABLE 1
#else
#define IR_PWM_AVAILABLE 0
#endif

// If a running PWM unit drives the IR pin (that is the IR library's carrier, and it stays connected after the library
// has sent something), stop it and let go of the pin so this file's own pulses can reach it. Returns how many units
// were let go. The library's own sending may stay silent afterwards until the Tricorder is rebooted, which does not
// matter: this file does not use it unless the REG sender is unavailable.
int irPwmRelease() {
#if IR_PWM_AVAILABLE
  NRF_PWM_Type* units[4] = { NRF_PWM0, NRF_PWM1, NRF_PWM2, NRF_PWM3 };
  int port, bit;
  if (!irPinLocate(IR_SEND_PIN, &port, &bit)) return 0;
  uint32_t want = ((uint32_t)port << 5) | (uint32_t)bit;
  int released = 0;
  for (int u = 0; u < 4; u++) {
    if (!(units[u]->ENABLE & 1UL)) continue;                 // not running: leave it alone
    for (int c = 0; c < 4; c++) {
      uint32_t v = units[u]->PSEL.OUT[c];
      if ((v & 0x80000000UL) || (v & 0x3FUL) != want) continue;   // not this pin
      units[u]->EVENTS_STOPPED = 0;
      units[u]->TASKS_STOP = 1;
      unsigned long t0 = micros();
      while (!units[u]->EVENTS_STOPPED && (micros() - t0) < 2000UL) { }   // wait for it to stop (at most 2 ms)
      units[u]->EVENTS_STOPPED = 0;
      units[u]->ENABLE = 0;
      units[u]->PSEL.OUT[c] = 0xFFFFFFFFUL;                  // disconnect the pin
      released++;
    }
  }
  return released;
#else
  return 0;
#endif
}

// ---- the carrier made by this file ----
#ifndef IR_DWT_CYCCNT
#define IR_DWT_CTRL   (*(volatile uint32_t*)0xE0001000UL)      // ARM Cortex-M debug watchpoint unit: control
#define IR_DWT_CYCCNT (*(volatile uint32_t*)0xE0001004UL)      // ... and its CPU cycle counter
#define IR_DEMCR      (*(volatile uint32_t*)0xE000EDFCUL)      // ... which has to be switched on here (bit 24)
#endif
#ifndef IR_PIN_HIGH
#define IR_PIN_HIGH() (*irOutSetReg = irPinMask)
#define IR_PIN_LOW()  (*irOutClrReg = irPinMask)
#endif

volatile uint32_t* irOutSetReg = 0;
volatile uint32_t* irOutClrReg = 0;
uint32_t irPinMask = 0;
int irRegState = 0;                     // 0 = not tried yet, 1 = ready, -1 = this build can't do it
unsigned long irCyclesPerUs = 0;        // the CPU clock in MHz, measured the first time (64 on this board)
uint32_t irPeriodCycles = 0;            // one carrier period in CPU cycles
uint32_t irOnCycles = 0;                // ... of which the pin is high for a third
uint32_t irCyc = 0;                     // the timeline: when the pulse in progress should end (cycle counter value)

// Where the pin's GPIO set / clear registers are
bool irRegLocate() {
#if IR_DRIVE_AVAILABLE
  int port, bit;
  if (!irPinLocate(IR_SEND_PIN, &port, &bit)) return false;
  irPinMask = 1UL << bit;
  NRF_GPIO_Type* gpio = (port == 0) ? NRF_P0 : NRF_P1;
  irOutSetReg = &gpio->OUTSET;
  irOutClrReg = &gpio->OUTCLR;
  return true;
#else
  return false;
#endif
}

// Start the CPU cycle counter and measure its speed against micros() (the carrier is set up by irRegSetCarrier())
bool irCycleCounterReady() {
  IR_DEMCR |= (1UL << 24);              // switch on the trace block that holds the cycle counter
  IR_DWT_CTRL |= 1UL;                   // start counting
  uint32_t c0 = IR_DWT_CYCCNT;
  unsigned long t0 = micros();
  while ((micros() - t0) < 2000UL) { }  // 2 ms
  uint32_t c1 = IR_DWT_CYCCNT;
  unsigned long dt = micros() - t0;
  if (dt == 0 || c1 == c0) return false;                // the counter isn't counting
  unsigned long cpu = (unsigned long)(((c1 - c0) + dt / 2) / dt);
  if (cpu < 8 || cpu > 256) return false;               // not a believable CPU clock
  irCyclesPerUs = cpu;
  return true;
}

// Wait until the cycle counter reaches 'target'
void irWaitCyc(uint32_t target) {
  while ((int32_t)(IR_DWT_CYCCNT - target) < 0) { }
}

// Set the carrier frequency (in kHz) for the frames that follow: the length of one carrier period in CPU cycles, and the
// third of it that the pin is high for
void irRegSetCarrier(unsigned khz) {
  if (khz < 30 || khz > 60) khz = 38;
  irPeriodCycles = (uint32_t)((irCyclesPerUs * 1000000UL) / ((unsigned long)khz * 1000UL));
  irOnCycles = irPeriodCycles / 3;
}

// Make the pin ready: set up the registers and the cycle counter once, then every send make sure it is an output on
// high drive. Returns false if this build can't do it (the caller then uses the library's pulses instead).
bool irRegReady() {
  if (irRegState < 0) return false;
  if (irRegState == 0) {
    if (!irRegLocate() || !irCycleCounterReady()) { irRegState = -1; return false; }
    irRegState = 1;
  }
  irRegSetCarrier(38);                  // every send starts at 38 kHz; recorded codes with another carrier change it afterwards
  irPwmRelease();                       // the library's PWM unit must not be driving the pin
  pinMode(IR_SEND_PIN, OUTPUT);         // an output again, whatever the library did to the pin ...
  irSetHighDrive();                     // ... then high drive, after pinMode() (which resets it)
  IR_PIN_LOW();
  return true;
}

// A mark: the carrier on for 'us' microseconds, ending exactly at the scheduled time (a whole number of carrier
// periods, the last one cut short if needed)
void irRegMark(unsigned long us) {
  uint32_t end = irCyc + (uint32_t)(us * irCyclesPerUs);
  uint32_t t = irCyc;
  for (;;) {
    int32_t rem = (int32_t)(end - t);
    if (rem <= 0) break;
    uint32_t on = (rem < (int32_t)irOnCycles) ? (uint32_t)rem : irOnCycles;
    uint32_t per = (rem < (int32_t)irPeriodCycles) ? (uint32_t)rem : irPeriodCycles;
    irWaitCyc(t);                       // the start of this carrier period
    IR_PIN_HIGH();
    irWaitCyc(t + on);
    IR_PIN_LOW();
    irWaitCyc(t + per);
    t += irPeriodCycles;
  }
  irCyc = end;
}

// A space: the pin low for 'us' microseconds
void irRegSpace(unsigned long us) {
  irCyc += (uint32_t)(us * irCyclesPerUs);
  irWaitCyc(irCyc);
}

// One NEC frame (and, if asked, the repeat frame that follows it 110 ms later) with the carrier made here. Returns
// false if this build can't do it.
bool irSendNecRegWord(uint32_t raw, bool withRepeat) {
  if (!irRegReady()) return false;
  irBoostBegin();
  uint32_t frameStart = IR_DWT_CYCCNT;
  irCyc = frameStart;
  irRegMark(IR_NEC_HEADER_MARK);
  irRegSpace(IR_NEC_HEADER_SPACE);
  for (int i = 0; i < 32; i++) {
    irRegMark(IR_NEC_BIT_MARK);
    irRegSpace((raw & 1UL) ? IR_NEC_ONE_SPACE : IR_NEC_ZERO_SPACE);
    raw >>= 1;
  }
  irRegMark(IR_NEC_BIT_MARK);           // stop bit
  irBoostEnd();
  if (withRepeat) {
    irCyc = frameStart + (uint32_t)(IR_NEC_REPEAT_PERIOD_MS * 1000UL * irCyclesPerUs);   // 110 ms after the frame started
    while ((int32_t)(irCyc - IR_DWT_CYCCNT) > (int32_t)(2000UL * irCyclesPerUs)) delay(1);  // let the other tasks (Bluetooth) run
    irBoostBegin();
    irWaitCyc(irCyc);
    irRegMark(IR_NEC_HEADER_MARK);
    irRegSpace(IR_NEC_REPEAT_HEADER_SPACE);
    irRegMark(IR_NEC_BIT_MARK);
    irBoostEnd();
  }
  return true;
}

bool irSendNecReg(uint16_t address, uint8_t command, bool withRepeat) {
  return irSendNecRegWord(irNecRaw(address, command), withRepeat);
}

// Recorded codes (clones): the stored mark / space times replayed as they were. The first frame starts at once; when
// 'frames' is more than 1 each further frame starts gapMs after the start of the one before (Sony devices want three
// frames 45 ms apart). t[0] is a mark, t[1] a space, and so on. Returns false if this build can't do it.
bool irSendRawReg(const uint16_t* t, int n, uint8_t khz, uint8_t frames, uint8_t gapMs) {
  if (!irRegReady()) return false;
  irRegSetCarrier(khz);
  uint32_t start = IR_DWT_CYCCNT;
  for (int f = 0; f < frames; f++) {
    irCyc = start + (uint32_t)((unsigned long)f * gapMs * 1000UL * irCyclesPerUs);
    if (f > 0) {
      while ((int32_t)(irCyc - IR_DWT_CYCCNT) > (int32_t)(2000UL * irCyclesPerUs)) delay(1);   // let the other tasks run
    }
    irBoostBegin();
    if (f > 0) irWaitCyc(irCyc);
    for (int i = 0; i < n; i++) {
      if ((i & 1) == 0) irRegMark(t[i]); else irRegSpace(t[i]);
    }
    irBoostEnd();
  }
  irRegSetCarrier(38);
  return true;
}

// ---- the same with the IR library's own pulses (used only if the carrier above can't be made) ----
// The library prepares the pin in enableIROut(), which puts the drive back to standard, so the drive is raised after it.

void irSendNecWord(uint32_t raw, bool withRepeat) {
  IrSender.enableIROut(38);
  irSetHighDrive();
  unsigned long frameStart = millis();
  IrSender.mark(IR_NEC_HEADER_MARK);
  IrSender.space(IR_NEC_HEADER_SPACE);
  for (int i = 0; i < 32; i++) {
    IrSender.mark(IR_NEC_BIT_MARK);
    IrSender.space((raw & 1UL) ? IR_NEC_ONE_SPACE : IR_NEC_ZERO_SPACE);
    raw >>= 1;
  }
  IrSender.mark(IR_NEC_BIT_MARK);            // stop bit
  if (withRepeat) {
    unsigned long elapsed = millis() - frameStart;
    if (elapsed < IR_NEC_REPEAT_PERIOD_MS) delay(IR_NEC_REPEAT_PERIOD_MS - elapsed);
    IrSender.mark(IR_NEC_HEADER_MARK);       // the NEC repeat frame
    IrSender.space(IR_NEC_REPEAT_HEADER_SPACE);
    IrSender.mark(IR_NEC_BIT_MARK);
  }
}

void irSendNec(uint16_t address, uint8_t command, bool withRepeat) {
  irSendNecWord(irNecRaw(address, command), withRepeat);
}

void irSendRawLib(const uint16_t* t, int n, uint8_t khz, uint8_t frames, uint8_t gapMs) {
  IrSender.enableIROut(khz);
  irSetHighDrive();
  unsigned long frameStart = millis();
  for (int f = 0; f < frames; f++) {
    if (f > 0) {
      unsigned long due = (unsigned long)f * gapMs;
      unsigned long elapsed = millis() - frameStart;
      if (elapsed < due) delay(due - elapsed);
    }
    for (int i = 0; i < n; i++) {
      if ((i & 1) == 0) IrSender.mark(t[i]); else IrSender.space(t[i]);
    }
  }
}

// ---- sending one code of the table ----

// A NEC code in the usual address / command form
void irSendCode(uint16_t address, uint8_t command) {
  if (irSendNecReg(address, command, true)) return;
  irBoostBegin();
  irSendNec(address, command, true);
  irBoostEnd();
}

// A recorded NEC word that is not in the usual address / command form
void irSendWordCode(uint32_t word) {
  if (irSendNecRegWord(word, true)) return;
  irBoostBegin();
  irSendNecWord(word, true);
  irBoostEnd();
}

// A recorded code of any other kind: its mark / space times, as recorded
void irSendRawCode(int slot) {
  IrSlot& s = irSlots[slot];
  if (irSendRawReg(s.raw, s.rawLen, s.khz, s.frames, s.gapMs)) return;
  irBoostBegin();
  irSendRawLib(s.raw, s.rawLen, s.khz, s.frames, s.gapMs);
  irBoostEnd();
}

// Is this NEC word the usual form, i.e. what irNecRaw() makes from an address and a command? If so, say which.
bool irNecSplit(uint32_t w, uint16_t* address, uint8_t* command) {
  uint8_t cmd = (uint8_t)((w >> 16) & 0xFF);
  uint16_t addr = ((((w >> 8) & 0xFF) == (uint8_t)~(w & 0xFF))) ? (uint16_t)(w & 0xFF) : (uint16_t)(w & 0xFFFF);
  if (irNecRaw(addr, cmd) != w) return false;
  *address = addr;
  *command = cmd;
  return true;
}

// Send one code of the table (0 to 16). Does nothing for a code that has not been assigned.
void irSendSlot(int slot) {
  if (!irSlotAvailable(slot)) return;
  IrSlot& s = irSlots[slot];
  if (s.kind == IR_KIND_NEC) {
    uint16_t addr;
    uint8_t cmd;
    if (irNecSplit(s.word, &addr, &cmd)) irSendCode(addr, cmd);
    else irSendWordCode(s.word);
  } else {
    irSendRawCode(slot);
  }
}

// Make a label on the black-and-white screen look greyed out: every other pixel in the box is switched off
void irDimBox(int x, int y, int w, int h) {
  for (int py = y; py < y + h; py++) {
    for (int px = x; px < x + w; px++) {
      if (((px + py) & 1) == 0) display.drawPixel(px, py, BLACK);
    }
  }
}

// ---- the colour screen (LCARS look, see Lcars.ino) ----

// The selected button as a big block (dimmed when no code is assigned to it), whether it can be sent, and the keys
void irSendDrawTft() {
  bool ok = irSlotAvailable(irSendCursor);
  const char* label = irSlotLabels[irSendCursor];
  lcFrame("IR REMOTE", false);
  if (ok) {
    lcBigPill(8, 11, 88, 16, lcOrange(), label, 2);
    lcBigPill(8, 29, 88, 8, lcOk(), "READY TO SEND", 1);
  } else {
    tft.fillRoundRect(8, 11, 88, 16, 5, lcDim(lcOrange()));
    tft.setTextSize(2);
    tft.setTextColor(lcGrey(), lcDim(lcOrange()));
    tft.setCursor(8 + (88 - 12 * (int)strlen(label)) / 2, 11);
    tft.print(label);
    lcBigPill(8, 29, 88, 8, lcAlert(), "NOT SET", 1);
  }
  tft.setTextSize(1);
  tft.setTextColor(lcPeach(), BLACK);
  tft.setCursor(8, 40);
  tft.print("ENTER = SEND");
  tft.setTextColor(lcGrey(), BLACK);
  tft.setCursor(8, 48);
  tft.print("HOLD LT = EXIT");
  tft.setTextColor(lcGold(), BLACK);
  tft.setCursor(8, 56);
  tft.print("CODES: ");
  tft.print(irCodeSetName());
}

// What the colour screen says while a code is sent, or that there is no code to send
void irSendDrawSending(int slot, bool ok) {
  lcFrame("IR REMOTE", false);
  if (ok) {
    lcBigPill(8, 13, 88, 9, lcOk(), "SENDING...", 1);
    lcBigPill(8, 26, 88, 18, lcOrange(), irSlotLabels[slot], 2);
  } else {
    lcBigPill(8, 24, 88, 18, lcAlert(), "NOT SET", 2);
  }
}

// ---- the IR Send screen ----
void IrSend(int pressType) {
  static int lastIrSendCursor = -1;
  static bool irEntered = false;

  // Exit: Long Press Left (Button 5)
  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = COMMS_MENU;
    lastIrSendCursor = -1;
    irEntered = false;
    if (irRegState == 1) { IR_PIN_LOW(); }    // leave the pin low
    return;
  }

  // First pass after the screen opens: make sure the table is set up (the saved cloned set, or the defaults)
  if (!irEntered) {
    irEntered = true;
    irCodesEnsure();
  }

  // Navigation: Short Press Enter (Button 1): send the selected code
  if (pressType == SHORT_PRESS && lastButton == 1) {
    if (irSlotAvailable(irSendCursor)) {
      irSendDrawSending(irSendCursor, true);
      irSendSlot(irSendCursor);
      lcoBlockScreen("IR SEND", NULL, "SIGNAL SENT", true);
    } else {                                          // a button that was skipped when cloning, or that has no default
      irSendDrawSending(irSendCursor, false);
      lcoBlockScreen("IR SEND", NULL, "NOT ASSIGNED", false);
    }
    delay(500);
    lastIrSendCursor = -1;
  }

  // Scrolling: Up (Button 2) and Down (Button 4) move one, Right (Button 3) and Left (Button 5) jump a column (six)
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      irSendCursor = (irSendCursor - 1 + IR_SLOTS) % IR_SLOTS;
    } else if (lastButton == 4) { // Down
      irSendCursor = (irSendCursor + 1) % IR_SLOTS;
    } else if (lastButton == 3) { // Right - jump to the next column
      if (irSendCursor < 12) {
        irSendCursor = irSendCursor + 6;
        if (irSendCursor > IR_SLOTS - 1) irSendCursor = IR_SLOTS - 1;     // the last column is one row shorter
      }
    } else if (lastButton == 5) { // Left - jump to the previous column
      if (irSendCursor >= 6) {
        irSendCursor = irSendCursor - 6;
      }
    }
  }

  // --- Only update display if something changed ---
  if (irSendCursor != lastIrSendCursor) {

    // --- Draw to SSD1306 (LCARS look, see LcarsOled.ino): three columns of six, the selected button a solid block. A code that
    // is not assigned is drawn dimmed. ---
    display.clearDisplay();
    lcoFrame("IR SEND", irCodeSetName());
    display.setTextSize(1);

    for (int i = 0; i < IR_SLOTS; i++) {
      int col = i / 6;
      int row = i % 6;
      int xPos = 9 + col * 40;                 // three columns of 38 pixels
      int yPos = 16 + row * 8;                             // six rows of 8, from just under the header to the bottom

      if (i == irSendCursor) {
        display.fillRoundRect(xPos, yPos, 38, 8, 2, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
      } else {
        display.setTextColor(SSD1306_WHITE);
      }
      display.setCursor(xPos + 2, yPos);
      display.print(irSlotLabels[i]);
      if (!irSlotAvailable(i)) irDimBox(xPos, yPos, 38, 8);
    }

    display.setTextColor(SSD1306_WHITE);

    // --- Draw the selected button and the keys on the colour screen ---
    irSendDrawTft();

    lastIrSendCursor = irSendCursor;
  }

  display.display();
  delay(50);
}
