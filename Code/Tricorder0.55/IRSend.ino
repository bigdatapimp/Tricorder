// --------- IR Sender code ----------
//
// DIAGNOSTICS (added while tracking down an IR LED that stopped flashing):
//   - the colour screen shows which pin the IR LED is being driven on ("Pin D3") and how long the last
//     transmission took ("Sent: 112 ms"). A normal NEC code takes roughly 60-130 ms.
//   - the same is printed on the Serial Monitor (9600 baud) after every send.
//   - the pin's drive strength is shown after the pin number: "hi" high drive, "std" standard drive, "n/a" this build
//     can't reach the pin's register. After a send it shows two values, "hi>hi": the drive just before the send,
//     and just after it ("hi>std" would mean the IR library put the pin back to standard drive during the send).
//   - HOLD RIGHT = next sending method (LIB -> HI -> REG -> LIB), see the three methods below. The screen shows
//     which one is in use.
//   - HOLD DOWN = switch the thread priority boost on / off. While a code is sent, the Tricorder raises the priority of
//     the thread that runs loop() so the RTOS threads of the Bluetooth stack cannot interrupt it (see below).
//   - HOLD UP = self-test: times 20 marks of 560 us made by the library and by this file's own carrier, each with
//     and without the priority boost (min / average / max in microseconds; NEC wants 560), the worst lateness of
//     any edge in a 2 ms burst, which of the chip's PWM units are running and whether one drives the IR pin, the
//     pin's raw configuration and the Bluetooth state. Run it before and after Bluetooth is switched on. Run it before and after Bluetooth is
//     switched on for the first time.
//   - Every send also logs the pin's raw PIN_CNF register at four points (start, set, mid, end) on the Serial
//     Monitor, with the sending method and the Bluetooth state.
//   - HOLD ENTER on this screen = "IR beacon": for 3 seconds the Tricorder sends an unassigned code over and over,
//     so you can watch the LED with a phone camera. It sends none of the twelve codes below (so it won't change
//     anything on your TV) and it can only be started with the Tricorder's own buttons, not from the phone.

// HIGH DRIVE: the IR LED is wired straight from the pin through a 220 ohm resistor. The nRF52840 normally drives a
// pin at "standard" strength (rated about 2 mA); switching the pin to "high drive" (about 10 mA) lets more current
// through the LED, so it is brighter and reaches further. With 220 ohm the current can never exceed about
// (3.3 V - LED voltage) / 220 ohm = 9-10 mA, inside the 15 mA per pin limit Arduino lists for this board.
//
// Why the code below sends the NEC frame itself instead of calling IrSender.sendNEC(): at the start of every send
// the IR library prepares the pin with pinMode(), and this board's pinMode() rebuilds the pin, which puts the
// drive strength back to standard. The library's pulse functions (mark / space) don't touch the pin set-up, so
// irSendNec() lets the library prepare the pin (enableIROut), raises the drive AFTER that, and then sends the frame
// with the library's own mark() and space(). The signal is the same NEC frame plus repeat that sendNEC() sends.
//
// THREE SENDING METHODS (hold RIGHT on this screen to change; the colour screen shows which one is in use):
//   REG  this file's NEC sender with the 38 kHz carrier made by THIS FILE: direct writes to the pin's GPIO set / clear
//        registers, every edge timed against the CPU's cycle counter. No PWM driver, no digitalWrite(), nothing
//        reconfigures the pin while a frame is sent, so the pin keeps its high drive, and a delayed interrupt
//        (Bluetooth) can only disturb a single carrier cycle.
//   HI   this file's NEC sender using the library's own mark() / space() for the pulses, pin set to high drive first.
//   LIB  the library's own IrSender.sendNEC() (how it always worked before).
// WHY THERE IS A PRIORITY BOOST: with Bluetooth running, some of the library's 560 us marks measured up to 5 ms, which
// destroys the NEC timing (the TV stopped answering as soon as Bluetooth was switched on). The boost raises the thread
// that runs loop() above the Bluetooth threads for the ~120 ms a send takes. It is only a cure if the stalls really are
// thread switches; the self-test measures the marks with and without it so that can be seen.
// What was learnt on the way: on this board the IR library makes its carrier with the chip's PWM hardware (hardware PWM
// sending is on by default for mbed boards) and sets that up again around every mark, which is why the pin's drive
// setting did not survive a frame in the HI method. An attempt to claim a PWM output of our own froze the Tricorder
// on creation, so there is no PWM method any more.
#define IR_MODE_LIB 0
#define IR_MODE_HI  1
#define IR_MODE_REG 2
#ifndef IR_MODE_DEFAULT
#define IR_MODE_DEFAULT IR_MODE_REG            // the method used after every power-up (REG reached furthest in the range test)
#endif
int irMode = IR_MODE_DEFAULT;

const char* irModeName() { return irMode == IR_MODE_REG ? "REG" : (irMode == IR_MODE_HI ? "HI" : "LIB"); }

unsigned long irLastSendUs = 0;      // how long the last transmission took, in microseconds (0 = none sent yet)

// The nRF52840's pin registers. Most builds already see them; if the chip's own header exists, include it.
#if defined(__has_include)
#if __has_include(<nrf.h>)
#include <nrf.h>
#endif
#endif

#if defined(NRF_P0) && defined(NRF_P1)
#define IR_DRIVE_AVAILABLE 1         // the registers are visible by name
#else
#define IR_DRIVE_AVAILABLE 0
#endif

// If they are not visible by name, the register can be reached by its address instead (see irPinCnf()).
// Set to 0 to switch that off.
#ifndef IR_RAW_REGISTERS
#define IR_RAW_REGISTERS 1
#endif
#ifndef IR_REG32
#define IR_REG32(addr) (*(volatile uint32_t*)(addr))
#endif
#define IR_GPIO_P0_BASE   0x50000000UL   // nRF52840 GPIO port 0
#define IR_GPIO_P1_BASE   0x50000300UL   // nRF52840 GPIO port 1
#define IR_PIN_CNF_OFFSET 0x700UL        // PIN_CNF[0] inside a port; one 32-bit register per pin after it
#define IR_PIN_CNF_VALID  0x0003070FUL   // the only bits a real PIN_CNF register uses (DIR, INPUT, PULL, DRIVE, SENSE)
#define IR_DRIVE_MASK  (7UL << 8)        // PIN_CNF bits 10:8 hold the drive strength
#define IR_DRIVE_H0H1  (3UL << 8)        // 3 = high drive for both '0' and '1'

char irDriveNote[12] = "";               // what the last send did to the pin's drive, e.g. "hi>hi" (set > after the send)

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

// The pin's PIN_CNF register, or 0 if this build can't reach it safely
volatile uint32_t* irPinCnf() {
  int port, bit;
  if (!irPinLocate(IR_SEND_PIN, &port, &bit)) return 0;
#if IR_DRIVE_AVAILABLE
  NRF_GPIO_Type* gpio = (port == 0) ? NRF_P0 : NRF_P1;
  return &gpio->PIN_CNF[bit];
#elif IR_RAW_REGISTERS
  unsigned long base = (port == 0) ? IR_GPIO_P0_BASE : IR_GPIO_P1_BASE;
  volatile uint32_t* reg = &IR_REG32(base + IR_PIN_CNF_OFFSET + 4UL * bit);
  if ((*reg & ~IR_PIN_CNF_VALID) != 0) return 0;     // that doesn't look like a pin register: leave it alone
  return reg;
#else
  return 0;
#endif
}

// "hi" = the IR pin is on high drive, "std" = standard drive, "n/a" = this build can't reach the pin's register
const char* irDriveState() {
  volatile uint32_t* reg = irPinCnf();
  if (reg == 0) return "n/a";
  return ((*reg & IR_DRIVE_MASK) == IR_DRIVE_H0H1) ? "hi" : "std";
}

// Switch the IR LED's pin to high drive. Only the drive bits are changed; the rest of the pin's setup is kept.
bool irSetHighDrive() {
  volatile uint32_t* reg = irPinCnf();
  if (reg == 0) return false;
  *reg = (*reg & ~IR_DRIVE_MASK) | IR_DRIVE_H0H1;
  return true;
}

// ... and back to standard drive
bool irSetStandardDrive() {
  volatile uint32_t* reg = irPinCnf();
  if (reg == 0) return false;
  *reg = *reg & ~IR_DRIVE_MASK;
  return true;
}

// The pin's whole PIN_CNF register, for the log (bit 0 direction, 1 input buffer, 3:2 pull, 10:8 drive, 17:16 sense)
#define IR_CNF_NA   0xFFFFFFFFUL     // this build can't reach the register
#define IR_CNF_SKIP 0xFFFFFFFEUL     // not measured
uint32_t irCnfNow() {
  volatile uint32_t* reg = irPinCnf();
  return (reg == 0) ? IR_CNF_NA : (uint32_t)*reg;
}
const char* irCnfText(uint32_t v, char* buf) {
  if (v == IR_CNF_NA) return "n/a";
  if (v == IR_CNF_SKIP) return "-";
  snprintf(buf, 12, "0x%08lX", (unsigned long)v);
  return buf;
}
uint32_t irCnfStart = IR_CNF_SKIP, irCnfSet = IR_CNF_SKIP, irCnfMid = IR_CNF_SKIP, irCnfEnd = IR_CNF_SKIP;
unsigned long irSendCount = 0;

// The Bluetooth state, for the log: the stack is started the first time Bluetooth is switched on and then keeps
// running until the Tricorder is rebooted (switching Bluetooth off only stops advertising)
const char* irBleText() {
  return bleEnabled ? "BLE on" : (bleStarted ? "BLE stack running (switched off)" : "BLE never started");
}
const char* irBleShort() {
  return bleEnabled ? "on" : (bleStarted ? "running" : "never");
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

const char* irDriveBeforeSend = "n/a";        // the pin's drive just before the first pulse of the last send

// The 32 bits of a NEC frame, sent least significant bit first: address, then its inverse (or the high byte of a
// 16 bit address), then the command, then its inverse
uint32_t irNecRaw(uint16_t address, uint8_t command) {
  uint32_t raw = (address < 0x100) ? (((uint32_t)(uint8_t)~address << 8) | address) : (uint32_t)address;
  return raw | ((uint32_t)command << 16) | ((uint32_t)(uint8_t)~command << 24);
}

// Send one NEC frame (and, if asked, the repeat frame that follows it 110 ms later, like IrSender.sendNEC(.., 1))
// with the IR pin on high drive. See the explanation at the top of this file.
void irSendNec(uint16_t address, uint8_t command, bool withRepeat) {
  uint32_t raw = irNecRaw(address, command);
  irCnfStart = irCnfNow();
  IrSender.enableIROut(38);                  // the library prepares the pin and the 38 kHz carrier ...
  irSetHighDrive();                          // ... so the drive can only be raised after this
  irCnfSet = irCnfNow();
  irDriveBeforeSend = irDriveState();
  unsigned long frameStart = millis();
  IrSender.mark(IR_NEC_HEADER_MARK);
  IrSender.space(IR_NEC_HEADER_SPACE);
  for (int i = 0; i < 32; i++) {
    IrSender.mark(IR_NEC_BIT_MARK);
    IrSender.space((raw & 1UL) ? IR_NEC_ONE_SPACE : IR_NEC_ZERO_SPACE);
    raw >>= 1;
  }
  IrSender.mark(IR_NEC_BIT_MARK);            // stop bit
  irCnfMid = irCnfNow();
  if (withRepeat) {
    unsigned long elapsed = millis() - frameStart;
    if (elapsed < IR_NEC_REPEAT_PERIOD_MS) delay(IR_NEC_REPEAT_PERIOD_MS - elapsed);
    IrSender.mark(IR_NEC_HEADER_MARK);       // the NEC repeat frame
    IrSender.space(IR_NEC_REPEAT_HEADER_SPACE);
    IrSender.mark(IR_NEC_BIT_MARK);
  }
  irCnfEnd = irCnfNow();
}

// Print a step of the REG method and wait until it has really been sent, so that if the Tricorder freezes straight
// afterwards the Serial Monitor shows the last step reached. Only used outside the timing-critical part of a frame.
#define IR_TRACE 1
void irTrace(const char* step) {
#if IR_TRACE
  Serial.print("IR REG: ");
  Serial.println(step);
  Serial.flush();
#endif
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

bool irBoost = true;                        // use the boost? (hold DOWN to switch)
bool irBoosted = false;                     // is this thread boosted right now?
#if IR_RTOS_AVAILABLE
osThreadId_t irThreadId = 0;
osPriority_t irSavedPriority = osPriorityNormal;
#endif

// Raise (or put back) the priority of the thread running loop(). Nothing may be printed to the Serial Monitor
// while the thread is boosted: the USB serial code needs lower-priority threads to run.
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
void irBoostBegin() { if (irBoost) irBoostSet(true); }
void irBoostEnd()   { irBoostSet(false); }
const char* irBoostText() { return IR_RTOS_AVAILABLE ? (irBoost ? "boost on" : "boost off") : "boost n/a"; }

// ---- which of the chip's PWM units are running (read only) ----
// The IR library may use one of them for its carrier. A unit that drives the IR pin would hide this file's own pulses.
#if defined(NRF_PWM0) && defined(NRF_PWM1) && defined(NRF_PWM2) && defined(NRF_PWM3)
#define IR_PWM_AVAILABLE 1
#else
#define IR_PWM_AVAILABLE 0
#endif

// Writes a short description like "PWM0 off, PWM1 on, PWM2 on(D3), PWM3 off" into buf and returns how many running
// units drive the IR pin (-1 = this build can't see the PWM registers)
int irPwmReport(char* buf, int n) {
#if IR_PWM_AVAILABLE
  NRF_PWM_Type* units[4] = { NRF_PWM0, NRF_PWM1, NRF_PWM2, NRF_PWM3 };
  int port, bit;
  buf[0] = '\0';
  if (!irPinLocate(IR_SEND_PIN, &port, &bit)) return -1;
  uint32_t want = ((uint32_t)port << 5) | (uint32_t)bit;       // how the PWM registers name this pin
  int drivers = 0;
  int len = 0;
  for (int u = 0; u < 4; u++) {
    bool running = (units[u]->ENABLE & 1UL) != 0;
    bool mine = false;
    for (int c = 0; c < 4; c++) {
      uint32_t v = units[u]->PSEL.OUT[c];
      if (!(v & 0x80000000UL) && (v & 0x3FUL) == want) mine = true;
    }
    if (running && mine) drivers++;
    len += snprintf(buf + len, n - len, "%sPWM%d %s%s", u ? ", " : "", u, running ? "on" : "off", (running && mine) ? "(D3)" : "");
    if (len >= n - 1) break;
  }
  return drivers;
#else
  snprintf(buf, n, "PWM units: n/a");
  return -1;
#endif
}

// If a running PWM unit drives the IR pin (that is the IR library's carrier, and it stays connected after the library
// has sent something), stop it and let go of the pin so this file's own pulses can reach it. Only ever called by the
// REG method. Returns how many units were let go. After this the library's own sending may stay silent until the
// Tricorder is rebooted.
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

// ---- mode REG: the 38 kHz carrier made by this file ----
#ifndef IR_DWT_CYCCNT
#define IR_DWT_CTRL   (*(volatile uint32_t*)0xE0001000UL)      // ARM Cortex-M debug watchpoint unit: control
#define IR_DWT_CYCCNT (*(volatile uint32_t*)0xE0001004UL)      // ... and its CPU cycle counter
#define IR_DEMCR      (*(volatile uint32_t*)0xE000EDFCUL)      // ... which has to be switched on here (bit 24)
#endif
#define IR_GPIO_OUTSET_OFFSET 0x508UL                          // GPIO "set these pins high" register, inside a port
#define IR_GPIO_OUTCLR_OFFSET 0x50CUL                          // GPIO "set these pins low" register
#ifndef IR_PIN_HIGH
#define IR_PIN_HIGH() (*irOutSetReg = irPinMask)
#define IR_PIN_LOW()  (*irOutClrReg = irPinMask)
#endif

volatile uint32_t* irOutSetReg = 0;
volatile uint32_t* irOutClrReg = 0;
uint32_t irPinMask = 0;
int irRegState = 0;                     // 0 = not tried yet, 1 = ready, -1 = this build can't do it
unsigned long irCyclesPerUs = 0;        // the CPU clock in MHz, measured the first time (64 on this board)
uint32_t irPeriodCycles = 0;            // one 38 kHz carrier period in CPU cycles
uint32_t irOnCycles = 0;                // ... of which the pin is high for a third
uint32_t irCyc = 0;                     // the timeline: when the pulse in progress should end (cycle counter value)
uint32_t irRegWorstLate = 0;            // the latest any edge was, in cycles, since it was last cleared

// Where the pin's GPIO set / clear registers are
bool irRegLocate() {
  int port, bit;
  if (!irPinLocate(IR_SEND_PIN, &port, &bit)) return false;
  irPinMask = 1UL << bit;
#if IR_DRIVE_AVAILABLE
  NRF_GPIO_Type* gpio = (port == 0) ? NRF_P0 : NRF_P1;
  irOutSetReg = &gpio->OUTSET;
  irOutClrReg = &gpio->OUTCLR;
  return true;
#elif IR_RAW_REGISTERS
  unsigned long base = (port == 0) ? IR_GPIO_P0_BASE : IR_GPIO_P1_BASE;
  irOutSetReg = &IR_REG32(base + IR_GPIO_OUTSET_OFFSET);
  irOutClrReg = &IR_REG32(base + IR_GPIO_OUTCLR_OFFSET);
  return true;
#else
  return false;
#endif
}

// Start the CPU cycle counter and measure its speed against micros()
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
  irPeriodCycles = (uint32_t)((cpu * 1000000UL) / 38000UL);   // 1684 at 64 MHz = 26.3 us = 38.0 kHz
  irOnCycles = irPeriodCycles / 3;
  return true;
}

// Wait until the cycle counter reaches 'target', and remember how late we were
void irWaitCyc(uint32_t target) {
  while ((int32_t)(IR_DWT_CYCCNT - target) < 0) { }
  int32_t late = (int32_t)(IR_DWT_CYCCNT - target);
  if (late > 0 && (uint32_t)late > irRegWorstLate) irRegWorstLate = (uint32_t)late;
}

// Make the pin ready: set up the registers and the cycle counter once, then every send make sure it is an output on
// high drive. Returns false if this build can't do it (the caller then falls back to the HI method).
bool irRegReady() {
  if (irRegState < 0) return false;
  if (irRegState == 0) {
    irTrace("locating the pin registers");
    if (irPinCnf() == 0 || !irRegLocate()) { irRegState = -1; irTrace("pin registers not reachable"); return false; }
    irTrace("starting the cycle counter");
    if (!irCycleCounterReady()) { irRegState = -1; irTrace("cycle counter not working"); return false; }
    irTrace("ready");
    irRegState = 1;
  }
  if (irPwmRelease() > 0) irTrace("let go of a PWM unit that was driving the IR pin");
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

// The same NEC frame (and repeat 110 ms later) as irSendNec(), with the carrier made here. Returns false if this build
// can't do it.
bool irSendNecReg(uint16_t address, uint8_t command, bool withRepeat) {
  uint32_t raw = irNecRaw(address, command);
  irCnfStart = irCnfNow();
  if (!irRegReady()) return false;
  irCnfSet = irCnfNow();
  irDriveBeforeSend = irDriveState();
  irTrace("frame start");
  irBoostBegin();
  uint32_t frameStart = IR_DWT_CYCCNT;
  irCyc = frameStart;
  irRegWorstLate = 0;
  irRegMark(IR_NEC_HEADER_MARK);
  irRegSpace(IR_NEC_HEADER_SPACE);
  for (int i = 0; i < 32; i++) {
    irRegMark(IR_NEC_BIT_MARK);
    irRegSpace((raw & 1UL) ? IR_NEC_ONE_SPACE : IR_NEC_ZERO_SPACE);
    raw >>= 1;
  }
  irRegMark(IR_NEC_BIT_MARK);           // stop bit
  irBoostEnd();
  irCnfMid = irCnfNow();
  irTrace("frame sent");                // in the quiet gap before the repeat frame
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
  irCnfEnd = irCnfNow();
  irTrace("send finished");
  return true;
}

// Send one NEC code, timing only the transmission itself
void irSendCode(uint16_t address, uint8_t command) {
  const char* driveBefore;
  unsigned long t0 = micros();
  if (irMode == IR_MODE_REG) {
    if (irSendNecReg(address, command, true)) {
      driveBefore = irDriveBeforeSend;
    } else {                              // this build can't make its own carrier: send the HI way instead
      Serial.println("IR: REG method not available in this build, sending with HI instead");
      irBoostBegin();
      irSendNec(address, command, true);
      irBoostEnd();
      driveBefore = irDriveBeforeSend;
    }
  } else if (irMode == IR_MODE_HI) {
    irBoostBegin();
    irSendNec(address, command, true);
    irBoostEnd();
    driveBefore = irDriveBeforeSend;
  } else {
    irCnfStart = irCnfNow();
    irCnfSet = IR_CNF_SKIP;
    irCnfMid = IR_CNF_SKIP;
    driveBefore = irDriveState();
    irBoostBegin();
    IrSender.sendNEC(address, command, 1);
    irBoostEnd();
    irCnfEnd = irCnfNow();
  }
  irLastSendUs = micros() - t0;
  snprintf(irDriveNote, sizeof(irDriveNote), "%s>%s", driveBefore, irDriveState());
  irSendCount++;
  char c1[12], c2[12], c3[12], c4[12];
  Serial.print("IR: sent address 0x");
  Serial.print(address, HEX);
  Serial.print(" command 0x");
  Serial.print(command, HEX);
  Serial.print(" on pin D");
  Serial.print(IR_SEND_PIN);
  Serial.print(" in ");
  Serial.print(irLastSendUs / 1000UL);
  Serial.print(" ms, pin drive ");
  Serial.print(irDriveNote);
  Serial.print(" | #");
  Serial.print(irSendCount);
  Serial.print(" ");
  Serial.print(irModeName());
  Serial.print(" ");
  Serial.print(irBoostText());
  Serial.print(" | cnf start=");
  Serial.print(irCnfText(irCnfStart, c1));
  Serial.print(" set=");
  Serial.print(irCnfText(irCnfSet, c2));
  Serial.print(" mid=");
  Serial.print(irCnfText(irCnfMid, c3));
  Serial.print(" end=");
  Serial.print(irCnfText(irCnfEnd, c4));
  char pwm[64];
  irPwmReport(pwm, sizeof(pwm));
  Serial.print(" | ");
  Serial.print(pwm);
  Serial.print(" | ");
  Serial.println(irBleText());
}

// 20 marks of 560 us (the NEC bit mark) with 560 us spaces, each mark timed: minimum / average / maximum in microseconds.
// What the TV sees. The library's work around every mark makes them longer than asked, and Bluetooth can stretch some
// of them a lot more.
unsigned long irStatMin = 0, irStatAvg = 0, irStatMax = 0;
void irLibMarkStats() {
  unsigned long mn = 100000UL, mx = 0, sum = 0;
  for (int i = 0; i < 20; i++) {
    unsigned long a = micros();
    IrSender.mark(560);
    unsigned long m = micros() - a;
    if (m < mn) mn = m;
    if (m > mx) mx = m;
    sum += m;
    IrSender.space(560);
  }
  irStatMin = mn; irStatAvg = sum / 20UL; irStatMax = mx;
}
void irRegMarkStats() {                                 // the same with this file's carrier (needs irRegReady())
  unsigned long mn = 100000UL, mx = 0, sum = 0;
  irCyc = IR_DWT_CYCCNT;
  for (int i = 0; i < 20; i++) {
    unsigned long a = micros();
    irRegMark(560);
    unsigned long m = micros() - a;
    if (m < mn) mn = m;
    if (m > mx) mx = m;
    sum += m;
    irRegSpace(560);
  }
  irStatMin = mn; irStatAvg = sum / 20UL; irStatMax = mx;
}
void irPrintStat(const char* before) {
  Serial.print(before);
  Serial.print(irStatMin);
  Serial.print("/");
  Serial.print(irStatAvg);
  Serial.print("/");
  Serial.print(irStatMax);
  Serial.print(" us");
}

// Hold UP: time the library's marks and this file's own, with and without the priority boost, and see what the pin does
void irSelfTest() {
  char c1[12], c2[12];
  unsigned long a = micros();
  for (int i = 0; i < 100; i++) delayMicroseconds(10);
  unsigned long delayUs = micros() - a;                 // about 1000-1200 if delayMicroseconds() is accurate
  IrSender.enableIROut(38);
  if (irMode != IR_MODE_LIB) irSetHighDrive();
  uint32_t cnfBefore = irCnfNow();
  a = micros();
  IrSender.mark(2000);                                  // a 2 ms burst of the library's carrier
  unsigned long markUs = micros() - a;                  // expect about 2000-2100
  uint32_t cnfAfter = irCnfNow();
  irLibMarkStats();
  unsigned long lMin = irStatMin, lAvg = irStatAvg, lMax = irStatMax;
  unsigned long bMin = 0, bAvg = 0, bMax = 0;
  bool haveBoost = (IR_RTOS_AVAILABLE != 0);
  if (haveBoost) {
    irBoostSet(true);
    irLibMarkStats();
    irBoostSet(false);
    bMin = irStatMin; bAvg = irStatAvg; bMax = irStatMax;
  }
  Serial.print("IR self-test: 100 x delayMicroseconds(10) took ");
  Serial.print(delayUs);
  Serial.print(" us (expect about 1000-1200), a 2000 us mark took ");
  Serial.print(markUs);
  Serial.print(" us (expect about 2000-2100), 20 marks of 560 us took min/avg/max ");
  Serial.print(lMin);
  Serial.print("/");
  Serial.print(lAvg);
  Serial.print("/");
  Serial.print(lMax);
  Serial.print(" us (NEC wants 560)");
  if (haveBoost) {
    Serial.print(", with the thread priority raised ");
    Serial.print(bMin);
    Serial.print("/");
    Serial.print(bAvg);
    Serial.print("/");
    Serial.print(bMax);
    Serial.print(" us");
  } else {
    Serial.print(", thread priority boost not available in this build");
  }
  Serial.print(", pin cnf ");
  Serial.print(irCnfText(cnfBefore, c1));
  Serial.print(" -> ");
  Serial.print(irCnfText(cnfAfter, c2));
  Serial.print(" | ");
  Serial.print(irModeName());
  Serial.print(" | ");
  Serial.println(irBleText());
  // this file's own carrier. Which PWM units run is read first (the library's marks above leave one connected to the
  // pin); REG then lets go of it.
  char pwm[64];
  int drivers = irPwmReport(pwm, sizeof(pwm));
  bool regOk = irRegReady();            // (the first time it prints its set-up steps, so do it before the line below)
  unsigned long rMin = 0, rAvg = 0, rMax = 0, sMin = 0, sAvg = 0, sMax = 0;
  unsigned long lateTenthsUs = 0;
  if (regOk) {
    irCyc = IR_DWT_CYCCNT;
    irRegWorstLate = 0;
    irRegMark(2000);
    lateTenthsUs = (unsigned long)irRegWorstLate * 10UL / irCyclesPerUs;
    irRegMarkStats();
    rMin = irStatMin; rAvg = irStatAvg; rMax = irStatMax;
    if (haveBoost) {
      irBoostSet(true);
      irRegMarkStats();
      irBoostSet(false);
      sMin = irStatMin; sAvg = irStatAvg; sMax = irStatMax;
    }
    IR_PIN_LOW();
  }
  Serial.print("IR REG test: ");
  if (regOk) {
    Serial.print("cycle counter ");
    Serial.print(irCyclesPerUs);
    Serial.print(" cycles per us, carrier period ");
    Serial.print((unsigned long)irPeriodCycles);
    Serial.print(" cycles, worst edge lateness in a 2 ms burst ");
    Serial.print(lateTenthsUs / 10UL);
    Serial.print(".");
    Serial.print(lateTenthsUs % 10UL);
    Serial.print(" us (small is good), 20 REG marks of 560 us took min/avg/max ");
    Serial.print(rMin);
    Serial.print("/");
    Serial.print(rAvg);
    Serial.print("/");
    Serial.print(rMax);
    Serial.print(" us");
    if (haveBoost) {
      Serial.print(", with the thread priority raised ");
      Serial.print(sMin);
      Serial.print("/");
      Serial.print(sAvg);
      Serial.print("/");
      Serial.print(sMax);
      Serial.print(" us");
    }
  } else {
    Serial.print("not available in this build");
  }
  Serial.print(" | ");
  Serial.print(pwm);
  if (drivers > 0) Serial.print(regOk ? "  <- a running PWM unit drove the IR pin; REG let go of it" : "  <- a running PWM unit drives the IR pin");
  Serial.println();
  tft.fillScreen(BLACK);
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(0, 0);
  tft.println("IR SELF-TEST");
  tft.setTextColor(WHITE);
  tft.setCursor(0, 11);
  tft.print("lib  ");
  tft.print(lMin);
  tft.print("-");
  tft.println(lMax);
  tft.setCursor(0, 21);
  tft.print("lib+ ");
  if (haveBoost) { tft.print(bMin); tft.print("-"); tft.println(bMax); } else tft.println("n/a");
  tft.setCursor(0, 31);
  tft.print("reg  ");
  if (regOk) { tft.print(rMin); tft.print("-"); tft.println(rMax); } else tft.println("n/a");
  tft.setCursor(0, 41);
  tft.print("reg+ ");
  if (regOk && haveBoost) { tft.print(sMin); tft.print("-"); tft.println(sMax); } else tft.println("n/a");
  tft.setCursor(0, 51);
  tft.print("BLE ");
  tft.print(irBleShort());
  tft.print(" ");
  tft.println(irDriveState());
  unsigned long until = millis() + 3000UL;              // leave the numbers on the screen for 3 s
  while ((long)(until - millis()) > 0) {
    delay(50);
    bleUpdate();
  }
}


void IrSend(int pressType) {
  static int lastIrSendCursor = -1; 
  static bool irEntered = false;
  
  uint8_t sRepeats = 1;
  
  // --- Original Options ---
  uint16_t pwr_Address = 0x4;
  uint8_t pwr_Command = 0x8;
  uint16_t vup_Address = 0x4;
  uint8_t vup_Command = 0x2;
  uint16_t vdn_Address = 0x4;
  uint8_t vdn_Command = 0x3;
  
  // --- New Options ---
  uint16_t ok_Address = 0x4;
  uint8_t ok_Command = 0x5A;
  
  uint16_t dlt_Address = 0x4;
  uint8_t dlt_Command = 0x58;
  
  uint16_t drt_Address = 0x4;
  uint8_t drt_Command = 0x59;
  
  uint16_t ddn_Address = 0x4;
  uint8_t ddn_Command = 0x57;
  
  uint16_t dup_Address = 0x4;
  uint8_t dup_Command = 0x56;
  
  uint16_t amzn_Address = 0x4;
  uint8_t amzn_Command = 0x47;
  
  uint16_t yt_Address = 0x4;
  uint8_t yt_Command = 0x49;

  uint16_t bck_Address = 0x4;
  uint8_t bck_Command = 0x4;

  uint16_t ext_Address = 0x4;
  uint8_t ext_Command = 0x4D;
  
  // --- Handle Button Inputs ---
  
  // Exit: Long Press Left (Button 5)
  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = COMMS_MENU;
    lastIrSendCursor = -1;
    irEntered = false;
    if (irRegState == 1) { IR_PIN_LOW(); }    // leave the pin low
    return;
  }

  // First pass after the screen opens: put the pin on high drive now, so the readout means something before
  // the first send ("hi" = it took, "std" = it did not stick, "n/a" = this build can't reach the register)
  if (!irEntered) {
    irEntered = true;
    if (irMode != IR_MODE_LIB) irSetHighDrive();
    irDriveNote[0] = '\0';
    Serial.print("IR: screen opened, pin D");
    Serial.print(IR_SEND_PIN);
    Serial.print(" drive ");
    Serial.println(irDriveState());
  }

  // Hold RIGHT: next sending method (LIB -> HI -> REG -> LIB)
  if (pressType == LONG_PRESS && lastButton == 3) {
    irMode = (irMode + 1) % 3;
    if (irMode == IR_MODE_LIB) irSetStandardDrive(); else irSetHighDrive();
    irDriveNote[0] = '\0';
    Serial.print("IR: sending method is now ");
    Serial.println(irMode == IR_MODE_REG ? "REG (this file's NEC sender and carrier: direct register writes, high drive)"
                 : (irMode == IR_MODE_HI ? "HI (this file's NEC sender, the library's mark/space, high drive)"
                                         : "LIB (the library's sendNEC, standard drive)"));
    lastIrSendCursor = -1;                    // redraw the screen
    return;
  }

  // Hold DOWN: the priority boost on / off
  if (pressType == LONG_PRESS && lastButton == 4) {
    irBoost = !irBoost;
    Serial.print("IR: priority boost is now ");
    Serial.println(irBoostText());
    lastIrSendCursor = -1;                    // redraw the screen
    return;
  }

  // Hold UP: self-test
  if (pressType == LONG_PRESS && lastButton == 2) {
    irSelfTest();
    lastIrSendCursor = -1;
    return;
  }

  // Diagnostic "IR beacon": hold ENTER for 3 s of repeated transmissions of an unassigned code
  if (pressType == LONG_PRESS && lastButton == 1) {
    tft.fillScreen(BLACK);
    tft.setCursor(0, 8);
    tft.setTextSize(1);
    tft.setTextColor(YELLOW);
    tft.println("IR BEACON");
    tft.setTextColor(WHITE);
    tft.println("Sending for 3 s");
    tft.print("on pin D");
    tft.println(IR_SEND_PIN);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println("IR beacon...");
    display.display();
    Serial.print("IR: beacon on pin D");
    Serial.println(IR_SEND_PIN);
    unsigned long beaconEnd = millis() + 3000UL;
    while ((long)(beaconEnd - millis()) > 0) {
      if (irMode == IR_MODE_REG && irSendNecReg(0x7A5A, 0x3C, false)) {   // an address and command that no device is likely to answer to
        // sent by this file's own carrier
      } else if (irMode != IR_MODE_LIB) {
        irBoostBegin();
        irSendNec(0x7A5A, 0x3C, false);
        irBoostEnd();
      } else {
        irBoostBegin();
        IrSender.sendNEC(0x7A5A, 0x3C, 0);
        irBoostEnd();
      }
      delay(30);
      bleUpdate();                            // keep the Bluetooth link serviced
    }
    lastIrSendCursor = -1;                    // redraw the screen
    return;
  }

  // Navigation: Short Press Enter (Button 1)
  if (pressType == SHORT_PRESS && lastButton == 1) {
    tft.fillScreen(BLACK);
    tft.setCursor(0, 20);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    
    switch (irSendCursor) {
      case 0: // Power
        tft.println("Sending PWR...");
        irSendCode(pwr_Address, pwr_Command);                
        break;
      case 1: // Vol UP
        tft.println("Sending VOL+");              
        irSendCode(vup_Address, vup_Command);              
        break;
      case 2: // Vol DN
        tft.println("Sending VOL-");
        irSendCode(vdn_Address, vdn_Command);                
        break;
      case 3: // OK
        tft.println("Sending OK...");
        irSendCode(ok_Address, ok_Command);                
        break;      
      case 4: // BACK
        tft.println("Sending BACK...");
        irSendCode(bck_Address, bck_Command);                
        break;
      case 5: // EXIT
        tft.println("Sending EXIT...");
        irSendCode(ext_Address, ext_Command);                
        break;   
      case 6: // D-Lt
        tft.println("Sending D-Lt...");
        irSendCode(dlt_Address, dlt_Command);                
        break;
      case 7: // D-Rt
        tft.println("Sending D-Rt...");
        irSendCode(drt_Address, drt_Command);                
        break;
      case 8: // D-Dn
        tft.println("Sending D-Dn...");
        irSendCode(ddn_Address, ddn_Command);                
        break;
      case 9: // D-Up
        tft.println("Sending D-Up...");
        irSendCode(dup_Address, dup_Command);                
        break;
      case 10: // Amzn
        tft.println("Sending Amzn...");
        irSendCode(amzn_Address, amzn_Command);                
        break;
      case 11: // YT
        tft.println("Sending YT...");
        irSendCode(yt_Address, yt_Command);                
        break;
    }
    
    // Visual feedback on SSD1306
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("Signal Sent!");
    display.display();
    
    delay(500); 
    
    lastIrSendCursor = -1; 
  }

  // Scrolling: Up (Button 2), Down (Button 4), Left (Button 5), Right (Button 3)
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      irSendCursor = (irSendCursor - 1 + 12) % 12;
    } else if (lastButton == 4) { // Down
      irSendCursor = (irSendCursor + 1) % 12;
    } else if (lastButton == 3) { // Right - Jump to next column (Jump +4)
      // If we are in the first or second column (indices 0-7), jump to the next column
      if (irSendCursor < 8) {
        irSendCursor = irSendCursor + 4; 
      }
    } else if (lastButton == 5) { // Left - Jump to previous column (Jump -4)
      // If we are in the second or third column (indices 4-11), jump to the previous column
      if (irSendCursor >= 4) {
        irSendCursor = irSendCursor - 4; 
      }
    }
  }

  // --- Only update display if something changed ---
  if (irSendCursor != lastIrSendCursor) {
    
    // --- Draw to SSD1306 (Three Columns) ---
    display.clearDisplay();
    display.setTextSize(0);
    display.setCursor(0, 0);
    
    const char* options[12] = {
      "1.PWR", "2.VUP", "3.VDN", "4.OK", 
      "5.BACK", "6.EXIT", "7.Lft", "8.Rt", 
      "9.Dn", "10.Up", "11.Amz", "12.YT"
    };

    for (int i = 0; i < 12; i++) {
      // Calculate column and row (3 cols, 4 rows)
      int col = i / 4;      
      int row = i % 4;      
      
      // Screen width 128. 128/3 = ~42 pixels per column
      int xPos = col * 42; 
      int yPos = 9 + (row * 10);
      
      display.setCursor(xPos, yPos);
      
      if (i == irSendCursor) {
        display.print(F(">"));
        display.setTextColor(BLACK);
        // Width changed from 64 to 42 to fit 3 columns
        display.fillRect(xPos, yPos - 1, 42, 10, SSD1306_WHITE);
      } else {
        display.print(F(" "));
        display.setTextColor(SSD1306_WHITE);
      }
      
      display.println(options[i]);
    }
    
    
    // --- Draw instructions on Color Screen ---
    tft.fillScreen(BLACK);
    tft.setCursor(0, 0);
    tft.setTextSize(1);
    tft.setTextColor(CYAN);
    tft.println("IR REMOTE");
    tft.setTextSize(0);
    tft.setTextColor(WHITE);
    tft.setCursor(0, 20);
    tft.println("Hold LT to exit");
    tft.setTextColor(GREEN);
    tft.println("Press ENTER to");
    tft.println("Select Code:");
    tft.setTextColor(YELLOW);
    tft.setCursor(0, 46);
    tft.print("Pin D");
    tft.print(IR_SEND_PIN);
    tft.print(" ");
    tft.print(irDriveNote[0] ? irDriveNote : irDriveState());
    tft.setCursor(0, 55);
    tft.print("Sent:");
    if (irLastSendUs == 0) {
      tft.print("--");
    } else {
      tft.print(irLastSendUs / 1000UL);
      tft.print("ms");
    }
    tft.print(" ");
    tft.print(irModeName());
    if (irBoost && IR_RTOS_AVAILABLE) tft.print("+");
    
    lastIrSendCursor = irSendCursor;
  }
  
  display.display();
  delay(50);
}