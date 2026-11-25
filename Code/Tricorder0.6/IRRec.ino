// --------- IR codes, IR Receive and remote cloning ----------
//
// This file holds three things:
//
//  1. THE CODE TABLE. The 17 IR codes that the IR Send screen and the phone app send (irSlots[]):
//       0 PWR   1 VOL+   2 VOL-   3 OK   4 BACK   5 EXIT   6 Left   7 Right   8 Down   9 Up   10 Amazon   11 YouTube
//       12 Eject   13 Play   14 Stop   15 Pause   16 Menu
//     It starts as the built-in defaults: the twelve NEC codes (address 0x04) that IR Send has always had. Eject, Play,
//     Stop, Pause and Menu have no default code, so they start "not assigned": the IR Send screen shows them dimmed,
//     sends nothing for them, and the phone app greys the key out.
//     Cloning a remote (below) replaces the whole table. It is kept in flash (mbed's key-value store, the same one the
//     time zone uses), so a cloned set is still there after power-off. If this core has no key-value store, or a save
//     fails, the screen says so and the cloned set lasts until the next power-off.
//
//  2. THE IR RECEIVER. Reads the receiver on pin IR_RECV_PIN (D2, the same pin and wiring as always) and records the
//     length of every mark and space of a frame. Because the raw times are kept, any remote can be cloned, whatever
//     its protocol. NEC and Samsung frames are recognised (NEC codes are stored as their 32-bit word and sent like the
//     built-in codes, with the NEC repeat frame); Sony frames are recognised and replayed three times at 40 kHz as
//     Sony devices expect; everything else is replayed exactly as it was recorded, once, at 38 kHz.
//
//  3. THE IR RECEIVE SCREEN (COMMS menu):
//       1. Clone remote      asks for the 17 buttons one at a time and records each from the remote you point at the
//                            receiver. RIGHT = skip this button, UP = go back one, LEFT = cancel (asks first).
//                            A skipped button is shown as not assigned. When the last button is done the new set
//                            replaces the codes in use; cancelling throws away what was recorded and leaves the codes
//                            that were in use as they were.
//       2. Receive IR data   shows whatever the receiver sees: protocol, value and number of pulses (also on Serial).
//     UP / DOWN = choose, ENTER = select, hold LEFT = leave the screen. On the menu, hold DOWN = go back to the
//     built-in default codes (asks first).
//     Hint: do the cloning before switching Bluetooth on; the Bluetooth stack can delay the receiver by a few ms.
//     The phone app has the same two options. It can open this screen and press its buttons (Remote.ino), start a clone
//     or open the data view with irRxRemoteAction(), and it watches everything through the IR Receive packet that
//     irRxPacket() builds (Bluetooth.ino).

#if defined(__has_include)
  #if __has_include("kvstore_global_api.h")
    #include "kvstore_global_api.h"
    #define IR_HAVE_KV 1
  #endif
#endif
#ifndef IR_HAVE_KV
  #define IR_HAVE_KV 0
#endif

#define IR_SLOTS            17        // codes in the table
#define IR_RAW_MAX          120       // most mark/space times one recorded code can hold (a NEC frame needs 67)
#define IR_KIND_NONE        0         // not assigned
#define IR_KIND_NEC         1         // a 32-bit NEC word
#define IR_KIND_RAW         2         // recorded mark/space times
#define IR_REC_VERSION      1         // layout of a record in flash
#define IR_KV_MARK_KEY      "/kv/ir_set"       // present when a cloned set has been saved

#define IR_RX_MAX_MARK_US   15000UL   // a mark longer than this is not an IR signal
#define IR_RX_END_GAP_US    8000UL    // a space this long ends a frame (inside a frame the longest space is 4.5 ms)
#define IR_RX_QUIET_MS      150UL     // the line must be quiet this long before the next press is taken
#define IR_RX_SPIN_US       4000UL    // how long one pass of the screen watches the pin

// Results of a poll of the receiver
#define IR_RX_NONE          0
#define IR_RX_OK            1         // a frame was recorded (irRxBuf / irRxLen)
#define IR_RX_NOISE         2         // something on the pin, but not an IR frame
#define IR_RX_TOOLONG       3         // a frame with more than IR_RAW_MAX marks and spaces

// Protocols that are recognised
#define IR_P_UNKNOWN        0
#define IR_P_NEC            1
#define IR_P_NECREP         2         // the short NEC "button is still held" frame
#define IR_P_SAMSUNG        3
#define IR_P_SONY           4

// Screens of IR Receive
#define IR_RX_MENU          0
#define IR_RX_DATA          1
#define IR_RX_CLONE         2
#define IR_RX_CONFIRM       3         // "Discard the new codes?"
#define IR_RX_RESTORE       4         // "Go back to the default codes?"
#define IR_RX_DONE          5         // a result is shown (see irRxDoneKind)

struct IrSlot {
  uint8_t kind;                       // IR_KIND_*
  uint8_t khz;                        // carrier frequency
  uint8_t frames;                     // how many times the frame is sent (3 for Sony, otherwise 1)
  uint8_t gapMs;                      // when frames > 1: time from the start of one frame to the start of the next
  uint8_t rawLen;                     // IR_KIND_RAW: number of mark/space times
  uint32_t word;                      // IR_KIND_NEC: the 32 bits, first bit sent = bit 0
  uint16_t raw[IR_RAW_MAX];           // IR_KIND_RAW: mark, space, mark ... in microseconds (the first is a mark)
};

IrSlot irSlots[IR_SLOTS];             // the codes in use
IrSlot irCloneSlots[IR_SLOTS];        // the set being recorded by "Clone remote"
bool irCodesLoaded = false;           // has the table been set up (defaults or the saved set)?
bool irCodesCustom = false;           // is a cloned set in use?

// Short names for the IR Send grid (5 characters at most), full names for the prompts
const char* const irSlotLabels[IR_SLOTS] = {"PWR", "VOL+", "VOL-", "OK", "BACK", "EXIT", "Left", "Right", "Down", "Up",
                                            "Amzn", "YT", "Ejct", "Play", "Stop", "Paus", "Menu"};
const char* const irSlotNames[IR_SLOTS] = {"Power", "Vol Up", "Vol Down", "OK", "Back", "Exit", "Left", "Right", "Down", "Up",
                                           "Amazon", "YouTube", "Eject", "Play", "Stop", "Pause", "Menu"};
// The order the clone asks for the buttons (the slot numbers themselves never change: the phone app relies on them)
const uint8_t irCloneOrder[IR_SLOTS] = {0, 1, 2, 3, 4, 5, 6, 7, 9, 8, 10, 11, 12, 13, 14, 15, 16};
// Commands of the built-in NEC codes (address 0x04) for slots 0 to 11
const uint8_t irDefaultCmds[12] = {0x08, 0x02, 0x03, 0x5A, 0x04, 0x4D, 0x58, 0x59, 0x57, 0x56, 0x47, 0x49};

// ---------------------------------------------------------------- the table

void irSlotClear(int which, int slot) {             // which: 0 = the codes in use, 1 = the clone being recorded
  IrSlot* t = which ? irCloneSlots : irSlots;
  memset(&t[slot], 0, sizeof(IrSlot));
  t[slot].khz = 38;
  t[slot].frames = 1;
}

void irSetDefaults() {
  for (int i = 0; i < IR_SLOTS; i++) {
    irSlotClear(0, i);
    if (i < 12) {
      irSlots[i].kind = IR_KIND_NEC;
      irSlots[i].word = irNecRaw(0x04, irDefaultCmds[i]);
    }
  }
  irCodesCustom = false;
}

// ---------------------------------------------------------------- saving and loading (flash key-value store)

// Recorded times are kept in steps of 10 us up to 2540 us (plenty: receivers tolerate far more), exact above that
uint16_t irQuantize(unsigned long us) {
  if (us > 65535UL) us = 65535UL;
  if (us <= 2544UL) {
    us = ((us + 5UL) / 10UL) * 10UL;
    if (us < 10UL) us = 10UL;
  }
  return (uint16_t)us;
}

// A record in flash: version, kind, khz, frames, gapMs, then for NEC the 4 bytes of the word, for RAW the number of
// times and the times (one byte of 10 us steps, or 0xFF followed by the exact time in 2 bytes). Returns its length.
int irRecordEncode(int slot, uint8_t* out) {
  IrSlot& s = irSlots[slot];
  int p = 0;
  out[p++] = IR_REC_VERSION;
  out[p++] = s.kind;
  out[p++] = s.khz;
  out[p++] = s.frames;
  out[p++] = s.gapMs;
  if (s.kind == IR_KIND_NEC) {
    for (int i = 0; i < 4; i++) out[p++] = (uint8_t)((s.word >> (8 * i)) & 0xFF);
  } else if (s.kind == IR_KIND_RAW) {
    out[p++] = s.rawLen;
    for (int i = 0; i < s.rawLen; i++) {
      uint16_t d = s.raw[i];
      if (d >= 10 && d <= 2540 && (d % 10) == 0) {
        out[p++] = (uint8_t)(d / 10);
      } else {
        out[p++] = 0xFF;
        out[p++] = (uint8_t)(d & 0xFF);
        out[p++] = (uint8_t)(d >> 8);
      }
    }
  }
  return p;
}

// The other way. Only fills the slot if the whole record makes sense.
bool irRecordDecode(int slot, const uint8_t* in, int len) {
  if (len < 5 || in[0] != IR_REC_VERSION) return false;
  uint8_t kind = in[1], khz = in[2], frames = in[3], gap = in[4];
  if (kind > IR_KIND_RAW || khz < 30 || khz > 60 || frames < 1 || frames > 5) return false;
  IrSlot t;
  memset(&t, 0, sizeof(t));
  t.kind = kind; t.khz = khz; t.frames = frames; t.gapMs = gap;
  int p = 5;
  if (kind == IR_KIND_NEC) {
    if (len < p + 4) return false;
    for (int i = 0; i < 4; i++) t.word |= ((uint32_t)in[p + i]) << (8 * i);
  } else if (kind == IR_KIND_RAW) {
    if (len < p + 1) return false;
    int n = in[p++];
    if (n < 3 || n > IR_RAW_MAX) return false;
    for (int i = 0; i < n; i++) {
      if (p >= len) return false;
      uint8_t b = in[p++];
      uint16_t d;
      if (b == 0xFF) {
        if (p + 2 > len) return false;
        d = (uint16_t)(in[p] | ((uint16_t)in[p + 1] << 8));
        p += 2;
      } else if (b == 0) {
        return false;
      } else {
        d = (uint16_t)b * 10;
      }
      if (d == 0) return false;
      t.raw[i] = d;
    }
    t.rawLen = (uint8_t)n;
  }
  irSlots[slot] = t;
  return true;
}

// Save the codes in use. True only if everything was really stored.
bool irCodesSave() {
#if IR_HAVE_KV
  uint8_t rec[368];
  char key[16];
  for (int i = 0; i < IR_SLOTS; i++) {
    int n = irRecordEncode(i, rec);
    snprintf(key, sizeof(key), "/kv/ir_s%02d", i);
    int r = kv_set(key, rec, n, 0);
    if (r != 0) {
      Serial.print("IR codes: save failed at slot ");
      Serial.print(i);
      Serial.print(", code ");
      Serial.println(r);
      return false;
    }
  }
  uint8_t v = IR_REC_VERSION;                         // the marker goes in last: a half-finished save is never used
  int r = kv_set(IR_KV_MARK_KEY, &v, 1, 0);
  if (r != 0) {
    Serial.print("IR codes: save failed, code ");
    Serial.println(r);
    return false;
  }
  return true;
#else
  return false;
#endif
}

// Load a saved cloned set into the table. False (table left as the defaults) if there isn't one or it is damaged.
bool irCodesLoadSaved() {
#if IR_HAVE_KV
  uint8_t v = 0;
  size_t actual = 0;
  int r = kv_get(IR_KV_MARK_KEY, &v, 1, &actual);
  if (r != 0 || actual != 1 || v != IR_REC_VERSION) return false;
  uint8_t rec[368];
  char key[16];
  for (int i = 0; i < IR_SLOTS; i++) {
    snprintf(key, sizeof(key), "/kv/ir_s%02d", i);
    actual = 0;
    r = kv_get(key, rec, sizeof(rec), &actual);
    if (r != 0 || !irRecordDecode(i, rec, (int)actual)) {
      Serial.print("IR codes: saved set is damaged at slot ");
      Serial.println(i);
      irSetDefaults();
      return false;
    }
  }
  return true;
#else
  return false;
#endif
}

// Set the table up the first time anything needs it: the saved cloned set if there is one, otherwise the defaults
void irCodesEnsure() {
  if (irCodesLoaded) return;
  irCodesLoaded = true;
  irSetDefaults();
  if (irCodesLoadSaved()) {
    irCodesCustom = true;
    Serial.println("IR codes: restored the cloned set from flash");
  } else {
    Serial.println("IR codes: using the default set");
  }
}

// Back to the built-in defaults. True if the saved set is gone too (so the defaults are also what the next power-up gets).
bool irCodesRestoreDefaults() {
  irCodesLoaded = true;
  irSetDefaults();
#if IR_HAVE_KV
  uint8_t v = 0;
  size_t actual = 0;
  if (kv_get(IR_KV_MARK_KEY, &v, 1, &actual) != 0) return true;        // nothing was saved
  int r = kv_remove(IR_KV_MARK_KEY);
  if (r != 0) {
    Serial.print("IR codes: could not remove the saved set, code ");
    Serial.println(r);
    return false;
  }
  return true;
#else
  return true;
#endif
}

bool irSlotAvailable(int slot) {
  irCodesEnsure();
  return slot >= 0 && slot < IR_SLOTS && irSlots[slot].kind != IR_KIND_NONE;
}

// Bit n set = code n is assigned. Sent to the phone in the Device packet so the app can grey out the other keys.
uint32_t irAvailableMask() {
  irCodesEnsure();
  uint32_t m = 0;
  for (int i = 0; i < IR_SLOTS; i++) {
    if (irSlots[i].kind != IR_KIND_NONE) m |= (1UL << i);
  }
  return m;
}

const char* irCodeSetName() {
  irCodesEnsure();
  return irCodesCustom ? "CLONED" : "DEFAULT";
}

// ---------------------------------------------------------------- the receiver

uint16_t irRxBuf[IR_RAW_MAX + 1];     // the frame just recorded: mark, space, mark ... (always ends with a mark)
int irRxLen = 0;
bool irRxArmed = false;               // is the receiver ready for a new press?
unsigned long irRxQuietSince = 0;

// The pin has just been seen low, 'startUs' is when. Record the mark and space lengths until a long gap ends the frame.
int irCaptureFrame(unsigned long startUs) {
  irRxLen = 0;
  int level = LOW;
  unsigned long t = startUs;
  int result = IR_RX_NOISE;
  irBoostBegin();                                       // nothing is printed in here: the thread may be boosted
  for (;;) {
    unsigned long limit = (level == LOW) ? IR_RX_MAX_MARK_US : IR_RX_END_GAP_US;
    int v;
    unsigned long now;
    for (;;) {
      v = digitalRead(IR_RECV_PIN);
      now = micros();
      if (v != level) break;
      if ((now - t) > limit) break;
    }
    if (v == level) {                                   // no edge: a timeout
      if (level == HIGH) result = (irRxLen >= 3) ? IR_RX_OK : IR_RX_NOISE;     // the gap after the last mark: the frame is over
      else result = IR_RX_NOISE;                        // stuck low: not an IR signal
      break;
    }
    if (irRxLen >= IR_RAW_MAX) { result = IR_RX_TOOLONG; break; }
    irRxBuf[irRxLen++] = irQuantize(now - t);
    t = now;
    level = v;
  }
  irBoostEnd();
  return result;
}

// Watch the pin for a few milliseconds. Returns IR_RX_NONE if nothing started, otherwise the result of the capture.
int irRxPoll() {
  if (!irRxArmed) {                                     // after a frame the line must be quiet for a while, so that the
    if (digitalRead(IR_RECV_PIN) == LOW) {              // repeats of a held button are not taken for the next press
      irRxQuietSince = millis();
      return IR_RX_NONE;
    }
    if ((millis() - irRxQuietSince) < IR_RX_QUIET_MS) {
      delay(1);                                         // nothing to watch yet: don't hammer the pin
      return IR_RX_NONE;
    }
    irRxArmed = true;
  }
  unsigned long t0 = micros();
  while ((micros() - t0) < IR_RX_SPIN_US) {
    if (digitalRead(IR_RECV_PIN) == LOW) {
      unsigned long a = micros();
      bool real = true;
      while ((micros() - a) < 100UL) {                  // a real mark lasts at least 100 us; less is a glitch
        if (digitalRead(IR_RECV_PIN) == HIGH) { real = false; break; }
      }
      if (!real) continue;
      int r = irCaptureFrame(a);
      irRxArmed = false;
      irRxQuietSince = millis();
      return r;
    }
  }
  return IR_RX_NONE;
}

// ---------------------------------------------------------------- what was received

bool irIn(unsigned long v, unsigned long lo, unsigned long hi) { return v >= lo && v <= hi; }

// The 32 data bits of a NEC / Samsung frame (header at 0 and 1, then mark+space per bit, then the stop mark)
bool irWord32(const uint16_t* t, uint32_t* value) {
  uint32_t w = 0;
  for (int k = 0; k < 32; k++) {
    unsigned long m = t[2 + 2 * k], s = t[3 + 2 * k];
    if (!irIn(m, 300, 900)) return false;
    if (irIn(s, 300, 900)) {
      // a 0
    } else if (irIn(s, 1200, 2200)) {
      w |= (1UL << k);
    } else {
      return false;
    }
  }
  if (!irIn(t[66], 300, 900)) return false;             // the stop mark
  *value = w;
  return true;
}

// Which protocol is this frame? Sets 'value' and 'bits' for the ones that are recognised.
int irAnalyze(const uint16_t* t, int n, uint32_t* value, int* bits) {
  *value = 0;
  *bits = 0;
  if (n == 3 && irIn(t[0], 7500, 10500) && irIn(t[1], 1800, 2800) && irIn(t[2], 300, 900)) return IR_P_NECREP;
  if (n == 67 && irIn(t[1], 3800, 5200)) {
    uint32_t w;
    if (irIn(t[0], 7500, 10500) && irWord32(t, &w)) { *value = w; *bits = 32; return IR_P_NEC; }
    if (irIn(t[0], 3800, 5200) && irWord32(t, &w)) { *value = w; *bits = 32; return IR_P_SAMSUNG; }
  }
  // Sony SIRC: 12, 15 or 20 bits. The header mark is 2.4 ms; the lower limit is generous because a frame that starts
  // between two passes of the screen is noticed a little late, which shortens its first mark.
  if ((n == 25 || n == 31 || n == 41) && irIn(t[0], 1500, 2900)) {
    int nb = (n - 1) / 2;
    uint32_t w = 0;
    bool ok = true;
    for (int k = 0; k < nb && ok; k++) {
      unsigned long s = t[1 + 2 * k], m = t[2 + 2 * k];
      if (!irIn(s, 300, 900)) ok = false;
      else if (irIn(m, 300, 900)) { }
      else if (irIn(m, 1000, 1500)) w |= (1UL << k);
      else ok = false;
    }
    if (ok) { *value = w; *bits = nb; return IR_P_SONY; }
  }
  return IR_P_UNKNOWN;
}

const char* irProtoName(int p) {
  switch (p) {
    case IR_P_NEC:     return "NEC";
    case IR_P_NECREP:  return "NEC repeat";
    case IR_P_SAMSUNG: return "Samsung";
    case IR_P_SONY:    return "Sony SIRC";
    default:           return "Unknown";
  }
}

// Describe the frame in irRxBuf on three short lines (for the colour screen), e.g. "NEC 32 bit", "F708FB04", "Addr 04 Cmd 08"
char irRxLine1[20], irRxLine2[20], irRxLine3[20];
int irRxProto = IR_P_UNKNOWN;
uint32_t irRxValue = 0;
int irRxBits = 0;
void irRxDescribe() {
  irRxProto = irAnalyze(irRxBuf, irRxLen, &irRxValue, &irRxBits);
  irRxLine1[0] = irRxLine2[0] = irRxLine3[0] = '\0';
  if (irRxProto == IR_P_NEC || irRxProto == IR_P_SAMSUNG) {
    snprintf(irRxLine1, sizeof(irRxLine1), "%s 32 bit", irProtoName(irRxProto));
    snprintf(irRxLine2, sizeof(irRxLine2), "%08lX", (unsigned long)irRxValue);
    unsigned a = (unsigned)(irRxValue & 0xFF), c = (unsigned)((irRxValue >> 16) & 0xFF);
    snprintf(irRxLine3, sizeof(irRxLine3), "Addr %02X Cmd %02X", a, c);
  } else if (irRxProto == IR_P_SONY) {
    snprintf(irRxLine1, sizeof(irRxLine1), "Sony SIRC %d bit", irRxBits);
    snprintf(irRxLine2, sizeof(irRxLine2), "%05lX", (unsigned long)irRxValue);
    snprintf(irRxLine3, sizeof(irRxLine3), "Cmd %02lX Addr %lX", (unsigned long)(irRxValue & 0x7F), (unsigned long)(irRxValue >> 7));
  } else if (irRxProto == IR_P_NECREP) {
    snprintf(irRxLine1, sizeof(irRxLine1), "NEC repeat");
    snprintf(irRxLine2, sizeof(irRxLine2), "(button held)");
  } else {
    snprintf(irRxLine1, sizeof(irRxLine1), "Unknown protocol");
    snprintf(irRxLine2, sizeof(irRxLine2), "%d pulses", irRxLen);
    snprintf(irRxLine3, sizeof(irRxLine3), "%u %u", (unsigned)irRxBuf[0], (unsigned)(irRxLen > 1 ? irRxBuf[1] : 0));
  }
}

// One line on the Serial Monitor for every frame, with all the raw times (useful for checking a remote)
void irRxPrintSerial(const char* what) {
  Serial.print("IR RX: ");
  Serial.print(what);
  Serial.print(" | ");
  Serial.print(irProtoName(irRxProto));
  if (irRxBits > 0) {
    Serial.print(" ");
    Serial.print(irRxBits);
    Serial.print(" bit 0x");
    Serial.print((unsigned long)irRxValue, HEX);
  }
  Serial.print(" | ");
  Serial.print(irRxLen);
  Serial.print(" pulses:");
  for (int i = 0; i < irRxLen; i++) {
    Serial.print(" ");
    Serial.print((unsigned long)irRxBuf[i]);
  }
  Serial.println();
}

// ---------------------------------------------------------------- cloning

#define IR_NOTE_NONE        0         // what the note under the prompt says (also sent to the phone)
#define IR_NOTE_GOT         1         // "Got <button>"
#define IR_NOTE_SKIPPED     2         // "Skipped <button>"
#define IR_NOTE_REDO        3         // "Redo <button>"
#define IR_NOTE_REPEAT      4         // "Repeat ignored - retry"
#define IR_NOTE_TOOLONG     5         // "Too long - retry"

int irCloneIndex = 0;                 // which prompt (0 to IR_SLOTS - 1) is waiting; IR_SLOTS = finished
int irRxNoteCode = IR_NOTE_NONE;      // the kind of note, and the button it is about (the phone shows its own wording)
int irRxNoteSlot = 0;
int irRxDupSlot = 255;                // the earlier button that has the same code as the one just recorded (255 = none)
uint8_t irRxSeq = 0;                  // counts every frame received, so the phone can tell a new one from the last
char irRxNote[24] = "";               // the result of the last press, shown on the prompt
char irRxWarn[20] = "";               // e.g. "Same as Vol Up"
int irRxDoneKind = 0;                 // 1 cloned, 2 nothing recorded, 3 cancelled, 4 defaults restored
bool irRxSaved = false;               // did the save to flash work?
int irRxCaptured = 0;                 // codes recorded in the last clone

int irCloneCount() {
  int c = 0;
  for (int i = 0; i < IR_SLOTS; i++) if (irCloneSlots[i].kind != IR_KIND_NONE) c++;
  return c;
}

// Bit n set = button n of the clone being recorded has a code (sent to the phone to show the progress)
uint32_t irCloneRecordedMask() {
  uint32_t m = 0;
  for (int i = 0; i < IR_SLOTS; i++) if (irCloneSlots[i].kind != IR_KIND_NONE) m |= (1UL << i);
  return m;
}

// Is the clone's code 'a' the same as code 'b'? (a NEC word that matches, or recorded times within 20 %)
bool irCloneSame(int a, int b) {
  IrSlot& x = irCloneSlots[a];
  IrSlot& y = irCloneSlots[b];
  if (x.kind == IR_KIND_NONE || x.kind != y.kind) return false;
  if (x.kind == IR_KIND_NEC) return x.word == y.word;
  if (x.rawLen != y.rawLen) return false;
  for (int i = 0; i < x.rawLen; i++) {
    int d = (int)x.raw[i] - (int)y.raw[i];
    if (d < 0) d = -d;
    int m = (x.raw[i] > y.raw[i]) ? x.raw[i] : y.raw[i];
    if (d * 5 > m) return false;
  }
  return true;
}

// Turn the frame in irRxBuf into the clone's code for 'slot'. False if it can't be used (a NEC repeat frame).
bool irCloneStore(int slot) {
  if (irRxProto == IR_P_NECREP) return false;
  irSlotClear(1, slot);
  IrSlot& s = irCloneSlots[slot];
  if (irRxProto == IR_P_NEC) {
    s.kind = IR_KIND_NEC;
    s.word = irRxValue;
    return true;
  }
  s.kind = IR_KIND_RAW;
  s.rawLen = (uint8_t)irRxLen;
  for (int i = 0; i < irRxLen; i++) s.raw[i] = irRxBuf[i];
  if (irRxProto == IR_P_SONY) {                         // Sony devices want the frame three times, 45 ms apart, at 40 kHz
    s.khz = 40;
    s.frames = 3;
    s.gapMs = 45;
  }
  return true;
}

void irCloneStart() {
  for (int i = 0; i < IR_SLOTS; i++) irSlotClear(1, i);
  irCloneIndex = 0;
  irRxNote[0] = '\0';
  irRxWarn[0] = '\0';
  irRxNoteCode = IR_NOTE_NONE;
  irRxDupSlot = 255;
  irRxLine1[0] = irRxLine2[0] = irRxLine3[0] = '\0';
  irRxArmed = false;
  irRxQuietSince = millis();
}

// All the buttons have been asked: the recorded set replaces the codes in use (unless nothing at all was recorded)
void irCloneFinish() {
  irRxCaptured = irCloneCount();
  if (irRxCaptured == 0) {
    irRxDoneKind = 2;
  } else {
    memcpy(irSlots, irCloneSlots, sizeof(irSlots));
    irCodesCustom = true;
    irRxSaved = irCodesSave();
    irRxDoneKind = 1;
    Serial.print("IR clone: ");
    Serial.print(irRxCaptured);
    Serial.print(" codes recorded, ");
    Serial.println(irRxSaved ? "saved to flash" : "NOT saved to flash (lost at power-off)");
  }
}

// ---------------------------------------------------------------- the screen

int irRxMode = IR_RX_MENU;
bool irRxEntered = false;
bool irRxDirty = true;
int irRxMenuCursor = 0;

// First pass after the screen was opened
void irRxEnter() {
  irRxEntered = true;
  irCodesEnsure();
  pinMode(IR_RECV_PIN, INPUT);
  irRxMode = IR_RX_MENU;
  irRxMenuCursor = 0;
  irRxDirty = true;
  irRxArmed = false;
  irRxQuietSince = millis();
  irDataReceived = false;
}

void irRxStartClone() {
  irCloneStart();
  irRxMode = IR_RX_CLONE;
  irRxDirty = true;
}

void irRxOpenData() {
  irRxMode = IR_RX_DATA;
  irDataReceived = false;
  irRxArmed = false;
  irRxQuietSince = millis();
  irRxDirty = true;
}

// May the phone leave this screen (open another one) right now? Not while a clone or a question is on the screen:
// the script that walks the menus presses ENTER, which could answer "discard the new codes?" with yes.
bool irRxCanLeave() {
  return !irRxEntered || irRxMode == IR_RX_MENU || irRxMode == IR_RX_DATA;
}

// The phone's commands for this screen: 1 = start a clone, 2 = open the data view, 3 = back to the menu (from the
// data view or a result). Everything else the phone does here is an ordinary button press. False if not possible now.
bool irRxRemoteAction(int action) {
  if (!irRxEntered) irRxEnter();
  if (action == 1 && irRxMode == IR_RX_MENU) { irRxStartClone(); return true; }
  if (action == 2 && irRxMode == IR_RX_MENU) { irRxOpenData(); return true; }
  if (action == 3 && (irRxMode == IR_RX_DATA || irRxMode == IR_RX_DONE)) { irRxMode = IR_RX_MENU; irRxDirty = true; return true; }
  return false;
}

// The IR Receive packet for the phone (the layout is documented at blePackIrRx() in Bluetooth.ino)
void irRxPacket(uint8_t* out) {
  blePackIrRx(out, (uint8_t)irRxMode, currentState == IRRECEIVER, irCodesCustom, irRxSaved, IR_HAVE_KV != 0, irRxSeq != 0,
              (uint8_t)irCloneIndex, (uint8_t)irRxDoneKind, (uint8_t)irRxNoteCode, (uint8_t)irRxNoteSlot, (uint8_t)irRxDupSlot,
              (uint8_t)(irRxSeq != 0 ? irRxProto : 0xFF), (uint8_t)irRxBits, irRxValue, (uint16_t)irRxLen, irRxSeq,
              irCloneRecordedMask());
}

void irRxOledStart() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

// ---- the colour screen (LCARS look, see Lcars.ino) ----
// Every IR Receive screen has the frame (IR RECEIVE), a status block under it with the screen's own title (green, amber or
// blue like the title was), and its text below: lines of at most 14 characters, 8 pixels apart from y = 22.
#define IRRX_TFT_CHARS 14

void irRxTftTitle(const char* title, uint16_t color) {
  lcFrame("IR RECEIVE", false);
  lcBigPill(8, 11, 88, 9, color == GREEN ? lcOk() : (color == YELLOW ? lcGold() : lcBlue()), title, 1);
}

// One line of text at height y, cut to what fits
void irRxTftLine(int y, const char* text, uint16_t color) {
  char buf[IRRX_TFT_CHARS + 1];
  strncpy(buf, text, IRRX_TFT_CHARS);
  buf[IRRX_TFT_CHARS] = '\0';
  tft.setTextSize(1);
  tft.setTextColor(color, BLACK);
  tft.setCursor(8, y);
  tft.print(buf);
}

// Replace the first 'from' in 's' by the (shorter) 'to'
void irRxReplace(char* s, const char* from, const char* to) {
  char* at = strstr(s, from);
  if (at == NULL) return;
  size_t fl = strlen(from), tl = strlen(to);
  memmove(at + tl, at + fl, strlen(at + fl) + 1);
  memcpy(at, to, tl);
}

// A line from the decoder (up to 19 characters) in the 14 the colour screen has: the longest phrases are shortened first
void irRxTftShort(char* dst, const char* src) {
  char tmp[24];
  strncpy(tmp, src, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = '\0';
  if (strlen(tmp) > IRRX_TFT_CHARS) irRxReplace(tmp, "Sony SIRC", "SIRC");
  if (strlen(tmp) > IRRX_TFT_CHARS) irRxReplace(tmp, "Unknown protocol", "Unknown");
  if (strlen(tmp) > IRRX_TFT_CHARS) irRxReplace(tmp, " Addr ", " A:");
  if (strlen(tmp) > IRRX_TFT_CHARS) irRxReplace(tmp, "Same as ", "= ");
  strncpy(dst, tmp, IRRX_TFT_CHARS);
  dst[IRRX_TFT_CHARS] = '\0';
}

// A choice in the menu: lit in orange when selected, dimmed when not
void irRxTftItem(int y, const char* text, bool selected) {
  if (selected) {
    lcBigPill(8, y, 88, 8, lcOrange(), text, 1);
    return;
  }
  tft.fillRoundRect(8, y, 88, 8, 3, lcDim(lcOrange()));
  tft.setTextSize(1);
  tft.setTextColor(lcGrey(), lcDim(lcOrange()));
  tft.setCursor(8 + (88 - 6 * (int)strlen(text)) / 2, y);
  tft.print(text);
}

// The two keys of a question
void irRxTftKeys() {
  lcBigPill(8, 47, 88, 8, lcAlert(), "ENTER = YES", 1);
  lcBigPill(8, 56, 88, 8, lcBlue(), "LEFT = NO", 1);
}

void irRxDrawMenu() {
  irRxOledStart();
  display.setCursor(0, 0);
  display.println(F("IR RECEIVE"));
  display.setCursor(0, 16);
  display.print(irRxMenuCursor == 0 ? "> " : "  ");
  display.println(F("1. Clone remote"));
  display.setCursor(0, 28);
  display.print(irRxMenuCursor == 1 ? "> " : "  ");
  display.println(F("2. Receive IR data"));
  display.setCursor(0, 52);
  display.print(F("Codes in use: "));
  display.println(irCodeSetName());
  display.display();
  irRxTftTitle("CHOOSE", CYAN);
  irRxTftItem(21, "CLONE REMOTE", irRxMenuCursor == 0);
  irRxTftItem(30, "RECEIVE DATA", irRxMenuCursor == 1);
  irRxTftLine(40, "HOLD LT: EXIT", lcGrey());
  irRxTftLine(48, "HOLD DN FOR", lcGold());
  irRxTftLine(56, "DEFAULT CODES", lcGold());
}

void irRxDrawData(bool got) {
  irRxOledStart();
  display.setCursor(0, 0);
  display.println(F("IR DATA   (LT = back)"));
  if (!got) {
    display.setCursor(0, 20);
    display.println(F("Point a remote at"));
    display.println(F("the receiver and"));
    display.println(F("press a button..."));
  } else {
    display.setCursor(0, 14);
    display.println(irRxLine1);
    display.println(irRxLine2);
    display.println(irRxLine3);
    char buf[24];
    snprintf(buf, sizeof(buf), "%d pulses", irRxLen);
    display.println(buf);
  }
  display.display();
  irRxTftTitle(got ? "SIGNAL FOUND" : "IR DATA", got ? GREEN : CYAN);
  if (!got) {
    irRxTftLine(22, "WAITING FOR", lcPeach());
    irRxTftLine(30, "A SIGNAL...", lcPeach());
    irRxTftLine(56, "LT BACK", lcGold());
  } else {
    char b[24];
    irRxTftShort(b, irRxLine1);
    irRxTftLine(22, b, lcPeach());
    irRxTftShort(b, irRxLine2);
    irRxTftLine(30, b, lcPeach());
    irRxTftShort(b, irRxLine3);
    irRxTftLine(38, b, lcPeach());
    snprintf(b, sizeof(b), "%d PULSES", irRxLen);
    irRxTftLine(56, b, lcGold());
  }
}

void irRxDrawClone() {
  int slot = irCloneOrder[irCloneIndex];
  irRxOledStart();
  char head[24];
  snprintf(head, sizeof(head), "CLONE REMOTE  %d/%d", irCloneIndex + 1, IR_SLOTS);
  display.setCursor(0, 0);
  display.println(head);
  display.setCursor(0, 11);
  display.println(F("Press on the remote:"));
  display.setTextSize(2);
  display.setCursor(0, 22);
  display.println(irSlotNames[slot]);
  display.setTextSize(1);
  display.setCursor(0, 42);
  display.println(irRxNote[0] ? irRxNote : "Waiting for IR...");
  display.setCursor(0, 55);
  display.println(F("R:skip U:back L:quit"));
  display.display();
  char title[16];
  snprintf(title, sizeof(title), "CLONE %d/%d", irCloneIndex + 1, IR_SLOTS);
  irRxTftTitle(title, CYAN);
  char b[24];
  if (irRxLine1[0]) {
    irRxTftShort(b, irRxLine1);
    irRxTftLine(22, b, lcOk());
    irRxTftShort(b, irRxLine2);
    irRxTftLine(30, b, lcPeach());
    irRxTftShort(b, irRxLine3);
    irRxTftLine(38, b, lcPeach());
  } else {
    irRxTftLine(22, "WAITING FOR", lcPeach());
    irRxTftLine(30, "THE REMOTE...", lcPeach());
  }
  if (irRxWarn[0]) {
    irRxTftShort(b, irRxWarn);
    irRxTftLine(46, b, lcAlert());
  }
  int set = irCloneCount();
  snprintf(b, sizeof(b), "SET %d SKIP %d", set, irCloneIndex - set);
  irRxTftLine(56, b, lcGold());
}

void irRxDrawConfirm(bool restore) {
  irRxOledStart();
  display.setCursor(0, 8);
  display.println(restore ? F("Go back to the default") : F("Discard the new codes?"));
  if (restore) display.println(F("codes?"));
  display.setCursor(0, 36);
  display.println(F("ENTER = yes"));
  display.println(F("LEFT  = no"));
  display.display();
  irRxTftTitle(restore ? "DEFAULT CODES" : "CANCEL CLONE", YELLOW);
  lcWrap(8, 22, IRRX_TFT_CHARS, 3, restore ? "USE THE BUILT-IN CODES AGAIN?" : "THROW AWAY THE CODES JUST RECORDED?", lcPeach());
  irRxTftKeys();
}

void irRxDrawDone() {
  irRxOledStart();
  display.setCursor(0, 0);
  char buf[24];
  if (irRxDoneKind == 1) {
    display.println(F("CLONE COMPLETE"));
    snprintf(buf, sizeof(buf), "%d set, %d skipped", irRxCaptured, IR_SLOTS - irRxCaptured);
    display.setCursor(0, 14);
    display.println(buf);
    display.setCursor(0, 28);
    display.println(irRxSaved ? F("Saved to flash") : F("NOT saved: lost at"));
    if (!irRxSaved) display.println(F("power-off"));
  } else if (irRxDoneKind == 2) {
    display.println(F("NOTHING RECORDED"));
    display.setCursor(0, 14);
    display.println(F("Every button was"));
    display.println(F("skipped. The codes"));
    display.println(F("are unchanged."));
  } else if (irRxDoneKind == 3) {
    display.println(F("CLONE CANCELLED"));
    display.setCursor(0, 14);
    display.println(F("The new codes were"));
    display.println(F("discarded. The codes"));
    display.println(F("in use are unchanged."));
  } else {
    display.println(F("DEFAULT CODES"));
    display.setCursor(0, 14);
    display.println(F("The built-in codes"));
    display.println(F("are in use again."));
    if (!irRxSaved) display.println(F("(the saved set is still in flash)"));
  }
  display.setCursor(0, 56);
  display.println(F("Press any button"));
  display.display();
  irRxTftTitle(irRxDoneKind == 1 ? "CLONE COMPLETE" : (irRxDoneKind == 3 ? "CANCELLED" : (irRxDoneKind == 4 ? "DEFAULTS" : "NO CODES")), irRxDoneKind == 1 ? GREEN : YELLOW);
  if (irRxDoneKind == 1) {
    snprintf(buf, sizeof(buf), "%d SET", irRxCaptured);
    irRxTftLine(22, buf, lcPeach());
    snprintf(buf, sizeof(buf), "%d SKIPPED", IR_SLOTS - irRxCaptured);
    irRxTftLine(30, buf, lcPeach());
    irRxTftLine(38, irRxSaved ? "SAVED" : "NOT SAVED", irRxSaved ? lcOk() : lcAlert());
    if (!irRxSaved) lcWrap(8, 46, IRRX_TFT_CHARS, 2, "LOST AT POWER-OFF", lcAlert());
  } else if (irRxDoneKind == 4) {
    irRxTftLine(22, "BUILT-IN CODES", lcPeach());
    irRxTftLine(30, "IN USE", lcPeach());
    if (!irRxSaved) lcWrap(8, 46, IRRX_TFT_CHARS, 2, "FLASH NOT CLEARED", lcGold());
  } else {
    lcWrap(8, 22, IRRX_TFT_CHARS, 2, "CODES UNCHANGED", lcPeach());
  }
}

// ---- IR Receive > 2. Receive IR data
void irRxRunData(int pressType) {
  if (pressType == SHORT_PRESS && lastButton == 5) {            // back to the menu
    irRxMode = IR_RX_MENU;
    irRxDirty = true;
    return;
  }
  if (irRxDirty) {
    irRxDrawData(irDataReceived);
    irRxDirty = false;
  }
  int r = irRxPoll();
  if (r == IR_RX_OK) {
    irRxSeq++;
    irRxDescribe();
    irRxPrintSerial("received");
    irDataReceived = true;
    storedIrData = irRxValue;
    storedIrAddr = (uint8_t)(irRxValue & 0xFF);
    storedIrCmd = (uint8_t)((irRxValue >> 16) & 0xFF);
    irRxDrawData(true);
  } else if (r == IR_RX_TOOLONG) {
    Serial.println("IR RX: a frame with more than 120 marks and spaces was ignored");
  }
}

// ---- IR Receive > 1. Clone remote
void irRxRunClone(int pressType) {
  if (pressType == SHORT_PRESS) {
    if (lastButton == 5) {                                      // LEFT: cancel (asks first)
      irRxMode = IR_RX_CONFIRM;
      irRxDirty = true;
      return;
    }
    if (lastButton == 3) {                                      // RIGHT: skip this button
      int slot = irCloneOrder[irCloneIndex];
      irSlotClear(1, slot);
      snprintf(irRxNote, sizeof(irRxNote), "Skipped %s", irSlotNames[slot]);
      irRxNoteCode = IR_NOTE_SKIPPED;
      irRxNoteSlot = slot;
      irRxWarn[0] = '\0';
      irRxDupSlot = 255;
      irCloneIndex++;
      irRxDirty = true;
    } else if (lastButton == 2 && irCloneIndex > 0) {           // UP: back one, to do it again
      irCloneIndex--;
      int slot = irCloneOrder[irCloneIndex];
      irSlotClear(1, slot);
      snprintf(irRxNote, sizeof(irRxNote), "Redo %s", irSlotNames[slot]);
      irRxNoteCode = IR_NOTE_REDO;
      irRxNoteSlot = slot;
      irRxWarn[0] = '\0';
      irRxDupSlot = 255;
      irRxLine1[0] = irRxLine2[0] = irRxLine3[0] = '\0';
      irRxArmed = false;
      irRxQuietSince = millis();
      irRxDirty = true;
    }
    if (irCloneIndex >= IR_SLOTS) {
      irCloneFinish();
      irRxMode = IR_RX_DONE;
      irRxDirty = true;
      return;
    }
  }
  if (irRxDirty) {
    irRxDrawClone();
    irRxDirty = false;
  }
  int r = irRxPoll();
  if (r == IR_RX_OK) {
    irRxSeq++;
    irRxDescribe();
    int slot = irCloneOrder[irCloneIndex];
    if (!irCloneStore(slot)) {                                  // a NEC repeat frame: the button is still held
      snprintf(irRxNote, sizeof(irRxNote), "Repeat ignored - retry");
      irRxNoteCode = IR_NOTE_REPEAT;
      irRxPrintSerial("ignored");
    } else {
      irRxWarn[0] = '\0';
      irRxDupSlot = 255;
      for (int i = 0; i < IR_SLOTS; i++) {                      // the same code as a button recorded earlier?
        if (i != slot && irCloneSlots[i].kind != IR_KIND_NONE && irCloneSame(slot, i)) {
          snprintf(irRxWarn, sizeof(irRxWarn), "Same as %s", irSlotNames[i]);
          irRxDupSlot = i;
          break;
        }
      }
      char what[24];
      snprintf(what, sizeof(what), "clone %s", irSlotNames[slot]);
      irRxPrintSerial(what);
      snprintf(irRxNote, sizeof(irRxNote), "Got %s", irSlotNames[slot]);
      irRxNoteCode = IR_NOTE_GOT;
      irRxNoteSlot = slot;
      irCloneIndex++;
      if (irCloneIndex >= IR_SLOTS) {
        irCloneFinish();
        irRxMode = IR_RX_DONE;
      }
    }
    irRxDirty = true;
  } else if (r == IR_RX_TOOLONG) {
    snprintf(irRxNote, sizeof(irRxNote), "Too long - retry");
    irRxNoteCode = IR_NOTE_TOOLONG;
    irRxDirty = true;
  }
}

// ---- the IR Receive screen
void irReceiver(int pressType) {
  if (!irRxEntered) irRxEnter();                                // first pass after the screen was opened

  // Hold LEFT: leave the screen (while cloning or answering a question it is the "no / cancel" instead)
  if (pressType == LONG_PRESS && lastButton == 5) {
    if (irRxMode == IR_RX_CLONE) {
      irRxMode = IR_RX_CONFIRM;
      irRxDirty = true;
    } else if (irRxMode == IR_RX_CONFIRM) {
      irRxMode = IR_RX_CLONE;
      irRxDirty = true;
    } else if (irRxMode == IR_RX_RESTORE || irRxMode == IR_RX_DONE) {
      irRxMode = IR_RX_MENU;
      irRxDirty = true;
    } else {
      tft.fillScreen(BLACK);
      display.clearDisplay();
      display.display();
      irDataReceived = false;
      irRxEntered = false;
      currentState = COMMS_MENU;
    }
    return;
  }

  switch (irRxMode) {
    case IR_RX_MENU:
      if (pressType == LONG_PRESS && lastButton == 4) {         // hold DOWN: back to the default codes
        irRxMode = IR_RX_RESTORE;
        irRxDirty = true;
        break;
      }
      if (pressType == SHORT_PRESS) {
        if (lastButton == 2 || lastButton == 4) {
          irRxMenuCursor = 1 - irRxMenuCursor;
          irRxDirty = true;
        } else if (lastButton == 1) {
          if (irRxMenuCursor == 0) irRxStartClone();
          else irRxOpenData();
          break;
        }
      }
      if (irRxDirty) {
        irRxDrawMenu();
        irRxDirty = false;
      }
      delay(5);
      break;

    case IR_RX_DATA:
      irRxRunData(pressType);
      break;

    case IR_RX_CLONE:
      irRxRunClone(pressType);
      break;

    case IR_RX_CONFIRM:                                         // "Discard the new codes?"
      if (pressType == SHORT_PRESS && lastButton == 1) {
        irRxDoneKind = 3;
        irRxMode = IR_RX_DONE;
        irRxDirty = true;
        Serial.println("IR clone: cancelled, the new codes were discarded");
      } else if (pressType == SHORT_PRESS && lastButton == 5) {
        irRxMode = IR_RX_CLONE;
        irRxDirty = true;
      }
      if (irRxDirty && irRxMode == IR_RX_CONFIRM) {
        irRxDrawConfirm(false);
        irRxDirty = false;
      }
      delay(5);
      break;

    case IR_RX_RESTORE:                                         // "Go back to the default codes?"
      if (pressType == SHORT_PRESS && lastButton == 1) {
        irRxSaved = irCodesRestoreDefaults();
        irRxDoneKind = 4;
        irRxMode = IR_RX_DONE;
        irRxDirty = true;
        Serial.println("IR codes: the default set is in use again");
      } else if (pressType == SHORT_PRESS && lastButton == 5) {
        irRxMode = IR_RX_MENU;
        irRxDirty = true;
      }
      if (irRxDirty && irRxMode == IR_RX_RESTORE) {
        irRxDrawConfirm(true);
        irRxDirty = false;
      }
      delay(5);
      break;

    case IR_RX_DONE:
      if (pressType == SHORT_PRESS) {
        irRxMode = IR_RX_MENU;
        irRxDirty = true;
        break;
      }
      if (irRxDirty) {
        irRxDrawDone();
        irRxDirty = false;
      }
      delay(5);
      break;
  }
}
