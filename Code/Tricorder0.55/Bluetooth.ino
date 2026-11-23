// ------------- Bluetooth (BLE) ---------------
//
// Sends sensor data to a phone over Bluetooth Low Energy. The Tricorder is the "peripheral" (it advertises as
// "Tricorder"), the phone connects to it and receives notifications.
//
// Protocol v5 - eleven read+notify characteristics and one write characteristic (Command) in one custom service (full details and byte layouts are in
// Tricorder_BLE_Protocol.md):
//   Environment  temperature, humidity, pressure, device clock                  once a second
//   Position     GPS latitude / longitude / altitude, speed, course, satellites, HDOP, fix age   once a second
//   Device       battery, active screen, uptime, free memory, protocol and firmware version     once a second
// and, for the Tricorder screens below, 5 per second while that screen is open ("live"), every 2 s otherwise:
//   Motion       compass heading, pitch, roll, field strength            (Navigation)
//   Sound        dB level, max, average (Leq), peak                      (dB Meter)
//   Spectrum     16 band levels, strongest frequency                     (Spectrum)
//   Fields       metal detector and EMF meter readings                   (Metal Detector, EMF Meter)
//   Shot         shot timer mode, countdown, running time, settings      (Shot Timer)
//   List         the shot times, or the stopwatch laps, 4 at a time      (Shot Timer, Stopwatch)
//   Stopwatch    elapsed time, state, lap count                          (Stopwatch)
//   Countdown    remaining time, set time, state                         (Countdown Timer)
//
// and one WRITE characteristic, Command: the phone can press the Tricorder's buttons, open its screens and send
// one of the IR Send screen's twelve codes (see Remote.ino). That is OFF until you switch it on at the Tricorder (this screen, RIGHT).
//
// Bluetooth is OFF until you turn it on (COMMS menu > Bluetooth > ENTER). It stays on when you leave
// the screen, so you can keep using the other tools while the phone receives data. It is off after every
// power-up on purpose: while it is on, anyone nearby with a BLE app can connect and read your GPS position.
//
// bleUpdate() must be called from loop() on every pass (it services the Bluetooth stack and sends the data).
//
// Needs the ArduinoBLE library (Library Manager) and   #include <ArduinoBLE.h>   at the top of the main sketch.

#include <ArduinoBLE.h>

#define BLE_PROTOCOL_VERSION 5
#define BLE_DEVICE_NAME      "Tricorder"
#define BLE_NOTIFY_PERIOD_MS 1000

#define BLE_SERVICE_UUID  "3f5a0000-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_ENV_UUID      "3f5a0001-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_POS_UUID      "3f5a0002-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_DEV_UUID      "3f5a0003-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_MOT_UUID      "3f5a0004-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_SND_UUID      "3f5a0005-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_SPC_UUID      "3f5a0006-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_FLD_UUID      "3f5a0007-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_SHT_UUID      "3f5a0008-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_LST_UUID      "3f5a0009-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_SWT_UUID      "3f5a000a-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_CDT_UUID      "3f5a000b-7c2b-4e6d-a1b3-5d8e9c0f1a20"
#define BLE_CMD_UUID      "3f5a000c-7c2b-4e6d-a1b3-5d8e9c0f1a20"

// Pressure correction. The Atmospheric screen converts to inHg with 0.02994 instead of the true 0.02953: a
// deliberate correction of the pressure sensor against a reference instrument (about +1.4 %). The same
// correction is applied to the pressure sent over Bluetooth, so the phone app (which converts with the true
// factor) agrees with the Tricorder's own screen. If you recalibrate the sensor, change both places.
#define BLE_PRESSURE_CAL (0.02994f / 0.02953f)

#define BLE_ENV_LEN 12
#define BLE_POS_LEN 20
#define BLE_DEV_LEN 16
#define BLE_MOT_LEN 10
#define BLE_SND_LEN 10
#define BLE_SPC_LEN 20
#define BLE_FLD_LEN 12
#define BLE_SHT_LEN 10
#define BLE_LST_LEN 19
#define BLE_SWT_LEN 8
#define BLE_CDT_LEN 8
#define BLE_CMD_LEN 4                // a command is 4 bytes
#define BLE_STREAMS 11               // number of characteristics that are sent
#define BLE_LIVE_PERIOD_MS   200      // Motion / Sound / Spectrum while their screen is open
#define BLE_IDLE_PERIOD_MS   2000     // ... and otherwise

// ---- Communications menu (Bluetooth and the two IR tools; add future tools to this list) ----
#define COMMS_MENU_COUNT 3
const char* commsMenuOptions[COMMS_MENU_COUNT] = {"Bluetooth", "IR TX", "IR RX"};

// ---- values published by the other screens, sent over Bluetooth ----
// Navigation (Nagivation.ino) fills in the compass values and Spectrum (Spectrum.ino) the strongest-bin values
// while their screens are open. The dB Meter's own variables are read directly.
float telHeading = 0.0;           // degrees, as shown on the Navigation screen (declination included)
float telPitch = 0.0;             // degrees
float telRoll = 0.0;              // degrees
float telField = 0.0;             // calibrated magnetic field strength, uT
bool  telLevel = false;           // the device is held level
unsigned long telMotionMs = 0;    // millis() when the compass values were last updated (0 = never)
float telSpecPeakHz = 0.0;        // frequency of the strongest spectrum bin
float telSpecPeakPow = 0.0;       // its power (same units as specBandPow)
unsigned long telSpecMs = 0;      // millis() when the spectrum was last updated (0 = never)
float telMetalMag = 0.0;          // Metal Detector screen: field magnitude, uT
float telMetalDev = 0.0;          // ... deviation from the baseline, uT
bool  telMetalDetected = false;
unsigned long telMetalMs = 0;     // millis() when the Metal Detector last updated (0 = never)
float telEmfMag = 0.0;            // EMF Meter screen: field magnitude, uT
unsigned long telEmfMs = 0;       // millis() when the EMF Meter last updated (0 = never)

// Variables that live in files compiled after this one
extern float dbLevel;
extern float dbMaxLevel;
extern float dbLeqDb;
extern float dbPeakDb;
extern float dbCalOffset;
extern bool  dbWeightA;
extern bool  dbSlow;
extern float specBandPow[16];     // must match SPEC_BANDS in Spectrum.ino
extern bool  remoteEnabled;       // Remote.ino
extern int  shotMode;             // Shot Timer state (ShotTimer.ino): 0 setup, 1 countdown, 2 timing, 3 results
extern int  shotDelaySec;
extern int  shotSens;
extern unsigned long shotCountdownEndMs;
extern uint32_t shotStartUs;
extern bool shotArmed;
extern uint32_t shotTimes[99];    // must match SHOT_MAX in ShotTimer.ino
extern int  shotCount;
extern bool shotOverflow;
extern bool swRunning;            // Stopwatch (Stopwatch.ino)
extern unsigned long swLapTotals[20];   // must match SW_MAX_LAPS in Stopwatch.ino
extern int  swLapCount;
extern int  cdMode;               // Countdown Timer (CountdownTimer.ino): 0 set, 1 running, 2 paused, 3 time up
extern int  cdSetMin;
extern int  cdSetSecs;
extern unsigned long cdEndMs;
extern unsigned long cdRemainMs;

// ======================= packet packing (no Bluetooth or display code in this block) =======================
// ---- PACK BEGIN ----
// All multi-byte values are little-endian.
void blePut16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

void blePut32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

// NaN test that doesn't depend on which math headers the core provides (a NaN never equals itself)
bool bleIsNan(float v) {
  return v != v;
}

// Round a double to a signed integer, limited to [lo, hi]
long bleRoundClamp(double v, long lo, long hi) {
  if (v != v) return lo;                            // NaN
  double r = (v >= 0.0) ? floor(v + 0.5) : -floor(-v + 0.5);
  if (r < (double)lo) return lo;
  if (r > (double)hi) return hi;
  return (long)r;
}

// Environment (12 bytes)
//   0  int16   temperature, 0.01 degC         (0x8000 = no reading)
//   2  uint16  relative humidity, 0.01 %      (0xFFFF = no reading)
//   4  uint32  pressure, Pa                   (0 = no reading)
//   8  uint32  device uptime, ms
void blePackEnvironment(uint8_t* out, float tempC, float humidityPct, float pressureHPa, uint32_t uptimeMs) {
  int16_t t = (int16_t)0x8000;
  if (!bleIsNan(tempC)) t = (int16_t)bleRoundClamp((double)tempC * 100.0, -32767, 32767);
  uint16_t h = 0xFFFF;
  if (!bleIsNan(humidityPct)) h = (uint16_t)bleRoundClamp((double)humidityPct * 100.0, 0, 65534);
  uint32_t p = 0;
  if (!bleIsNan(pressureHPa)) p = (uint32_t)bleRoundClamp((double)pressureHPa * 100.0, 0, 2000000000L);
  blePut16(out + 0, (uint16_t)t);
  blePut16(out + 2, h);
  blePut32(out + 4, p);
  blePut32(out + 8, uptimeMs);
}

// Position (20 bytes)
//   0  int32   latitude, 1e-7 degrees
//   4  int32   longitude, 1e-7 degrees
//   8  int32   altitude above sea level, cm
//  12  uint16  speed, cm/s                    (0xFFFF = invalid)
//  14  uint16  course, 0.1 degrees 0-3599     (0xFFFF = invalid)
//  16  uint8   satellites in use              (0xFF = unknown)
//  17  uint8   HDOP, 0.1                      (0xFF = unknown)
//  18  uint8   age of the position fix, s     (0xFF = never had one, 0xFE = 254 s or more)
//  19  uint8   flags: bit0 position valid, bit1 altitude valid, bit2 speed valid, bit3 course valid
void blePackPosition(uint8_t* out, bool locValid, double lat, double lng, bool altValid, double altM,
                     bool spdValid, double mps, bool crsValid, double crsDeg, int sats,
                     bool hdopValid, double hdop, bool everFixed, uint32_t fixAgeMs) {
  int32_t la = 0, lo = 0, al = 0;
  if (locValid) {
    la = (int32_t)bleRoundClamp(lat * 1e7, -900000000L, 900000000L);
    lo = (int32_t)bleRoundClamp(lng * 1e7, -1800000000L, 1800000000L);
  }
  if (altValid) al = (int32_t)bleRoundClamp(altM * 100.0, -100000000L, 100000000L);
  uint16_t sp = 0xFFFF;
  if (spdValid) sp = (uint16_t)bleRoundClamp(mps * 100.0, 0, 65534);
  uint16_t cr = 0xFFFF;
  if (crsValid) {
    long c = bleRoundClamp(crsDeg * 10.0, 0, 3600);
    if (c >= 3600) c = 0;                           // 360.0 deg is 0.0 deg
    cr = (uint16_t)c;
  }
  uint8_t sat = 0xFF;
  if (sats >= 0) sat = (uint8_t)((sats > 254) ? 254 : sats);
  uint8_t hd = 0xFF;
  if (hdopValid) hd = (uint8_t)bleRoundClamp(hdop * 10.0, 0, 254);
  uint8_t age = 0xFF;
  if (everFixed) {
    uint32_t s = fixAgeMs / 1000UL;
    age = (uint8_t)((s > 254UL) ? 254 : s);
  }
  uint8_t flags = 0;
  if (locValid) flags |= 0x01;
  if (altValid) flags |= 0x02;
  if (spdValid) flags |= 0x04;
  if (crsValid) flags |= 0x08;

  blePut32(out + 0, (uint32_t)la);
  blePut32(out + 4, (uint32_t)lo);
  blePut32(out + 8, (uint32_t)al);
  blePut16(out + 12, sp);
  blePut16(out + 14, cr);
  out[16] = sat;
  out[17] = hd;
  out[18] = age;
  out[19] = flags;
}

// Device (16 bytes)
//   0  uint16  battery voltage, mV
//   2  uint8   battery percent 0-100
//   3  uint8   active screen id (see the protocol document; 0 = a menu, 255 = other)
//   4  uint32  device uptime, ms
//   8  uint32  largest free memory block, bytes
//  12  uint8   protocol version
//  13  uint8   flags: bit0 remote control is switched on, bit1 the phone's remote script is still running,
//                     bit2 remote control is on but loop() never calls remoteApply() (the loop() line is missing)
//  14  uint16  firmware version x 1000, e.g. 0.55 is sent as 550   (0xFFFF = unknown)
void blePackDevice(uint8_t* out, int batteryMv, int batteryPct, uint8_t screenId, uint32_t uptimeMs,
                   uint32_t freeBytes, float firmware, uint8_t flags) {
  blePut16(out + 0, (uint16_t)bleRoundClamp((double)batteryMv, 0, 65535));
  out[2] = (uint8_t)bleRoundClamp((double)batteryPct, 0, 100);
  out[3] = screenId;
  blePut32(out + 4, uptimeMs);
  blePut32(out + 8, freeBytes);
  out[12] = BLE_PROTOCOL_VERSION;
  out[13] = flags;
  uint16_t fw = 0xFFFF;
  if (!bleIsNan(firmware) && firmware >= 0.0f) fw = (uint16_t)bleRoundClamp((double)firmware * 1000.0, 0, 65534);
  blePut16(out + 14, fw);
}

// Motion (10 bytes)
//   0  uint16  compass heading, 0.1 degrees 0-3599     (0xFFFF = no reading)
//   2  int16   pitch, 0.1 degrees                      (0x8000 = no reading)
//   4  int16   roll, 0.1 degrees                       (0x8000 = no reading)
//   6  uint16  magnetic field strength, 0.1 uT         (0xFFFF = no reading)
//   8  uint8   flags: bit0 held level, bit1 values valid, bit2 magnetometer calibrated,
//                     bit3 live (the Navigation screen is open and updating)
//   9  uint8   reserved (0)
void blePackMotion(uint8_t* out, bool valid, float heading, float pitch, float roll, float field,
                   bool level, bool calibrated, bool live) {
  uint16_t h = 0xFFFF;
  int16_t p = (int16_t)0x8000;
  int16_t r = (int16_t)0x8000;
  uint16_t f = 0xFFFF;
  if (valid && !bleIsNan(heading)) {
    long c = bleRoundClamp((double)heading * 10.0, 0, 3600);
    if (c >= 3600) c = 0;                           // 360.0 degrees is 0.0 degrees
    h = (uint16_t)c;
  }
  if (valid && !bleIsNan(pitch)) p = (int16_t)bleRoundClamp((double)pitch * 10.0, -32767, 32767);
  if (valid && !bleIsNan(roll)) r = (int16_t)bleRoundClamp((double)roll * 10.0, -32767, 32767);
  if (valid && !bleIsNan(field)) f = (uint16_t)bleRoundClamp((double)field * 10.0, 0, 65534);
  uint8_t flags = 0;
  if (level) flags |= 0x01;
  if (valid) flags |= 0x02;
  if (calibrated) flags |= 0x04;
  if (live) flags |= 0x08;
  blePut16(out + 0, h);
  blePut16(out + 2, (uint16_t)p);
  blePut16(out + 4, (uint16_t)r);
  blePut16(out + 6, f);
  out[8] = flags;
  out[9] = 0;
}

uint16_t bleDbTenths(float db) {
  return (uint16_t)bleRoundClamp((double)db * 10.0, 0, 65534);
}

// Sound (10 bytes), all levels in dB SPL
//   0  uint16  current level, 0.1 dB       (time-weighted: Fast or Slow)
//   2  uint16  highest level since reset, 0.1 dB
//   4  uint16  average level since reset (Leq), 0.1 dB
//   6  uint16  loudest single sample since reset, 0.1 dB
//   8  uint8   flags: bit0 A-weighted (otherwise flat), bit1 Slow (otherwise Fast),
//                     bit2 live (the dB Meter screen is open), bit3 values valid
//   9  uint8   reserved (0)
void blePackSound(uint8_t* out, bool valid, float level, float maxLevel, float leq, float peak,
                  bool aWeighted, bool slow, bool live) {
  blePut16(out + 0, valid ? bleDbTenths(level) : 0);
  blePut16(out + 2, valid ? bleDbTenths(maxLevel) : 0);
  blePut16(out + 4, valid ? bleDbTenths(leq) : 0);
  blePut16(out + 6, valid ? bleDbTenths(peak) : 0);
  uint8_t flags = 0;
  if (aWeighted) flags |= 0x01;
  if (slow) flags |= 0x02;
  if (live) flags |= 0x04;
  if (valid) flags |= 0x08;
  out[8] = flags;
  out[9] = 0;
}

// dB SPL (flat) of a linear mean-square power, as a whole number 0-254
uint8_t bleLevelByte(double power, double calOffsetDb) {
  if (power != power || power < 0.0) return 0;
  return (uint8_t)bleRoundClamp(10.0 * log10(power + 1e-20) + calOffsetDb, 0, 254);
}

// Spectrum (20 bytes)
//   0-15  uint8   level of each of the 16 bands, dB SPL, 1 dB steps  (0 = nothing / no data)
//  16     uint16  frequency of the strongest bin, Hz
//  18     uint8   level of the strongest bin, dB SPL
//  19     uint8   flags: bit0 live (the Spectrum screen is open and updating), bit1 values valid
// Band edges are in the protocol document.
void blePackSpectrum(uint8_t* out, const float* bandPower, int bands, float calOffsetDb,
                     float peakHz, float peakPower, bool valid, bool live) {
  for (int i = 0; i < 16; i++) out[i] = 0;
  if (valid) {
    for (int i = 0; i < 16 && i < bands; i++) out[i] = bleLevelByte((double)bandPower[i], (double)calOffsetDb);
  }
  uint16_t hz = 0;
  uint8_t lvl = 0;
  if (valid) {
    hz = (uint16_t)bleRoundClamp((double)peakHz, 0, 65534);
    lvl = bleLevelByte((double)peakPower, (double)calOffsetDb);
  }
  blePut16(out + 16, hz);
  out[18] = lvl;
  out[19] = (uint8_t)((live ? 0x01 : 0) | (valid ? 0x02 : 0));
}
// 0.1 unit value as uint16, or 0xFFFF when there is no reading
uint16_t bleTenthsOrNone(bool valid, float v) {
  if (!valid || bleIsNan(v)) return 0xFFFF;
  return (uint16_t)bleRoundClamp((double)v * 10.0, 0, 65534);
}

// Fields (12 bytes): magnetic readings from the Metal Detector and EMF Meter screens
//   0  uint16  metal detector: field magnitude, 0.1 uT            (0xFFFF = no reading)
//   2  uint16  metal detector: deviation from the baseline, 0.1 uT (0xFFFF = no reading)
//   4  uint16  metal detector: baseline, 0.1 uT                    (0xFFFF = no reading)
//   6  uint16  EMF meter: field magnitude, 0.1 uT                  (0xFFFF = no reading)
//   8  uint8   flags: bit0 metal detector live, bit1 metal detector valid, bit2 METAL DETECTED,
//                     bit3 EMF meter live, bit4 EMF meter valid
//   9  uint8   reserved (0)
//  10  uint8   metal detection threshold, uT
//  11  uint8   EMF meter full scale, uT
void blePackFields(uint8_t* out, bool metalValid, float metalMag, float metalDev, float baseline,
                   bool detected, bool metalLive, bool emfValid, float emfMag, bool emfLive,
                   float thresholdUt, float emfFullScaleUt) {
  blePut16(out + 0, bleTenthsOrNone(metalValid, metalMag));
  blePut16(out + 2, bleTenthsOrNone(metalValid, metalDev));
  blePut16(out + 4, bleTenthsOrNone(metalValid, baseline));
  blePut16(out + 6, bleTenthsOrNone(emfValid, emfMag));
  uint8_t flags = 0;
  if (metalLive) flags |= 0x01;
  if (metalValid) flags |= 0x02;
  if (detected) flags |= 0x04;
  if (emfLive) flags |= 0x08;
  if (emfValid) flags |= 0x10;
  out[8] = flags;
  out[9] = 0;
  out[10] = (uint8_t)bleRoundClamp((double)thresholdUt, 0, 255);
  out[11] = (uint8_t)bleRoundClamp((double)emfFullScaleUt, 0, 255);
}

// Shot (10 bytes): the Shot Timer's state
//   0  uint8   mode: 0 setup, 1 countdown, 2 timing, 3 results
//   1  uint8   countdown setting, seconds
//   2  uint8   sensitivity 1-8
//   3  uint8   seconds left in the countdown (mode 1), otherwise 0
//   4  uint8   number of shots registered (0-99)
//   5  uint8   flags: bit0 more than 99 sounds (overflow), bit1 live, bit2 valid
//   6  uint32  mode 2: time since the start beep, microseconds; mode 3: time of the last shot; otherwise 0
void blePackShotStatus(uint8_t* out, bool valid, bool live, int mode, int delaySec, int sens, int remainSec,
                       int count, bool overflow, uint32_t elapsedUs) {
  out[0] = (uint8_t)bleRoundClamp((double)mode, 0, 3);
  out[1] = (uint8_t)bleRoundClamp((double)delaySec, 0, 255);
  out[2] = (uint8_t)bleRoundClamp((double)sens, 0, 255);
  out[3] = (uint8_t)bleRoundClamp((double)remainSec, 0, 255);
  out[4] = (uint8_t)bleRoundClamp((double)count, 0, 255);
  out[5] = (uint8_t)((overflow ? 0x01 : 0) | (live ? 0x02 : 0) | (valid ? 0x04 : 0));
  blePut32(out + 6, elapsedUs);
}

// List (19 bytes): a list sent four entries at a time; the Tricorder cycles through the groups
//   0  uint8   kind: 1 = shot times (microseconds after the start beep), 2 = stopwatch laps (milliseconds, the
//              total elapsed time at each lap press)
//   1  uint8   index of the first entry in this packet (0-based)
//   2  uint8   total number of entries in the list
//   3  uint32 x 4   the entries; a slot past the end of the list is 0xFFFFFFFF
// 'values' is the whole list. Returns nothing; use bleNextListStart() to pick the next group.
int bleNextListStart(int start, int total) {
  int next = start + 4;
  return (next < total) ? next : 0;
}

void blePackList(uint8_t* out, uint8_t kind, int start, int total, const uint32_t* values) {
  if (total < 0) total = 0;
  if (total > 255) total = 255;
  if (start < 0 || start >= total) start = 0;
  out[0] = kind;
  out[1] = (uint8_t)start;
  out[2] = (uint8_t)total;
  for (int k = 0; k < 4; k++) {
    int idx = start + k;
    uint32_t v = 0xFFFFFFFFUL;
    if (idx < total) v = values[idx];
    blePut32(out + 3 + 4 * k, v);
  }
}

// Stopwatch (8 bytes)
//   0  uint32  elapsed time, milliseconds
//   4  uint8   state: 0 ready, 1 running, 2 stopped
//   5  uint8   number of laps
//   6  uint8   flags: bit0 live, bit1 valid
//   7  uint8   reserved (0)
void blePackStopwatch(uint8_t* out, bool valid, bool live, uint32_t elapsedMs, int state, int laps) {
  blePut32(out + 0, elapsedMs);
  out[4] = (uint8_t)bleRoundClamp((double)state, 0, 2);
  out[5] = (uint8_t)bleRoundClamp((double)laps, 0, 255);
  out[6] = (uint8_t)((live ? 0x01 : 0) | (valid ? 0x02 : 0));
  out[7] = 0;
}

// Countdown (8 bytes)
//   0  uint32  time left, milliseconds (the set time while setting; 0 once time is up)
//   4  uint16  the set time, seconds
//   6  uint8   state: 0 setting, 1 running, 2 paused, 3 time up
//   7  uint8   flags: bit0 live, bit1 valid
void blePackCountdown(uint8_t* out, bool valid, bool live, uint32_t remainingMs, int setSeconds, int state) {
  blePut32(out + 0, remainingMs);
  blePut16(out + 4, (uint16_t)bleRoundClamp((double)setSeconds, 0, 65535));
  out[6] = (uint8_t)bleRoundClamp((double)state, 0, 3);
  out[7] = (uint8_t)((live ? 0x01 : 0) | (valid ? 0x02 : 0));
}
// ---- PACK END ----

// ======================= Bluetooth service =======================
BLEService bleService(BLE_SERVICE_UUID);
BLECharacteristic bleEnvChar(BLE_ENV_UUID, BLERead | BLENotify, BLE_ENV_LEN, true);
BLECharacteristic blePosChar(BLE_POS_UUID, BLERead | BLENotify, BLE_POS_LEN, true);
BLECharacteristic bleDevChar(BLE_DEV_UUID, BLERead | BLENotify, BLE_DEV_LEN, true);
BLECharacteristic bleMotChar(BLE_MOT_UUID, BLERead | BLENotify, BLE_MOT_LEN, true);
BLECharacteristic bleSndChar(BLE_SND_UUID, BLERead | BLENotify, BLE_SND_LEN, true);
BLECharacteristic bleSpcChar(BLE_SPC_UUID, BLERead | BLENotify, BLE_SPC_LEN, true);
BLECharacteristic bleFldChar(BLE_FLD_UUID, BLERead | BLENotify, BLE_FLD_LEN, true);
BLECharacteristic bleShtChar(BLE_SHT_UUID, BLERead | BLENotify, BLE_SHT_LEN, true);
BLECharacteristic bleLstChar(BLE_LST_UUID, BLERead | BLENotify, BLE_LST_LEN, true);
BLECharacteristic bleSwtChar(BLE_SWT_UUID, BLERead | BLENotify, BLE_SWT_LEN, true);
BLECharacteristic bleCdtChar(BLE_CDT_UUID, BLERead | BLENotify, BLE_CDT_LEN, true);
BLECharacteristic bleCmdChar(BLE_CMD_UUID, BLEWrite, BLE_CMD_LEN);     // the phone writes commands here (Remote.ino)

bool bleStarted = false;               // BLE.begin() succeeded (done once, the first time Bluetooth is turned on)
bool bleEnabled = false;               // user switch: advertising / connectable
bool bleConnected = false;
bool bleFailed = false;
unsigned long bleLastMs[BLE_STREAMS];  // when each stream was last sent
int bleNextStream = 0;                 // where the round-robin starts, so no stream can be starved
bool bleShotSeen = false;              // the Shot Timer / Stopwatch / Countdown screens have been opened at least once
bool bleSwSeen = false;
bool bleCdSeen = false;
int bleListStart = 0;                  // next group of the shot / lap list to send
bool bleListWasLaps = false;
bool bleListIdleShots = true;          // when idle, the shot list and the lap list take turns
unsigned long bleSentCount = 0;        // data packets sent since power-up
bool bleStatusChanged = true;

// Stable ids for the "active screen" field, so the phone doesn't depend on the order of the State enum
uint8_t bleScreenId() {
  switch (currentState) {
    case NAVIGATION:     return 1;
    case METALDETECTOR:  return 2;
    case EMFMETER:       return 3;
    case ATMOSPHERIC:    return 4;
    case DBMETER:        return 5;
    case SPECTRUM:       return 6;
    case BATTERY:        return 7;
    case SYSINFO:        return 8;
    case TIMEZONE:       return 9;
    case REBOOT:         return 10;
    case IRSEND:         return 11;
    case IRRECEIVER:     return 12;
    case SHOTTIMER:      return 13;
    case STOPWATCH:      return 14;
    case COUNTDOWNTIMER: return 15;
    case BLUETOOTH_SCREEN: return 16;
    case MAIN_MENU: case GEO_MENU: case ENV_MENU: case COMMS_MENU: case SYS_MENU: case UTIL_MENU: return 0;
    default:             return 255;
  }
}

// Same battery calculation as the Battery screen
void bleReadBattery(int* mv, int* pct) {
  int adcValue = analogRead(VOLTAGE_PIN);
  float voltage = (adcValue / 1023.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
  float percentage = ((voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V)) * 100.0;
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;
  *mv = (int)(voltage * 1000.0 + 0.5);
  *pct = (int)(percentage + 0.5);
}

bool bleBegin() {
  if (bleStarted) return true;
  if (!BLE.begin()) return false;
  BLE.setLocalName(BLE_DEVICE_NAME);
  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setAdvertisedService(bleService);
  bleService.addCharacteristic(bleEnvChar);
  bleService.addCharacteristic(blePosChar);
  bleService.addCharacteristic(bleDevChar);
  bleService.addCharacteristic(bleMotChar);
  bleService.addCharacteristic(bleSndChar);
  bleService.addCharacteristic(bleSpcChar);
  bleService.addCharacteristic(bleFldChar);
  bleService.addCharacteristic(bleShtChar);
  bleService.addCharacteristic(bleLstChar);
  bleService.addCharacteristic(bleSwtChar);
  bleService.addCharacteristic(bleCdtChar);
  bleService.addCharacteristic(bleCmdChar);
  BLE.addService(bleService);

  uint8_t zero[BLE_POS_LEN];
  memset(zero, 0, sizeof(zero));
  bleEnvChar.writeValue(zero, BLE_ENV_LEN);
  blePosChar.writeValue(zero, BLE_POS_LEN);
  bleDevChar.writeValue(zero, BLE_DEV_LEN);
  bleMotChar.writeValue(zero, BLE_MOT_LEN);
  bleSndChar.writeValue(zero, BLE_SND_LEN);
  bleSpcChar.writeValue(zero, BLE_SPC_LEN);
  bleFldChar.writeValue(zero, BLE_FLD_LEN);
  bleShtChar.writeValue(zero, BLE_SHT_LEN);
  bleLstChar.writeValue(zero, BLE_LST_LEN);
  bleSwtChar.writeValue(zero, BLE_SWT_LEN);
  bleCdtChar.writeValue(zero, BLE_CDT_LEN);
  bleStarted = true;
  return true;
}

void bleSetEnabled(bool on) {
  if (on == bleEnabled) return;
  if (on) {
    if (!bleBegin()) {
      bleFailed = true;
      bleEnabled = false;
    } else {
      bleFailed = false;
      bleEnabled = true;
      BLE.advertise();
    }
  } else {
    bleEnabled = false;
    remoteSetEnabled(false);                        // remote control never outlives Bluetooth
    if (bleStarted) {
      BLE.stopAdvertise();
      if (BLE.connected()) BLE.disconnect();
    }
    bleConnected = false;
  }
  bleStatusChanged = true;
}

void bleSendEnvironment() {
  uint8_t pkt[BLE_ENV_LEN];
  float t = HS300x.readTemperature();
  float h = HS300x.readHumidity();
  float p = BARO.readPressure(MILLIBAR) * BLE_PRESSURE_CAL;   // hPa, corrected (see BLE_PRESSURE_CAL)
  blePackEnvironment(pkt, t, h, p, millis());
  bleEnvChar.writeValue(pkt, BLE_ENV_LEN);
  bleSentCount++;
}

void bleSendPosition() {
  uint8_t pkt[BLE_POS_LEN];
  bool loc = gps.location.isValid();
  unsigned long fixAge = gps.location.age();          // 0xFFFFFFFF = never had a fix
  blePackPosition(pkt,
                  loc, loc ? gps.location.lat() : 0.0, loc ? gps.location.lng() : 0.0,
                  gps.altitude.isValid(), gps.altitude.meters(),
                  gps.speed.isValid(), gps.speed.mps(),
                  gps.course.isValid(), gps.course.deg(),
                  gps.satellites.isValid() ? (int)gps.satellites.value() : -1,
                  gps.hdop.isValid(), gps.hdop.hdop(),
                  fixAge != 0xFFFFFFFFUL, (uint32_t)fixAge);
  blePosChar.writeValue(pkt, BLE_POS_LEN);
  bleSentCount++;
}

void bleSendDevice() {
  uint8_t pkt[BLE_DEV_LEN];
  int mv, pct;
  bleReadBattery(&mv, &pct);
  uint8_t flags = (uint8_t)((remoteEnabled ? 0x01 : 0) | (remoteBusy() ? 0x02 : 0) | (remoteHookMissing() ? 0x04 : 0));
  blePackDevice(pkt, mv, pct, bleScreenId(), millis(), sysLargestFreeBlock(), FWver, flags);   // FWver: float in the main sketch
  bleDevChar.writeValue(pkt, BLE_DEV_LEN);
  bleSentCount++;
}

void bleSendMotion() {
  uint8_t pkt[BLE_MOT_LEN];
  unsigned long now = millis();
  bool have = (telMotionMs != 0);
  bool live = have && currentState == NAVIGATION && (now - telMotionMs) < 1500;
  bool calibrated = (magOffX != 0.0 || magOffY != 0.0 || magOffZ != 0.0 ||
                     magScaleX != 1.0 || magScaleY != 1.0 || magScaleZ != 1.0);
  blePackMotion(pkt, have, telHeading, telPitch, telRoll, telField, telLevel, calibrated, live);
  bleMotChar.writeValue(pkt, BLE_MOT_LEN);
  bleSentCount++;
}

void bleSendSound() {
  uint8_t pkt[BLE_SND_LEN];
  bool live = (currentState == DBMETER);
  bool valid = live || dbMaxLevel > 0.0f;
  blePackSound(pkt, valid, dbLevel, dbMaxLevel, dbLeqDb, dbPeakDb, dbWeightA, dbSlow, live);
  bleSndChar.writeValue(pkt, BLE_SND_LEN);
  bleSentCount++;
}

void bleSendSpectrum() {
  uint8_t pkt[BLE_SPC_LEN];
  unsigned long now = millis();
  bool have = (telSpecMs != 0);
  bool live = have && currentState == SPECTRUM && (now - telSpecMs) < 1500;
  blePackSpectrum(pkt, specBandPow, 16, dbCalOffset, telSpecPeakHz, telSpecPeakPow, have, live);
  bleSpcChar.writeValue(pkt, BLE_SPC_LEN);
  bleSentCount++;
}

void bleSendFields() {
  uint8_t pkt[BLE_FLD_LEN];
  unsigned long now = millis();
  // these two screens refresh every half second, so "live" allows a little over that
  bool metalHave = (telMetalMs != 0);
  bool metalLive = metalHave && currentState == METALDETECTOR && (now - telMetalMs) < 2500;
  bool emfHave = (telEmfMs != 0);
  bool emfLive = emfHave && currentState == EMFMETER && (now - telEmfMs) < 2500;
  blePackFields(pkt, metalHave, telMetalMag, telMetalDev, baselineMagnitude, telMetalDetected, metalLive,
                emfHave, telEmfMag, emfLive, DETECTION_THRESHOLD, MAX_EMF);
  bleFldChar.writeValue(pkt, BLE_FLD_LEN);
  bleSentCount++;
}

void bleSendShot() {
  uint8_t pkt[BLE_SHT_LEN];
  bool live = (currentState == SHOTTIMER);
  if (live) bleShotSeen = true;
  bool valid = bleShotSeen || shotCount > 0;
  uint32_t elapsed = 0;
  int remain = 0;
  if (shotMode == 2 && shotArmed) {
    elapsed = (uint32_t)micros() - shotStartUs;                      // timing: running time
  } else if (shotMode == 3 && shotCount > 0) {
    elapsed = shotTimes[shotCount - 1];                              // results: the total = the last shot
  }
  if (shotMode == 1) {
    long left = (long)(shotCountdownEndMs - millis());
    remain = (left > 0) ? (int)((left + 999) / 1000) : 0;
  }
  blePackShotStatus(pkt, valid, live, shotMode, shotDelaySec, shotSens, remain, shotCount, shotOverflow, elapsed);
  bleShtChar.writeValue(pkt, BLE_SHT_LEN);
  bleSentCount++;
}

// The shot times (kind 1) or the stopwatch laps (kind 2), four at a time, cycling through the list
void bleSendList() {
  uint8_t pkt[BLE_LST_LEN];
  bool laps;
  if (currentState == STOPWATCH) {
    laps = true;
  } else if (currentState == SHOTTIMER) {
    laps = false;
  } else {                                      // idle: the two lists take turns
    laps = !bleListIdleShots;
    bleListIdleShots = !bleListIdleShots;
  }
  if (laps != bleListWasLaps) {                 // switched lists: start again from the first entry
    bleListWasLaps = laps;
    bleListStart = 0;
  }
  if (laps) {
    uint32_t tmp[20];
    int n = swLapCount;
    if (n < 0) n = 0;
    if (n > 20) n = 20;
    for (int i = 0; i < n; i++) tmp[i] = (uint32_t)swLapTotals[i];
    if (bleListStart >= n) bleListStart = 0;
    blePackList(pkt, 2, bleListStart, n, tmp);
    bleListStart = bleNextListStart(bleListStart, n);
  } else {
    int n = shotCount;
    if (n < 0) n = 0;
    if (n > 99) n = 99;
    if (bleListStart >= n) bleListStart = 0;
    blePackList(pkt, 1, bleListStart, n, shotTimes);
    bleListStart = bleNextListStart(bleListStart, n);
  }
  bleLstChar.writeValue(pkt, BLE_LST_LEN);
  bleSentCount++;
}

void bleSendStopwatch() {
  uint8_t pkt[BLE_SWT_LEN];
  bool live = (currentState == STOPWATCH);
  if (live) bleSwSeen = true;
  uint32_t elapsed = (uint32_t)swElapsed();
  int state = swRunning ? 1 : ((elapsed > 0) ? 2 : 0);
  blePackStopwatch(pkt, bleSwSeen, live, elapsed, state, swLapCount);
  bleSwtChar.writeValue(pkt, BLE_SWT_LEN);
  bleSentCount++;
}

void bleSendCountdown() {
  uint8_t pkt[BLE_CDT_LEN];
  bool live = (currentState == COUNTDOWNTIMER);
  if (live) bleCdSeen = true;
  int setSeconds = cdSetMin * 60 + cdSetSecs;
  uint32_t remaining = 0;
  if (cdMode == 1) {
    long left = (long)(cdEndMs - millis());
    remaining = (left > 0) ? (uint32_t)left : 0;
  } else if (cdMode == 2) {
    remaining = (uint32_t)cdRemainMs;
  } else if (cdMode == 0) {
    remaining = (uint32_t)setSeconds * 1000UL;
  }
  blePackCountdown(pkt, bleCdSeen, live, remaining, setSeconds, cdMode);
  bleCdtChar.writeValue(pkt, BLE_CDT_LEN);
  bleSentCount++;
}

// ---- scheduler: the streams are numbered 0-10 ----
//   0 Environment  1 Position  2 Device  3 Motion  4 Sound  5 Spectrum  6 Fields  7 Shot  8 List  9 Stopwatch  10 Countdown

// true while the Tricorder screen that feeds this stream is open
bool bleStreamLive(int i) {
  switch (i) {
    case 3:  return currentState == NAVIGATION;
    case 4:  return currentState == DBMETER;
    case 5:  return currentState == SPECTRUM;
    case 6:  return currentState == METALDETECTOR || currentState == EMFMETER;
    case 7:  return currentState == SHOTTIMER;
    case 8:  return currentState == SHOTTIMER || currentState == STOPWATCH;
    case 9:  return currentState == STOPWATCH;
    case 10: return currentState == COUNTDOWNTIMER;
    default: return false;
  }
}

unsigned long blePeriod(int i) {
  if (i <= 2) return BLE_NOTIFY_PERIOD_MS;
  return bleStreamLive(i) ? BLE_LIVE_PERIOD_MS : BLE_IDLE_PERIOD_MS;
}

void bleSendStream(int i) {
  switch (i) {
    case 0:  bleSendEnvironment(); break;
    case 1:  bleSendPosition(); break;
    case 2:  bleSendDevice(); break;
    case 3:  bleSendMotion(); break;
    case 4:  bleSendSound(); break;
    case 5:  bleSendSpectrum(); break;
    case 6:  bleSendFields(); break;
    case 7:  bleSendShot(); break;
    case 8:  bleSendList(); break;
    case 9:  bleSendStopwatch(); break;
    case 10: bleSendCountdown(); break;
  }
}

// Call from loop() on every pass
void bleUpdate() {
  if (!bleEnabled) return;
  BLE.poll();

  // a command from the phone? (ignored unless remote control has been switched on at the Tricorder)
  if (bleCmdChar.written()) {
    uint8_t cmd[BLE_CMD_LEN] = {0, 0, 0, 0};
    int n = bleCmdChar.valueLength();
    if (n > BLE_CMD_LEN) n = BLE_CMD_LEN;
    if (n > 0) bleCmdChar.readValue(cmd, n);
    remoteHandleCommand(cmd, n);
  }

  bool c = BLE.connected();
  unsigned long now = millis();
  if (c != bleConnected) {
    bleConnected = c;
    bleStatusChanged = true;
    if (c) {
      Serial.println(F("BLE: phone connected"));
      // stagger the first packets so no single loop pass does all the sensor reads
      for (int i = 0; i < BLE_STREAMS; i++) bleLastMs[i] = now - blePeriod(i) + (unsigned long)i * 80UL;
      bleNextStream = 0;
    } else {
      Serial.println(F("BLE: phone disconnected"));
      remotePhoneDisconnected();
      BLE.advertise();                                                // let the phone reconnect
    }
  }
  if (!c) return;

  // send at most one packet per pass, starting after the stream that was sent last (round-robin)
  for (int k = 0; k < BLE_STREAMS; k++) {
    int i = (bleNextStream + k) % BLE_STREAMS;
    if (now - bleLastMs[i] >= blePeriod(i)) {
      bleLastMs[i] = now;
      bleNextStream = (i + 1) % BLE_STREAMS;
      bleSendStream(i);
      return;
    }
  }
}

// ======================= screen =======================
bool  bleUiInit = false;
unsigned long bleLastUiMs = 0;
char  bleShown[4][16];

const char* bleStatusText() {
  if (bleFailed) return "Error";
  if (!bleEnabled) return "Off";
  if (bleConnected) return "Connected";
  return "Advertising";
}

void bleDrawOled() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 9);
  display.print(F("BLUETOOTH"));
  display.setCursor(0, 20);
  display.print(F("Status: "));
  display.print(bleStatusText());
  display.setCursor(0, 30);
  display.print(F("Remote: "));
  display.print(remoteEnabled ? "ON" : "off");
  if (bleStarted) {
    display.setCursor(0, 40);
    display.print(BLE.address());
  }
  display.setCursor(0, 52);
  display.print(bleEnabled ? "ENT=Off  RIGHT=Remote" : "ENTER = Turn on");
  display.display();
}

void bleDrawTft() {
  char buf[16];
  tftTextDiff(bleShown[0], 0, 0, 1, "BLUETOOTH", CYAN, BLACK);
  uint16_t color = WHITE;
  const char* big = "OFF   ";
  if (bleFailed) { color = RED; big = "ERROR "; }
  else if (bleEnabled && bleConnected) { color = GREEN; big = "LINKED"; }
  else if (bleEnabled) { color = BLUE; big = "WAIT  "; }
  tftTextDiff(bleShown[1], 0, 16, 2, big, color, BLACK);
  snprintf(buf, sizeof(buf), "Sent %-7lu", bleSentCount);
  tftTextDiff(bleShown[2], 0, 44, 1, buf, YELLOW, BLACK);
  tftTextDiff(bleShown[3], 0, 54, 1, remoteEnabled ? "Remote ON " : "Remote off", remoteEnabled ? GREEN : WHITE, BLACK);
}

void BluetoothScreen(int pressType) {
  // --- Exit (Bluetooth keeps running in the background) ---
  if (pressType == LONG_PRESS && lastButton == 5) {
    bleUiInit = false;
    tft.fillScreen(BLACK);
    currentState = COMMS_MENU;
    return;
  }

  // --- First pass after entering ---
  if (!bleUiInit) {
    bleUiInit = true;
    tft.fillScreen(BLACK);
    for (int i = 0; i < 4; i++) tftTextForget(bleShown[i], 16);
    bleStatusChanged = true;
    bleLastUiMs = 0;
  }

  // --- Enter: turn Bluetooth on / off ---
  if (pressType == SHORT_PRESS && lastButton == 1) {
    if (!bleEnabled) {
      display.clearDisplay();                       // BLE.begin() takes a moment the first time
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 20);
      display.print(F("Starting Bluetooth..."));
      display.display();
    }
    bleSetEnabled(!bleEnabled);
  }

  // --- Right: switch remote control on / off (only while Bluetooth is on) ---
  if (pressType == SHORT_PRESS && lastButton == 3 && bleEnabled) {
    remoteSetEnabled(!remoteEnabled);
    bleStatusChanged = true;
  }

  // --- Redraw when something changed, and once a second for the packet counter ---
  unsigned long now = millis();
  if (bleStatusChanged || now - bleLastUiMs >= 1000) {
    bleStatusChanged = false;
    bleLastUiMs = now;
    bleDrawTft();
    bleDrawOled();
  }
}
