# Tricorder BLE Protocol, version 13

This is the contract between the Tricorder firmware (`Bluetooth.ino`) and any phone app. If either side changes
it, change this document first.

## Connection

| | |
|---|---|
| Role | The Tricorder is the **peripheral**. It advertises; the phone is the **central** and connects. |
| Advertised name | `Tricorder` |
| Service UUID | `3f5a0000-7c2b-4e6d-a1b3-5d8e9c0f1a20` |
| Default state | Bluetooth is **off** after every power-up. Turn it on at COMMS menu > Bluetooth > ENTER. |
| Updates | Environment, Position and Device: once a second. Every other characteristic: 5 times a second while its Tricorder screen is open ("live"), otherwise every 2 seconds. Packets are sent one at a time, taking turns. |
| Security | None: no pairing or encryption. While Bluetooth is on, anyone nearby can connect and read the data. Remote control (the Command characteristic) is a separate switch on the Tricorder and is off until the user turns it on; see "Remote control" below. |

Every characteristic except Command supports **Read** and **Notify**. All numbers are **little-endian**. Signed values are
two's complement.

## Characteristics

| Name | UUID | Length |
|---|---|---|
| Environment | `3f5a0001-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 12 bytes |
| Position | `3f5a0002-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 20 bytes |
| Device | `3f5a0003-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 20 bytes (19 in protocol v6 and v7, 16 in v5) |
| Motion | `3f5a0004-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 12 bytes (10 in protocol v8 and older) |
| Sound | `3f5a0005-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 10 bytes |
| Audio | `3f5a000e-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 20 to 244 bytes, notify only (protocol v10, v11 added the quality choice, v12 the larger packets) |
| Seismometer | `3f5a000f-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 12 bytes (protocol v13) |
| Spectrum | `3f5a0006-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 20 bytes |
| Fields | `3f5a0007-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 12 bytes |
| Shot | `3f5a0008-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 10 bytes |
| List | `3f5a0009-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 19 bytes |
| Stopwatch | `3f5a000a-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 8 bytes |
| Countdown | `3f5a000b-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 8 bytes |
| Command | `3f5a000c-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 4 bytes, **write only** |
| IR Receive (protocol v7) | `3f5a000d-7c2b-4e6d-a1b3-5d8e9c0f1a20` | 19 bytes |

All of these except Command support Read and Notify. Command supports Write (with response).

## Live data

Motion, Sound, Spectrum, Fields, Shot, List, Stopwatch and Countdown come from the matching Tricorder screens
(Navigation, dB Meter, Spectrum, Metal Detector and EMF Meter, Shot Timer, Stopwatch, Countdown). They are
**live** while that screen is open on the Tricorder. When it isn't, the packet still arrives (every 2 seconds)
with the **last known values** and its *live* flag cleared, or its *valid* flag cleared if the screen has never
been opened since power-up. A phone app should show these as "not live" and tell the user which screen to open.

### Environment (12 bytes)

| Offset | Type | Field | Unit | "No data" value |
|---|---|---|---|---|
| 0 | int16 | temperature | 0.01 degC | `0x8000` (-32768) |
| 2 | uint16 | relative humidity | 0.01 % | `0xFFFF` |
| 4 | uint32 | pressure | Pa (divide by 100 for hPa) | `0` |
| 8 | uint32 | device uptime | ms | |

The temperature sensor sits on a board that warms itself, so expect it to read a few degrees high.

### Position (20 bytes)

| Offset | Type | Field | Unit | "No data" value |
|---|---|---|---|---|
| 0 | int32 | latitude | 1e-7 degrees | 0 (check flag bit 0) |
| 4 | int32 | longitude | 1e-7 degrees | 0 (check flag bit 0) |
| 8 | int32 | altitude above sea level | cm | 0 (check flag bit 1) |
| 12 | uint16 | speed | cm/s | `0xFFFF` |
| 14 | uint16 | course | 0.1 degrees, 0-3599 | `0xFFFF` |
| 16 | uint8 | satellites in use | | `0xFF` |
| 17 | uint8 | HDOP | 0.1 | `0xFF` |
| 18 | uint8 | age of position fix | seconds | `0xFF` = never had a fix; `0xFE` = 254 s or more |
| 19 | uint8 | flags | bit 0 position valid, bit 1 altitude valid, bit 2 speed valid, bit 3 course valid | |

Trust a field only when its flag bit is set. Latitude/longitude: divide by 10,000,000 for degrees.

### Device (20 bytes)

| Offset | Type | Field | Unit |
|---|---|---|---|
| 0 | uint16 | battery voltage | mV |
| 2 | uint8 | battery charge | 0-100 % |
| 3 | uint8 | active screen id | see below |
| 4 | uint32 | device uptime | ms |
| 8 | uint32 | largest free memory block | bytes |
| 12 | uint8 | protocol version | currently 13 |
| 13 | uint8 | flags | bit 0 remote control is switched on at the Tricorder, bit 1 the Tricorder is still carrying out the last remote command, bit 2 remote control is on but the sketch's `loop()` never applies the commands (the `remoteApply()` line is missing), bit 3 the time zone in use is saved in flash (otherwise it is lost at power-off; protocol v8) |
| 14 | uint16 | firmware version x 1000 | 600 = firmware 0.6; `0xFFFF` = unknown |
| 16 | uint24 | IR codes that exist (protocol v6) | bit n set = IR code n (see Remote control) has a code on the Tricorder. A key whose bit is clear should be greyed out. Bits 0-11 are set until a remote is cloned; bits 12-16 (Eject, Play, Stop, Pause, Menu) are clear until then. A protocol v5 Tricorder sends only the first 16 bytes: treat codes 0-11 as present and the rest as absent. |
| 19 | uint8 | time zone in use (protocol v8) | 0 Eastern, 1 Central, 2 Mountain, 3 Arizona (no DST), 4 Pacific, 5 Alaska, 6 Hawaii (no DST). A v6 or v7 Device packet is 19 bytes long and has no time zone. |

### Motion (12 bytes)

Compass values, as the Tricorder's Navigation screen shows them (declination included).

| Offset | Type | Field | Unit | "No data" value |
|---|---|---|---|---|
| 0 | uint16 | heading, clockwise from north | 0.1 degrees, 0-3599 | `0xFFFF` |
| 2 | int16 | pitch | 0.1 degrees | `0x8000` |
| 4 | int16 | roll | 0.1 degrees | `0x8000` |
| 6 | uint16 | magnetic field strength (calibrated) | 0.1 uT | `0xFFFF` |
| 8 | uint8 | flags | bit 0 held level, bit 1 values valid, bit 2 magnetometer calibrated, bit 3 **live** | |
| 9 | uint8 | flags 2 (protocol v9) | bit 0 the declination is saved in flash (otherwise it is lost at power-off) | |
| 10 | int16 | compass declination in use (protocol v9), east positive | 0.01 degrees, -4500 to 4500 | `0x8000` |

A protocol v8 or older Motion packet is 10 bytes long and has neither field (byte 9 was reserved, 0).
| 9 | uint8 | reserved | always 0 | |

### Sound (10 bytes)

The dB Meter's readings, in dB SPL. *A-weighted* and *Slow* describe how the Tricorder is currently set.

| Offset | Type | Field | Unit |
|---|---|---|---|
| 0 | uint16 | current level (time-weighted) | 0.1 dB |
| 2 | uint16 | highest level since reset | 0.1 dB |
| 4 | uint16 | average level since reset (Leq) | 0.1 dB |
| 6 | uint16 | loudest single sample since reset | 0.1 dB |
| 8 | uint8 | flags | bit 0 A-weighted (else flat), bit 1 Slow (else Fast), bit 2 **live**, bit 3 values valid |
| 9 | uint8 | reserved | always 0 |

### Spectrum (20 bytes)

| Offset | Type | Field | Unit |
|---|---|---|---|
| 0-15 | uint8 x 16 | level of each band (table below) | dB SPL, flat-weighted, 1 dB steps; 0 = nothing |
| 16 | uint16 | frequency of the strongest bin | Hz |
| 18 | uint8 | level of the strongest bin | dB SPL |
| 19 | uint8 | flags | bit 0 **live**, bit 1 values valid |

Each band's level is the loudest single frequency (62.5 Hz wide bin) inside the band. Levels use the dB Meter's
calibration, so they are comparable with its flat-weighted readings.

| Band | From Hz | To Hz | Band | From Hz | To Hz |
|---|---|---|---|---|---|
| 0 | 62.5 | 125 | 8 | 1000 | 1312.5 |
| 1 | 125 | 187.5 | 9 | 1312.5 | 1750 |
| 2 | 187.5 | 250 | 10 | 1750 | 2312.5 |
| 3 | 250 | 312.5 | 11 | 2312.5 | 3062.5 |
| 4 | 312.5 | 437.5 | 12 | 3062.5 | 4062.5 |
| 5 | 437.5 | 562.5 | 13 | 4062.5 | 5375 |
| 6 | 562.5 | 750 | 14 | 5375 | 6687.5 |
| 7 | 750 | 1000 | 15 | 6687.5 | 8000 |

### Fields (12 bytes)

Magnetic readings from the Metal Detector and EMF Meter screens (raw field magnitude, as those screens show it).

| Offset | Type | Field | Unit | "No data" value |
|---|---|---|---|---|
| 0 | uint16 | metal detector: field magnitude | 0.1 uT | `0xFFFF` |
| 2 | uint16 | metal detector: deviation from the baseline | 0.1 uT | `0xFFFF` |
| 4 | uint16 | metal detector: baseline | 0.1 uT | `0xFFFF` |
| 6 | uint16 | EMF meter: field magnitude | 0.1 uT | `0xFFFF` |
| 8 | uint8 | flags | bit 0 metal **live**, bit 1 metal valid, bit 2 **METAL DETECTED**, bit 3 EMF **live**, bit 4 EMF valid | |
| 9 | uint8 | reserved | always 0 | |
| 10 | uint8 | metal detection threshold | uT | |
| 11 | uint8 | EMF meter full scale | uT | |

These two screens refresh every half second, so they are live at about 2 updates a second.

### Shot (10 bytes)

The Shot Timer's state. The shot times themselves are in the List characteristic.

| Offset | Type | Field | Unit |
|---|---|---|---|
| 0 | uint8 | mode | 0 setup, 1 countdown, 2 timing, 3 results |
| 1 | uint8 | countdown setting | seconds |
| 2 | uint8 | sensitivity | 1-8 |
| 3 | uint8 | seconds left in the countdown (mode 1), otherwise 0 | seconds |
| 4 | uint8 | shots registered | 0-99 |
| 5 | uint8 | flags | bit 0 more than 99 sounds (overflow), bit 1 **live**, bit 2 valid |
| 6 | uint32 | mode 2: time since the start beep; mode 3: time of the last shot (= the total); otherwise 0 | microseconds |

While timing, a phone should keep the running time moving between packets (add the time since the packet arrived).

### List (19 bytes)

The shot times or the stopwatch laps, **four entries per packet**. The Tricorder cycles through the groups, so a
phone rebuilds the whole list by collecting packets. A list of 99 entries is complete after about 5 seconds.

| Offset | Type | Field |
|---|---|---|
| 0 | uint8 | kind: 1 = shot times (microseconds after the start beep), 2 = stopwatch laps (milliseconds: the total elapsed time at each lap press) |
| 1 | uint8 | index of the first entry in this packet (0-based) |
| 2 | uint8 | total number of entries in the list |
| 3 | uint32 x 4 | the entries; a slot past the end of the list is `0xFFFFFFFF` |

To rebuild a list: make it exactly *total* entries long, then store this packet's entries at *start*, *start* + 1, ...
The total is the time of the last entry; a split is the difference between neighbouring entries (the first split is
the first time). A total of 0 means the list is empty: clear it. While neither screen is open, the shot list and
the lap list take turns, every 2 seconds each.

### Stopwatch (8 bytes)

| Offset | Type | Field | Unit |
|---|---|---|---|
| 0 | uint32 | elapsed time | milliseconds |
| 4 | uint8 | state | 0 ready, 1 running, 2 stopped |
| 5 | uint8 | number of laps | |
| 6 | uint8 | flags | bit 0 **live**, bit 1 valid |
| 7 | uint8 | reserved | always 0 |

The Tricorder clears its stopwatch when you leave the screen.

### Countdown (8 bytes)

| Offset | Type | Field | Unit |
|---|---|---|---|
| 0 | uint32 | time left (the set time while setting; 0 once time is up) | milliseconds |
| 4 | uint16 | the set time | seconds |
| 6 | uint8 | state | 0 setting, 1 running, 2 paused, 3 time up |
| 7 | uint8 | flags | bit 0 **live**, bit 1 valid |

While running, a phone should keep the time ticking down between packets.

### IR Receive (19 bytes, protocol v7)

What the Tricorder's IR Receive screen (COMMS > IR RX) is doing. Sent every 200 ms while that screen is open, every 2 s
otherwise (it then still reports the last state).

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | uint8 | screen | 0 menu, 1 IR data view, 2 clone, 3 "discard the new codes?", 4 "back to the default codes?", 5 result |
| 1 | uint8 | flags | bit 0 **live** (the IR Receive screen is open), bit 1 a cloned set is in use, bit 2 the last save to flash worked, bit 3 this firmware has a flash store, bit 4 at least one IR frame has been received since power-up |
| 2 | uint8 | clone prompt | the button being asked for, in the clone's order, 0-16 (17 = finished) |
| 3 | uint8 | result | on the result screen: 1 clone complete, 2 nothing recorded, 3 cancelled, 4 default codes restored |
| 4 | uint8 | note | under the prompt: 0 none, 1 got, 2 skipped, 3 redo, 4 repeat frame ignored, 5 frame too long |
| 5 | uint8 | note button | the IR button number (0-16) the note is about |
| 6 | uint8 | duplicate | the earlier button whose code equals the one just recorded; 255 = none |
| 7 | uint8 | protocol | of the last frame: 0 unknown, 1 NEC, 2 NEC repeat, 3 Samsung, 4 Sony SIRC; 255 = none yet |
| 8 | uint8 | bits | data bits (NEC and Samsung 32; Sony 12, 15 or 20) |
| 9 | uint32 | value | of the last frame (NEC / Samsung: address in the low byte, command in bits 16-23; Sony: command in the low 7 bits, address above) |
| 13 | uint16 | pulses | marks and spaces in the last frame |
| 15 | uint8 | counter | counts every frame received (wraps at 256): a change means a new frame |
| 16 | uint24 | progress | bit n set = IR button n of the clone being recorded has a code |

The clone asks for the buttons in this order (button numbers): 0 Power, 1 Vol Up, 2 Vol Down, 3 OK, 4 Back, 5 Exit,
6 Left, 7 Right, **9 Up, 8 Down**, 10 Amazon, 11 YouTube, 12 Eject, 13 Play, 14 Stop, 15 Pause, 16 Menu. A button
that has been passed (prompt higher than its position) and has no bit in the progress mask was skipped.

### Seismometer (12 bytes, protocol v13)
Characteristic `3f5a000f-...`. Sent 5 times a second while the Seismometer screen is open on the Tricorder (screen id 19), otherwise every
2 seconds. "Shaking" is the size of the acceleration left after the slowly changing part (gravity) has been taken away, in
any direction, in mg (thousandths of a g).

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | uint8 | flags | bit 0 the screen has run at least once (valid), bit 1 live (open and updating), bit 2 an event is active, bit 3 getting ready (the first 5 seconds after opening or after a long ENTER: ignore the numbers) |
| 1 | uint8 | sensitivity | 0-7: the full scale of the Tricorder's seismograph, 2, 5, 10, 20, 50, 100, 200, 500 mg |
| 2 | uint16 | shaking now | RMS over about the last second, hundredths of a mg (65535 = 655 mg or more) |
| 4 | uint16 | strongest shake since the last packet | tenths of a mg (65535 = 6553 mg or more). A short jolt between two packets is never missed. |
| 6 | uint16 | strongest shake since reset | tenths of a mg |
| 8 | uint16 | events since reset | an event is a quarter second whose strongest shake is more than 5 times the noise floor (and at least 5 mg) |
| 10 | uint16 | noise floor | hundredths of a mg: the slow average of the quiet level |

The phone app logs, for each row of its data log, the strongest shake (the largest "since the last packet" value of that
interval) and the average shaking (the root mean square of "shaking now" over the packets of that interval), but only from
packets that are live.

## Remote control

The phone can press the Tricorder's buttons and open its screens by writing 4-byte commands to the **Command**
characteristic. The Tricorder feeds them into the same code that handles its real buttons, so each screen behaves
exactly as if the button had been pressed on the device.

| Byte | Meaning |
|---|---|
| 0 | command: 1 = BUTTON, 2 = GOTO, 3 = IR, 4 = IR RECEIVE (protocol v7), 5 = TIME ZONE, 6 = REBOOT (protocol v8), 7 = DECLINATION (protocol v9), 8 = LIVE AUDIO (protocol v10, quality byte in v11, packet size byte in v12) |
| 1 | BUTTON: button, 1 ENTER, 2 UP, 3 RIGHT, 4 DOWN, 5 LEFT. GOTO: screen id (0 = the main menu). IR: the code number (0-16, below). IR RECEIVE: the action, 1 start a clone, 2 open the IR data view, 3 back to the IR Receive menu. TIME ZONE: the zone 0-6 (see the Device packet). REBOOT: `0x52` |
| 2 | BUTTON: 0 = short press, 1 = long press. REBOOT: `0x42`. Otherwise 0 |
| 3 | REBOOT: `0x54`. Otherwise 0 |

- **GOTO** may open: Navigation (1), Metal Detector (2), EMF Meter (3), Atmospheric (4), dB Meter (5), Spectrum (6),
  Battery (7), System Info (8), IR Send (11), IR Receive (12), Shot Timer (13), Stopwatch (14), Countdown (15), Seismometer (19), and the main menu (0).
  Opening IR Send transmits nothing. The Tricorder
  backs out to its main menu and walks to the screen with a few presses; this takes under a second, during which the
  Device flags report *busy*.
- **Safety.** Remote control is **off** every time the Tricorder starts. The user switches it on at the Tricorder
  (COMMS menu > Bluetooth, RIGHT), and it switches off when Bluetooth is turned off. While it is off, commands are
  silently ignored. The Device packet's flag bit 0 says whether it is on.
- The phone can never GOTO the Reboot, Time Zone, Declination and Bluetooth screens. On Reboot, Time Zone, Declination and IR Send only a LEFT
  press (to back out) is accepted from a BUTTON command.
- **TIME ZONE** (command 5) sets the zone the Tricorder uses for its GPS clock, exactly as ENTER does on its Time Zone
  screen (it is saved to flash if the core allows; Device flag bit 3 says whether it was). A zone above 6 is ignored.
  It works whatever screen the Tricorder is on: the phone still cannot open the Time Zone screen itself.
- **DECLINATION** (command 7) sets the compass declination the Tricorder adds to its heading, exactly as ENTER does on its
  Declination screen (it is saved to flash if the core allows; Motion flags 2 bit 0 says whether it was). The command is
  `07 lo hi 00`: bytes 1 and 2 are a signed 16-bit number (little endian) of hundredths of a degree, east positive. A value
  outside -4500 to 4500 is ignored. It works whatever screen the Tricorder is on.
- **LIVE AUDIO** (command 8) starts or stops the live microphone stream: `08 01 q s` starts, `08 00 00 00` stops. `q` is the quality (protocol v11): 0 = 8 kHz, 1 = 5.33 kHz, 2 = 4 kHz, 3 = 16 kHz (protocol v12; needs big packets, about 33 a second; anything else means 0). `s` is the packet size (protocol v12): each packet is `4 + 8 x s` bytes long, `s` from 2 (20 bytes, also what 0 and any other value mean) to 30 (244 bytes). The phone must only ask for a size that fits in what the Bluetooth link allows: `s` such that `4 + 8 x s` is at most the ATT MTU minus 3 (a phone that has negotiated MTU 247 can ask for 30). The Tricorder accepts an MTU up to 247 (`ATT.setMaxMtu(247)`); the Android app asks for it with `requestMtu(247)` when it starts listening and reads the granted value in `onMtuChanged`. The Tricorder
  opens its LIVE AUDIO screen (screen id 18) and sends packets on the Audio characteristic until the phone says stop, LEFT is
  pressed on the Tricorder, the phone disconnects, or remote control is switched off. The phone cannot press buttons on that screen
  other than LEFT. The phone should ask for a short connection interval while it listens.
- **REBOOT** (command 6) restarts the Tricorder. The command is `06 52 42 54` ("RBT"): a reboot command that does not
  carry all three confirmation bytes is ignored, so a stray or malformed write can never restart the unit. The link
  drops; after the restart Bluetooth and remote control are off again until they are switched on at the Tricorder.
  The phone still cannot open the Reboot screen. An app should ask the user before it sends this.
- **IR Receive** can be opened and operated by the phone, because it only listens (cloning transmits nothing). All five
  buttons are accepted there; the IR RECEIVE command (4) starts a clone, opens the data view or returns to the menu
  from the data view or a result. While a clone or a question ("discard the new codes?", "default codes?") or a result
  is on the screen the Tricorder refuses GOTO and IR commands, so a scripted ENTER can never answer a question: answer or
  cancel first (BUTTON commands: RIGHT skips, UP goes back one, LEFT cancels and asks, ENTER = yes, LEFT = no). On the Bluetooth screen UP, DOWN and LEFT are accepted but not ENTER or
  RIGHT, which would switch Bluetooth or remote control off. (The Bluetooth screen is where the user switches
  remote control on, so it is the screen the Tricorder is usually on when the phone starts sending commands.)
- **IR** is the one command that makes the Tricorder transmit, and it can only choose one of the seventeen codes
  on the Tricorder's IR Send screen: 0 PWR, 1 VOL+, 2 VOL-, 3 OK, 4 BACK, 5 EXIT, 6 LEFT, 7 RIGHT, 8 DOWN, 9 UP,
  10 Amazon, 11 YouTube, 12 Eject, 13 Play, 14 Stop, 15 Pause, 16 Menu. Until a remote is cloned (on the Tricorder:
  COMMS > IR Receive > Clone remote) codes 0-11 are the built-in NEC codes (address 0x04) and 12-16 are not assigned;
  after a clone they are the recorded codes, and any button that was skipped is not assigned. The Device packet's
  IR mask (byte 16) says which codes exist. The Tricorder opens the IR Send screen first if it is not open (about a
  second), then selects the option and presses ENTER, so exactly the same code runs as when it is done by hand. Once
  the screen is open (Device packet, active screen 11) each command sends at once; a phone should wait for that
  before sending quick successive codes. A code number above 16, or a code that is not assigned, is ignored (the Tricorder's Serial Monitor says which).
- A real button press on the Tricorder cancels whatever the phone was doing.
- Commands that are malformed (unknown command, button outside 1-5, press kind above 1) or too short are ignored.
- Send the next command only after the previous write has completed (Android's `onCharacteristicWrite`).
- **Diagnosis.** Every command the Tricorder receives is printed on its Serial Monitor with the reason it was accepted or
  ignored (remote control off, bad button, button not allowed on this screen, and so on).

Typical buttons on each screen (what the Tricorder's own buttons do there):

| Screen | Buttons |
|---|---|
| Shot Timer | setup: UP / DOWN countdown + / -, RIGHT / LEFT sensitivity + / -, ENTER start. Countdown: LEFT cancel. Timing: ENTER stop. Results: ENTER new run, LEFT back to setup |
| Stopwatch | ENTER start / stop, RIGHT lap, DOWN reset (while stopped) |
| Countdown | setting: LEFT / RIGHT minutes / seconds, UP / DOWN change, ENTER start. Running: ENTER pause, LEFT cancel. Paused: ENTER resume, LEFT reset. Time up: any short press |
| dB Meter | UP A-weighting / flat, DOWN Fast / Slow, RIGHT reset max, long UP / long DOWN calibration + / - 1 dB |
| Spectrum | UP / DOWN display gain + / - 5 dB |
| Metal Detector | long ENTER recalibrate the baseline (about 10 seconds) |
| Navigation | long ENTER calibrate the compass (15 seconds, rotate the Tricorder) |

### Active screen ids

| Id | Screen | Id | Screen |
|---|---|---|---|
| 0 | a menu | 9 | Time Zone |
| 1 | Navigation | 10 | Reboot |
| 2 | Metal Detector | 11 | IR Send |
| 3 | EMF Meter | 12 | IR Receive |
| 4 | Atmospheric | 13 | Shot Timer |
| 5 | dB Meter | 14 | Stopwatch |
| 6 | Spectrum | 15 | Countdown |
| 7 | Battery | 16 | Bluetooth |
| 8 | System Info | 17 | Declination |
| | | 18 | Live Audio |
| | | 19 | Seismometer |
| | | 255 | anything else |

## Example packets (for checking what the phone shows)

These are real outputs from the firmware's packing code for the inputs shown.

**Environment**: `29 09 1f 10 cd 8b 01 00 40 e2 01 00`

- temperature `0x0929` = 2345 -> **23.45 degC**
- humidity `0x101f` = 4127 -> **41.27 %**
- pressure `0x00018bcd` = 101325 Pa -> **1013.25 hPa**
- uptime `0x0001e240` = 123456 ms

**Position**: `b9 e7 d0 15  7a d1 52 cc  2e 49 00 00  7b 00  00 00  09  0c  02  0f`

- latitude `0x15d0e7b9` = 366012345 -> **36.6012345**
- longitude `0xcc52d17a` (signed) = -866987654 -> **-86.6987654**
- altitude `0x0000492e` = 18734 cm -> **187.34 m**
- speed `0x007b` = 123 cm/s -> **1.23 m/s**
- course `0x0000` -> **0.0 degrees** (359.97 rounds to 360.0, which wraps to 0.0)
- satellites 9, HDOP `0x0c` = 12 -> **1.2**, fix age 2 s, flags `0x0f` (all valid)

**IR Receive**: `02 19 01 00 01 00 ff 01 20 04 fb 08 f7 43 00 01 01 00 00`

- screen 2 (clone), live; flags `0x19` = live + flash store + a frame received; prompt 1 (Vol Up is next)
- note 1 (got) about button 0 (Power); no duplicate (255); protocol 1 (NEC), 32 bits
- value `0xf708fb04` (address 0x04, command 0x08), 67 pulses, counter 1, progress mask `0x000001` (Power recorded)

**Device**: `ac 0f 57 05 00 5e d0 b2 20 bf 02 00 08 00 58 02 ff 0f 00 01`

- battery `0x0fac` = 4012 mV, 87 %, screen id 5 (dB Meter)
- uptime `0xb2d05e00` = 3,000,000,000 ms, free block `0x0002bf20` = 180000 bytes
- protocol version 9, flags `0x00` (remote control off, time zone not saved), firmware `0x0258` = 600 -> **0.6**
- IR mask `0x000fff` -> codes 0-11 exist, 12-16 (Eject, Play, Stop, Pause, Menu) do not
- time zone `0x01` -> **Central**

**Motion**: `d2 04  e0 ff  2f 00  e3 01  0f  01  58 01`

- heading `0x04d2` = 1234 -> **123.4 degrees**
- pitch `0xffe0` = -32 -> **-3.2 degrees**, roll `0x002f` = 47 -> **4.7 degrees**
- field `0x01e3` = 483 -> **48.3 uT**
- flags `0x0f`: held level, valid, calibrated, live
- flags 2 `0x01`: the declination is saved; declination `0x0158` = 344 -> **+3.44 degrees** (east)

**Sound**: `d3 02  aa 03  67 02  19 04  0d  00`

- level `0x02d3` = 723 -> **72.3 dB**, max `0x03aa` = 938 -> **93.8 dB**
- Leq `0x0267` = 615 -> **61.5 dB**, peak `0x0419` = 1049 -> **104.9 dB**
- flags `0x0d`: A-weighted, Fast, live, valid

**Seismometer**: `03 05 41 01  7b 00  c8 01  03 00  7d 00`

- flags `0x03` live and valid, sensitivity 5 (100 mg full scale), shaking now `0x0141` = 321 -> **3.21 mg**, strongest since the last packet `0x007b` = 123 -> **12.3 mg**, since reset `0x01c8` = 456 -> **45.6 mg**, 3 events, noise floor `0x007d` = 125 -> **1.25 mg**

**Spectrum**: `00 46 47 5f 7d fe 00 00 04 0e 19 22 2c 36 41 4a  e8 03  47  03`

- band levels 0, 70, 71, 95, 125, 254, 0, 0, 4, 14, 25, 34, 44, 54, 65, 74 dB
- strongest frequency `0x03e8` = **1000 Hz** at `0x47` = **71 dB**
- flags `0x03`: live, valid

**Fields**: `e3 01  25 00  be 01  09 02  13  00  14  c8`

- metal detector field `0x01e3` = 483 -> **48.3 uT**, deviation `0x0025` = 37 -> **3.7 uT**, baseline `0x01be` = 446 -> **44.6 uT**
- EMF meter `0x0209` = 521 -> **52.1 uT**
- flags `0x13`: metal live, metal valid, EMF valid (no metal detected, EMF screen not open)
- threshold `0x14` = 20 uT, EMF full scale `0xc8` = 200 uT

**Shot**: `02 05 04 00 03 06  4e 61 bc 00`

- mode 2 (timing), countdown setting 5 s, sensitivity 4, 3 shots registered, flags `0x06`: live, valid
- time since the start beep `0x00bc614e` = 12345678 us -> **12.35 s**

**List**: `01 00 06  50 d4 12 00  a8 9b 20 00  00 6f 2c 00  e4 23 43 00`

- kind 1 (shot times), first entry 0, 6 entries in the list
- entries 0-3: 1234000, 2137000, 2912000, 4400100 us -> **1.23, 2.14, 2.91, 4.40 s**

**Stopwatch**: `3c 44 01 00  01  03  03  00`

- elapsed `0x0001443c` = 83004 ms -> **01:23.00**, running, 3 laps, flags `0x03`: live, valid

**Countdown**: `3c f0 00 00  2c 01  01  03`

- time left `0x0000f03c` = 61500 ms -> **01:02** (rounded up), set time `0x012c` = 300 s -> **05:00**, running, live, valid

**Commands** (all 4 bytes, written to the Command characteristic)

- `01 05 01 00`: BUTTON 5 (LEFT), long press = go back one level
- `01 01 00 00`: BUTTON 1 (ENTER), short press
- `02 0d 00 00`: GOTO screen 13 (Shot Timer)
- `02 00 00 00`: GOTO the main menu
- `03 0a 00 00`: IR, code 10 (Amazon)
- `03 01 00 00`: IR, code 1 (VOL+)
- `02 13 00 00`: GOTO screen 19 (Seismometer)

**Device** with remote control on: `ac 0f 57 05  00 5e d0 b2  20 bf 02 00  08  01  58 02  ff 0f 00  01` (byte 13 = `01`).

## Testing with the nRF Connect app

1. Install **nRF Connect for Mobile** (Nordic Semiconductor) from the Play Store.
2. On the Tricorder: COMMS menu > Bluetooth > ENTER. The screen shows "Advertising".
3. In nRF Connect: **SCAN**, find **Tricorder**, tap **CONNECT**. (Android asks for the "Nearby devices"
   permission the first time.) The Tricorder screen changes to "Connected".
4. Open the **Unknown Service** (`3F5A0000-...`). Tap the triple-arrow (notify) icon on each of the eleven
   characteristics. The first three change about once a second; the others change fast while their screen is
   open on the Tricorder.
5. Decode the bytes with the tables above. Change screens on the Tricorder and watch the Device packet's byte 3
   change.
6. Disconnect in the app, then connect again, to confirm the Tricorder re-advertises.


## Audio (protocol v10, larger packets in v12)
Characteristic `3f5a000e-7c2b-4e6d-a1b3-5d8e9c0f1a20`, notify only, 20 to 244 bytes (it is a variable-length characteristic), while
streaming (nothing otherwise). The sound is the Tricorder's microphone, mono, IMA ADPCM at 4 bits a sample, at the rate the phone
asked for: 8 kHz, 5.33 kHz, 4 kHz or 16 kHz. The packets do not say which rate or size was asked for; the phone knows (the size is
simply the length of the notification). With the default 23-byte ATT MTU only 20-byte packets fit (250 a second at 8 kHz, more
than many phones' links carry); with MTU 247 a 244-byte packet carries 480 samples (60 ms at 8 kHz) and only about 17 are needed a second.

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | uint8 | sequence number | counts up and wraps at 256; a jump means packets were lost or dropped |
| 1 | int16 | predictor | the decoder's value before the first sample of this packet |
| 3 | uint8 | step index | 0 to 88, the decoder's step index before the first sample |
| 4 | N bytes | 2 N samples | 4 bits each, the low nibble is the earlier sample; N = packet length - 4 (16 to 240) |

Because every packet carries its own predictor and step index, a decoder can start from any packet, and a lost packet only
loses the sound it held (4 ms in a 20-byte packet, 60 ms in a 244-byte one at 8 kHz). Decoding of one sample (standard IMA ADPCM): `step = STEP[index]; vp = step>>3; if (code&4) vp += step;
if (code&2) vp += step>>1; if (code&1) vp += step>>2; predictor += (code&8) ? -vp : vp` (clamped to int16), then
`index += INDEX[code]` (clamped to 0..88), with the usual IMA tables (see `LiveAudio.kt` or `MicStream.ino`).
