// ------------- Live microphone stream to the phone ---------------
//
// The phone's Live Audio screen asks for this (Remote.ino, command 8). The Tricorder opens its "LIVE AUDIO" screen, takes the
// PDM microphone that is already running (16 kHz), and sends it over Bluetooth as small packets:
//   16 kHz microphone -> DC blocker -> low-pass and divide the rate by 2, 3 or 4 -> IMA ADPCM, 4 bits a sample
//   -> packets on the Audio characteristic. The phone chooses the quality and the packet size (protocol v12):
//        quality 0: 8 kHz    1: 5.33 kHz    2: 4 kHz    3: 16 kHz (the microphone as it is, about 33 packets a second of 244 bytes)
//        packet size: 20 bytes (the Bluetooth default, 250 packets a second at 8 kHz) up to 244 bytes (about 17 a second), whatever
//        the phone and the Tricorder agreed on as the largest Bluetooth packet (ATT MTU, see ATT.setMaxMtu in Bluetooth.ino).
//   A Bluetooth link that cannot carry that many packets a second loses some: bigger packets fix that.
// Each packet starts with the encoder's state, so a lost packet costs only its own sound (4 ms in a 20-byte packet, 60 ms in 244).
//
//   Packet (4 + 8 x size bytes, 20 to 244):  0 sequence number (counts up, wraps at 256)   1-2 predictor (int16, little endian)
//                       3 step index (0-88)    4-end: 2 samples a byte (4 bits each), low nibble first
//
// If the link cannot keep up, the oldest packets are thrown away (the sound jumps ahead) rather than letting the delay grow.
// The stream stops by itself when LEFT is pressed, the phone says stop, the phone disconnects or remote control is switched off.
//
// Needs, in the main sketch:   State enum: AUDIO_STREAM     loop(): case AUDIO_STREAM: AudioStreamScreen(pressType); break;

#define MIC_RING_FRAMES      24       // 24 x 16 ms = 384 ms of microphone buffering while the link is busy
#define MIC_FRAME_MAX        256
#define MIC_PKT_MAX          244      // the largest packet: a 247-byte Bluetooth packet less 3 bytes of header
#define MIC_PKT_QUEUE        40       // packet slots (about 10 KB); micQueueLimit of them are used, so the queue holds about 300 ms
#define MIC_QUEUE_MS         300
#define MIC_SEND_PER_PASS    8        // packets handed to Bluetooth per pass of loop(), so the buttons stay responsive
#define MIC_GAIN             1.0f     // digital gain on top of PDM.setGain() in setup()

// ----------------------- microphone frames (filled by the PDM callback in Calibration.ino) -----------------------
short micRing[MIC_RING_FRAMES][MIC_FRAME_MAX];
volatile int micRingLen[MIC_RING_FRAMES];
volatile uint8_t micRingHead = 0;                 // advanced only by the PDM callback
volatile uint8_t micRingTail = 0;                 // advanced only by micStreamPump()
volatile bool micStreamOn = false;                // frames are only queued while streaming
volatile unsigned long micRingDropped = 0;

// Called from onPDMdata() (interrupt context): just copies the frame into the queue
void micStreamPush(const short* data, int n) {
  if (!micStreamOn) return;
  uint8_t next = (micRingHead + 1) % MIC_RING_FRAMES;
  if (next == micRingTail) {                      // queue full: drop this frame
    micRingDropped++;
    return;
  }
  if (n > MIC_FRAME_MAX) n = MIC_FRAME_MAX;
  memcpy(micRing[micRingHead], data, n * sizeof(short));
  micRingLen[micRingHead] = n;
  micRingHead = next;
}

// ----------------------- IMA ADPCM encoder (no hardware code in this block) -----------------------
// ---- ADPCM BEGIN ----
static const int16_t micStepTable[89] = {
  7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130,
  143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282,
  1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630,
  9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};
static const int8_t micIndexTable[16] = {-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8};

int micDecim = 2;                                 // 16 kHz divided by this: 1 = 16 kHz, 2 = 8 kHz, 3 = 5.33 kHz, 4 = 4 kHz

int micPred = 0;                                  // encoder state: the value the decoder will have reached
int micIdx = 0;                                   // and its step index

// Encode one sample into a 4-bit code and move the encoder state on exactly as the decoder will
uint8_t micEncodeSample(int sample) {
  int step = micStepTable[micIdx];
  int diff = sample - micPred;
  uint8_t code = 0;
  if (diff < 0) { code = 8; diff = -diff; }
  int vp = step >> 3;
  if (diff >= step) { code |= 4; diff -= step; vp += step; }
  step >>= 1;
  if (diff >= step) { code |= 2; diff -= step; vp += step; }
  step >>= 1;
  if (diff >= step) { code |= 1; vp += step; }
  micPred += (code & 8) ? -vp : vp;
  if (micPred > 32767) micPred = 32767;
  if (micPred < -32768) micPred = -32768;
  micIdx += micIndexTable[code];
  if (micIdx < 0) micIdx = 0;
  if (micIdx > 88) micIdx = 88;
  return code;
}
// ---- ADPCM END ----

// ----------------------- 16 kHz -> 8 kHz, samples -> packets -----------------------
float micHpX = 0.0f, micHpY = 0.0f;               // DC blocker (the PDM microphone has an offset)
float micH1 = 0.0f, micH2 = 0.0f;                 // low-pass history (divide by 2)
bool  micPhase = false;                           // every second input sample becomes an output sample (divide by 2)
float micAcc = 0.0f;                              // running sum of the samples being averaged (divide by 3 or 4)
int   micAccN = 0;
int   micPktLen = 20;                             // bytes in a packet, set by the phone: 4 + 8 x size, 20 to 244
int   micSamplesPerPkt = 32;                      // 2 samples a byte after the 4-byte header
int   micQueueLimit = MIC_PKT_QUEUE - 2;          // most packets kept waiting for the link (about MIC_QUEUE_MS)
short micFifo[2 * (MIC_PKT_MAX - 4) + 16];        // samples waiting to fill a packet
int   micFifoN = 0;
uint8_t micSeq = 0;
uint8_t micPkt[MIC_PKT_QUEUE][MIC_PKT_MAX];       // finished packets waiting for the link
int   micPktHead = 0;                             // next free slot
int   micPktTail = 0;                             // next packet to send
unsigned long micPktMade = 0;
unsigned long micPktSent = 0;
unsigned long micPktDropped = 0;                  // thrown away because the link was too slow

int micPktQueued() { return (micPktHead - micPktTail + MIC_PKT_QUEUE) % MIC_PKT_QUEUE; }

// Make one packet from the samples in the FIFO (micSamplesPerPkt of them)
void micMakePacket() {
  uint8_t* p = micPkt[micPktHead];
  p[0] = micSeq++;
  p[1] = (uint8_t)(micPred & 0xFF);
  p[2] = (uint8_t)((micPred >> 8) & 0xFF);
  p[3] = (uint8_t)micIdx;
  for (int i = 0; i < micSamplesPerPkt; i += 2) {
    uint8_t lo = micEncodeSample(micFifo[i]);
    uint8_t hi = micEncodeSample(micFifo[i + 1]);
    p[4 + i / 2] = (uint8_t)(lo | (hi << 4));
  }
  micFifoN -= micSamplesPerPkt;
  for (int i = 0; i < micFifoN; i++) micFifo[i] = micFifo[i + micSamplesPerPkt];
  micPktMade++;
  micPktHead = (micPktHead + 1) % MIC_PKT_QUEUE;
  while (micPktQueued() > micQueueLimit) {        // queue full: the oldest packet goes
    micPktTail = (micPktTail + 1) % MIC_PKT_QUEUE;
    micPktDropped++;
  }
}

void micProcessSample(short s) {
  float x = (float)s;
  float y = x - micHpX + 0.995f * micHpY;         // DC blocker
  micHpX = x;
  micHpY = y;
  float lp;
  if (micDecim == 1) {
    lp = y;                                       // 16 kHz: the microphone's own rate, nothing to remove
  } else if (micDecim == 2) {
    lp = (y + 2.0f * micH1 + micH2) * 0.25f;      // [1 2 1] / 4: keeps most of what is below 4 kHz, softens what would alias
    micH2 = micH1;
    micH1 = y;
    micPhase = !micPhase;
    if (!micPhase) return;
  } else {
    micAcc += y;                                  // the average of 3 or 4 samples is the low-pass for the lower rates
    if (++micAccN < micDecim) return;
    lp = micAcc / (float)micDecim;
    micAcc = 0.0f;
    micAccN = 0;
  }
  lp *= MIC_GAIN;
  if (lp > 32767.0f) lp = 32767.0f;
  if (lp < -32768.0f) lp = -32768.0f;
  micFifo[micFifoN++] = (short)lp;
  if (micFifoN >= micSamplesPerPkt) micMakePacket();
}

void micStreamReset() {
  micRingHead = 0;
  micRingTail = 0;
  micRingDropped = 0;
  micHpX = micHpY = micH1 = micH2 = 0.0f;
  micPhase = false;
  micAcc = 0.0f;
  micAccN = 0;
  micFifoN = 0;
  micPred = 0;
  micIdx = 0;
  micSeq = 0;
  micPktHead = micPktTail = 0;
  micPktMade = micPktSent = micPktDropped = 0;
  // the packet queue holds about MIC_QUEUE_MS of sound, whatever the packet size
  long fs = (micDecim == 1) ? 16000 : (micDecim == 2) ? 8000 : (micDecim == 3) ? 5333 : 4000;
  long packetMs = (long)micSamplesPerPkt * 1000L / fs;
  long limit = MIC_QUEUE_MS / (packetMs > 0 ? packetMs : 1);
  if (limit < 3) limit = 3;
  if (limit > MIC_PKT_QUEUE - 2) limit = MIC_PKT_QUEUE - 2;     // one slot always stays empty (full looks like empty), and one more while a packet is added
  micQueueLimit = (int)limit;
}

// Turn the microphone frames into packets and hand a few packets to Bluetooth. Called on every pass of loop() while streaming.
void micStreamPump() {
  while (micRingTail != micRingHead) {
    int t = micRingTail;
    int n = micRingLen[t];
    for (int i = 0; i < n; i++) micProcessSample(micRing[t][i]);
    micRingTail = (t + 1) % MIC_RING_FRAMES;
  }
  int sent = 0;
  while (micPktTail != micPktHead && sent < MIC_SEND_PER_PASS) {
    if (!bleSendAudioPacket(micPkt[micPktTail], micPktLen)) break;       // not connected, or Bluetooth is off
    micPktTail = (micPktTail + 1) % MIC_PKT_QUEUE;
    micPktSent++;
    sent++;
  }
}

// ----------------------- the screen -----------------------
bool micStopRequested = false;                    // the phone said stop

bool micStreamActive() { return micStreamOn; }

// The phone asked to start (Remote.ino) at this quality (0 = 8 kHz, 1 = 5.33 kHz, 2 = 4 kHz, 3 = 16 kHz) with packets of 4 + 8 x size8 bytes
// (size8 2 to 30, so 20 to 244 bytes; anything else means 20): open the screen
void micStreamStartFromPhone(int quality, int size8) {
  if (currentState == AUDIO_STREAM) return;
  micDecim = (quality == 1) ? 3 : (quality == 2) ? 4 : (quality == 3) ? 1 : 2;
  if (size8 < 2 || size8 > 30) size8 = 2;
  micPktLen = 4 + 8 * size8;
  micSamplesPerPkt = 2 * (micPktLen - 4);
  tft.fillScreen(BLACK);
  currentState = AUDIO_STREAM;
}

// The phone asked to stop (Remote.ino): the screen closes itself on its next pass
void micStreamStopFromPhone() {
  if (currentState == AUDIO_STREAM) micStopRequested = true;
}

void micDrawTft() {
  lcFrame("MIC LIVE", false);
  lcBigPill(8, 14, 88, 12, lcAlert(), "STREAMING", 1);
  lcBigPill(8, 30, 88, 12, lcBlue(), "TO PHONE", 1);
  lcBigPill(8, 46, 88, 14, lcGold(), "LEFT = STOP", 1);
}

void micDrawOled() {
  char buf[16];
  display.clearDisplay();
  lcoFrame("LIVE AUDIO", "ON AIR");
  snprintf(buf, sizeof(buf), "%lu", micPktSent);
  lcoRow(0, "SENT", buf);
  snprintf(buf, sizeof(buf), "%lu", micPktDropped + micRingDropped);
  lcoRow(1, "LOST", buf);
  snprintf(buf, sizeof(buf), "%s %dB", micDecim == 1 ? "16 KHZ" : micDecim == 2 ? "8 KHZ" : micDecim == 3 ? "5.3 KHZ" : "4 KHZ", micPktLen);
  lcoTextAt(9, 40, buf, 19);
  lcoTextAt(9, 54, "LEFT = STOP", 19);
  display.display();
}

void AudioStreamScreen(int pressType) {
  static bool started = false;
  static unsigned long lastUiMs = 0;

  bool stop = (pressType == SHORT_PRESS || pressType == LONG_PRESS) && lastButton == 5;
  if (started && (micStopRequested || !bleConnected || !remoteIsOn())) stop = true;

  if (stop) {
    micStreamOn = false;
    started = false;
    micStopRequested = false;
    Serial.print(F("Live audio stopped: "));
    Serial.print(micPktSent);
    Serial.print(F(" packets sent, "));
    Serial.print(micPktDropped);
    Serial.println(F(" dropped"));
    tft.fillScreen(BLACK);
    menuDirty = true;
    currentState = MAIN_MENU;
    return;
  }

  if (!started) {
    started = true;
    micStopRequested = false;
    micStreamReset();
    micStreamOn = true;                            // the PDM callback starts queueing frames
    micDrawTft();
    micDrawOled();
    lastUiMs = millis();
    Serial.println(F("Live audio started"));
  }

  micStreamPump();

  if (millis() - lastUiMs >= 500) {
    lastUiMs = millis();
    micDrawOled();
  }
}
