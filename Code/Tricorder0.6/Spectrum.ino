// ------------- Audio Spectrum Analyzer ---------------
//
// Uses the same PDM microphone stream as the dB meter (sampleBuffer / samplesRead / onPDMdata).
// 256-point FFT at 16 kHz -> 62.5 Hz per bin, 0-8 kHz. Bins are grouped into 16 log-spaced bands
// and drawn as segmented bars on the colour TFT in the LCARS look (with falling peak-hold caps; see Lcars.ino).
// The OLED shows peak frequency, level and gain.
//
// Levels use the same reference as the dB meter (REFERENCE_RMS), so numbers are comparable, but
// remember a spectrum shows the level in ONE band, which is lower than the overall level.
//
// Controls:  UP / DOWN (short) = display gain +/-5 dB     LEFT (long) = exit

#define SPEC_N        256       // FFT size (must be a power of two, and <= sampleBuffer size)
#define SPEC_BANDS    16
#define SPEC_SEGS      14       // segments per bar: SPEC_RANGE_DB / SPEC_SEGS = 5 dB each
#define SPEC_SEG_H     2        // a segment is 2 pixels high ...
#define SPEC_SEG_PITCH 3        // ... and 3 pixels apart (so there is a 1 pixel gap between segments)
#define SPEC_BAR_W     4        // 16 bars x 5 px = 80 px, centred in the free area of the LCARS screen
#define SPEC_BAR_PITCH 5
#define SPEC_BAR_X0    12
#define SPEC_BASE_Y    51       // y of the bottom pixel row of the lowest segment; bars grow upward from here
#define SPEC_GREEN_SEGS  8      // segments below this are green, then amber below SPEC_YELLOW_SEGS, then red
#define SPEC_YELLOW_SEGS 11
#define SPEC_FLOOR_DB 0.0       // level (dB) that maps to an empty bar, before gain
#define SPEC_RANGE_DB 70.0      // dB span from empty to full bar
#define SPEC_SAMPLE_RATE 16000.0
#define SPEC_CAP_HOLD_MS 600    // how long a peak cap sits before it starts to fall
#define SPEC_CAP_STEP_MS 55     // then it falls one segment this often

// First FFT bin of each band; band i covers bins specEdges[i] .. specEdges[i+1]-1
// (bin k is k * 62.5 Hz).  Edges in Hz: 62 125 187 250 312 437 562 750 1k 1.3k 1.75k 2.3k 3k 4k 5.4k 6.7k 8k
const int specEdges[SPEC_BANDS + 1] = {1, 2, 3, 4, 5, 7, 9, 12, 16, 21, 28, 37, 49, 65, 86, 107, 128};

float specRe[SPEC_N];
float specIm[SPEC_N];
float specWindow[SPEC_N];       // Hann window
float specCos[SPEC_N / 2];      // twiddle factors
float specSin[SPEC_N / 2];
bool  specTablesReady = false;

float specBandPow[SPEC_BANDS];  // smoothed band power (linear)
int   specBarH[SPEC_BANDS];     // bar heights currently drawn (in segments)
int   specCapH[SPEC_BANDS];     // peak caps currently drawn (in segments)
unsigned long specCapTime[SPEC_BANDS];
int   specGainDb = 0;           // display gain, adjusted with UP/DOWN

// ---- one-time tables ----
void spectrumInitTables() {
  for (int i = 0; i < SPEC_N; i++) {
    specWindow[i] = 0.5 - 0.5 * cos(2.0 * PI * i / (SPEC_N - 1));
  }
  for (int k = 0; k < SPEC_N / 2; k++) {
    specCos[k] = cos(2.0 * PI * k / SPEC_N);
    specSin[k] = sin(2.0 * PI * k / SPEC_N);
  }
  specTablesReady = true;
}

// ---- in-place radix-2 FFT of specRe/specIm ----
void spectrumFFT() {
  const int n = SPEC_N;

  // bit-reversal permutation
  for (int i = 1, j = 0; i < n; i++) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float t = specRe[i]; specRe[i] = specRe[j]; specRe[j] = t;
      t = specIm[i]; specIm[i] = specIm[j]; specIm[j] = t;
    }
  }

  // butterflies
  for (int len = 2; len <= n; len <<= 1) {
    int half = len >> 1;
    int step = n / len;
    for (int i = 0; i < n; i += len) {
      for (int k = 0; k < half; k++) {
        float wr = specCos[k * step];
        float wi = -specSin[k * step];
        int a = i + k;
        int b = a + half;
        float tr = specRe[b] * wr - specIm[b] * wi;
        float ti = specRe[b] * wi + specIm[b] * wr;
        specRe[b] = specRe[a] - tr;
        specIm[b] = specIm[a] - ti;
        specRe[a] += tr;
        specIm[a] += ti;
      }
    }
  }
}

// ---- the bars on the TFT: SPEC_SEGS segments each, from the baseline up ----

// A segment's colour follows its height: green, then amber, then red
uint16_t spectrumSegColor(int i) {
  if (i < SPEC_GREEN_SEGS) return lcOk();
  if (i < SPEC_YELLOW_SEGS) return lcGold();
  return lcAlert();
}

// What segment i of a bar is, for a bar of h segments with its peak cap at 'cap': 0 = dark, 1 = lit, 2 = the cap.
// The cap marks the top segment of the peak (so it is the bar's own top segment while the bar is at its peak).
int spectrumSegState(int i, int h, int cap) {
  if (cap > 0 && i == cap - 1) return 2;
  return (i < h) ? 1 : 0;
}

void spectrumDrawSegment(int band, int i, int state) {
  int x = SPEC_BAR_X0 + band * SPEC_BAR_PITCH;
  int y = SPEC_BASE_Y - i * SPEC_SEG_PITCH - (SPEC_SEG_H - 1);
  if (state == 2) tft.fillRect(x, y, SPEC_BAR_W, SPEC_SEG_H, lcPeach());
  else            lcSeg(x, y, SPEC_BAR_W, SPEC_SEG_H, spectrumSegColor(i), state == 1);
}

// Bring one bar from (oldH, oldCap) to (h, cap): only the segments that change are redrawn
void spectrumUpdateBand(int band, int oldH, int oldCap, int h, int cap) {
  for (int i = 0; i < SPEC_SEGS; i++) {
    int before = spectrumSegState(i, oldH, oldCap);
    int after = spectrumSegState(i, h, cap);
    if (before != after) spectrumDrawSegment(band, i, after);
  }
}

// The header's tag block shows the display gain, e.g. "G+10"
void spectrumDrawTag() {
  char buf[8];
  snprintf(buf, sizeof(buf), "G%+d", specGainDb);
  lcSetTag(buf);
}

// Static parts of the TFT screen, drawn once on entry
void spectrumDrawStatic() {
  lcFrame("SPECTRUM", true);
  spectrumDrawTag();
  for (int b = 0; b < SPEC_BANDS; b++) {
    for (int i = 0; i < SPEC_SEGS; i++) spectrumDrawSegment(b, i, 0);
  }
  tft.fillRoundRect(8, 53, 88, 2, 1, lcOrange());                 // the rail under the bars
  tft.setTextSize(1);
  tft.setTextColor(lcGrey(), BLACK);
  tft.setCursor(SPEC_BAR_X0 + 3 * SPEC_BAR_PITCH, 56);  tft.print("250");   // band 3 starts at 250 Hz
  tft.setCursor(SPEC_BAR_X0 + 8 * SPEC_BAR_PITCH, 56);  tft.print("1k");    // band 8 starts at 1 kHz
  tft.setCursor(SPEC_BAR_X0 + 12 * SPEC_BAR_PITCH, 56); tft.print("4k");    // band 12 (3.06-4.06 kHz) contains 4 kHz
}

void SpectrumAnalyzer(int pressType) {
  static bool initialized = false;
  static unsigned long lastOledMs = 0;
  static float peakFreq = 0.0;
  static float peakDb = 0.0;
  static short local[SPEC_N];

  // --- Buttons ---
  if (pressType == LONG_PRESS && lastButton == 5) {   // Long Left: exit
    tft.fillScreen(BLACK);
    initialized = false;
    currentState = ENV_MENU;
    return;
  }
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2 && specGainDb < 30)  { specGainDb += 5; lastOledMs = 0; }   // Up: more gain
    if (lastButton == 4 && specGainDb > -30) { specGainDb -= 5; lastOledMs = 0; }   // Down: less gain
    if (initialized) spectrumDrawTag();                                             // the header shows the gain
  }

  // --- First pass after entering ---
  if (!initialized) {
    initialized = true;
    if (!specTablesReady) spectrumInitTables();
    for (int i = 0; i < SPEC_BANDS; i++) {
      specBandPow[i] = 0.0;
      specBarH[i] = 0;
      specCapH[i] = 0;
      specCapTime[i] = 0;
    }
    peakFreq = 0.0;
    peakDb = 0.0;
    lastOledMs = 0;
    spectrumDrawStatic();
  }

  // --- New audio? Copy it out quickly (the PDM callback overwrites sampleBuffer) ---
  if (samplesRead > 0) {
    int n = samplesRead;
    if (n > SPEC_N) n = SPEC_N;
    memcpy(local, sampleBuffer, n * sizeof(short));
    samplesRead = 0;
    for (int i = n; i < SPEC_N; i++) local[i] = 0;

    // remove DC offset, normalise to +/-1.0, apply window
    float mean = 0.0;
    for (int i = 0; i < SPEC_N; i++) mean += local[i];
    mean /= SPEC_N;
    for (int i = 0; i < SPEC_N; i++) {
      specRe[i] = ((local[i] - mean) / 32768.0) * specWindow[i];
      specIm[i] = 0.0;
    }

    spectrumFFT();

    // Bin power as RMS^2 of the tone in that bin. A sine of amplitude A gives |X| = A*N/4 with a Hann window.
    const float ampScale = 4.0 / SPEC_N;
    float binPow[SPEC_N / 2];
    float maxPow = 0.0;
    int maxBin = 1;
    for (int k = 1; k < SPEC_N / 2; k++) {
      float mag = sqrt(specRe[k] * specRe[k] + specIm[k] * specIm[k]) * ampScale;
      float p = mag * mag * 0.5;               // (A/sqrt2)^2
      binPow[k] = p;
      if (p > maxPow) { maxPow = p; maxBin = k; }
    }
    peakFreq = maxBin * (SPEC_SAMPLE_RATE / SPEC_N);
    peakDb = 10.0 * log10(maxPow + 1e-20) - 20.0 * log10(REFERENCE_RMS);
    telSpecPeakHz = peakFreq;                      // published for Bluetooth (see Bluetooth.ino)
    telSpecPeakPow = maxPow;
    telSpecMs = millis();

    // --- Bands: loudest bin in each band, smoothed (fast attack, slower release) ---
    unsigned long now = millis();
    for (int b = 0; b < SPEC_BANDS; b++) {
      float bp = 0.0;
      for (int k = specEdges[b]; k < specEdges[b + 1]; k++) {
        if (binPow[k] > bp) bp = binPow[k];
      }
      float rate = (bp > specBandPow[b]) ? 0.7 : 0.15;
      specBandPow[b] += rate * (bp - specBandPow[b]);

      float dB = 10.0 * log10(specBandPow[b] + 1e-20) - 20.0 * log10(REFERENCE_RMS);
      int h = (int)((dB + specGainDb - SPEC_FLOOR_DB) * SPEC_SEGS / SPEC_RANGE_DB + 0.5);
      if (h < 0) h = 0;
      if (h > SPEC_SEGS) h = SPEC_SEGS;

      // Peak cap: jumps up with the bar, holds, then falls one segment every SPEC_CAP_STEP_MS
      int cap = specCapH[b];
      if (h >= cap) {
        cap = h;
        specCapTime[b] = now;
      } else if (now - specCapTime[b] > SPEC_CAP_HOLD_MS && cap > 0) {
        cap--;
        if (cap < h) cap = h;
        specCapTime[b] = now - SPEC_CAP_HOLD_MS + SPEC_CAP_STEP_MS;       // the next step comes after SPEC_CAP_STEP_MS
      }

      // --- Draw only what changed ---
      if (cap != specCapH[b] || h != specBarH[b]) {
        spectrumUpdateBand(b, specBarH[b], specCapH[b], h, cap);
        specBarH[b] = h;
        specCapH[b] = cap;
      }
    }
  }

  // --- OLED text, 4 times a second ---
  if (millis() - lastOledMs >= 250) {
    lastOledMs = millis();
    // the OLED in the LCARS look (see LcarsOled.ino): peak frequency, level and gain
    display.clearDisplay();
    lcoFrame("SPECTRUM", NULL);
    char v[16];
    if (peakDb > 10.0) snprintf(v, sizeof(v), "%d Hz", (int)(peakFreq + 0.5));       // not a "peak" that is just noise floor
    else               snprintf(v, sizeof(v), "---");
    lcoRow(0, "PEAK", v);
    snprintf(v, sizeof(v), "%.1f dB", peakDb);
    lcoRow(1, "LEVEL", v);
    if (specGainDb > 0) snprintf(v, sizeof(v), "+%d dB", specGainDb);
    else                snprintf(v, sizeof(v), "%d dB", specGainDb);
    lcoRow(2, "GAIN", v);
    lcoText(3, "UP/DN = GAIN");
    lcoText(4, "HOLD LT = EXIT");
    display.display();
  }
}
