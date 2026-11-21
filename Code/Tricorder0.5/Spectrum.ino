// ------------- Audio Spectrum Analyzer ---------------
//
// Uses the same PDM microphone stream as the dB meter (sampleBuffer / samplesRead / onPDMdata).
// 256-point FFT at 16 kHz -> 62.5 Hz per bin, 0-8 kHz. Bins are grouped into 16 log-spaced bands
// and drawn as bars on the colour TFT (with falling peak-hold caps). The OLED shows peak frequency,
// level and gain.
//
// Levels use the same reference as the dB meter (REFERENCE_RMS), so numbers are comparable, but
// remember a spectrum shows the level in ONE band, which is lower than the overall level.
//
// Controls:  UP / DOWN (short) = display gain +/-5 dB     LEFT (long) = exit

#define SPEC_N        256       // FFT size (must be a power of two, and <= sampleBuffer size)
#define SPEC_BANDS    16
#define SPEC_BAR_W    5         // 16 bars x 6 px = 96 px (the TFT width)
#define SPEC_BASE_Y   52        // y of the baseline; bars grow upward from here
#define SPEC_MAX_H    50        // tallest bar in pixels
#define SPEC_GREEN_H  30        // bar rows below this are green, then yellow, then red from SPEC_YELLOW_H
#define SPEC_YELLOW_H 40
#define SPEC_FLOOR_DB 0.0       // level (dB) that maps to an empty bar, before gain
#define SPEC_RANGE_DB 70.0      // dB span from empty to full bar
#define SPEC_SAMPLE_RATE 16000.0
#define SPEC_CAP_HOLD_MS 600    // how long a peak cap sits before it starts to fall

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
int   specBarH[SPEC_BANDS];     // bar heights currently drawn
int   specCapH[SPEC_BANDS];     // peak caps currently drawn
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

// ---- Fill bar rows [from, to) (measured up from the baseline) with the level colours, or erase them ----
void spectrumFillRows(int x, int from, int to, bool erase) {
  int a = from;
  while (a < to) {
    int zoneEnd;
    uint16_t color;
    if (a < SPEC_GREEN_H)       { zoneEnd = SPEC_GREEN_H;  color = GREEN; }
    else if (a < SPEC_YELLOW_H) { zoneEnd = SPEC_YELLOW_H; color = YELLOW; }
    else                        { zoneEnd = SPEC_MAX_H;    color = RED; }
    int b = (zoneEnd < to) ? zoneEnd : to;
    tft.fillRect(x, SPEC_BASE_Y - b, SPEC_BAR_W, b - a, erase ? BLACK : color);
    a = b;
  }
}

// Static parts of the TFT screen, drawn once on entry
void spectrumDrawStatic() {
  tft.fillScreen(BLACK);
  tft.drawFastHLine(0, SPEC_BASE_Y, 96, BLUE2);
  tft.setTextSize(1);
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(18, 56); tft.print("250");   // band 3 starts at 250 Hz
  tft.setCursor(48, 56); tft.print("1k");     // band 8 starts at 1 kHz
  tft.setCursor(72, 56); tft.print("4k");     // band 12 (3.06-4.06 kHz) contains 4 kHz
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
    currentState = MET_MENU;
    return;
  }
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2 && specGainDb < 30)  { specGainDb += 5; lastOledMs = 0; }   // Up: more gain
    if (lastButton == 4 && specGainDb > -30) { specGainDb -= 5; lastOledMs = 0; }   // Down: less gain
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
      int h = (int)((dB + specGainDb - SPEC_FLOOR_DB) * SPEC_MAX_H / SPEC_RANGE_DB);
      if (h < 0) h = 0;
      if (h > SPEC_MAX_H) h = SPEC_MAX_H;

      // Peak cap: jumps up with the bar, holds, then falls one pixel per frame
      int cap = specCapH[b];
      if (h >= cap) {
        cap = h;
        specCapTime[b] = now;
      } else if (now - specCapTime[b] > SPEC_CAP_HOLD_MS && cap > 0) {
        cap--;
        if (cap < h) cap = h;
      }

      // --- Draw only what changed ---
      int x = b * 6;
      if (cap != specCapH[b] || h != specBarH[b]) {
        // erase the old cap row (it sits just above the old cap height)
        tft.drawFastHLine(x, SPEC_BASE_Y - specCapH[b] - 1, SPEC_BAR_W, BLACK);
        // grow or shrink the bar
        if (h > specBarH[b])      spectrumFillRows(x, specBarH[b], h, false);
        else if (h < specBarH[b]) spectrumFillRows(x, h, specBarH[b], true);
        // new cap
        if (cap > 0) tft.drawFastHLine(x, SPEC_BASE_Y - cap - 1, SPEC_BAR_W, WHITE);
        specBarH[b] = h;
        specCapH[b] = cap;
      }
    }
  }

  // --- OLED text, 4 times a second ---
  if (millis() - lastOledMs >= 250) {
    lastOledMs = millis();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 9);
    display.println(F("Spectrum"));

    display.setCursor(0, 20);
    display.print(F("Peak: "));
    if (peakDb > 10.0) {                    // don't report a "peak" that is just noise floor
      display.print((int)(peakFreq + 0.5));
      display.println(F(" Hz"));
    } else {
      display.println(F("---"));
    }

    display.setCursor(0, 30);
    display.print(F("Level: "));
    display.print(peakDb, 1);
    display.println(F(" dB"));

    display.setCursor(0, 40);
    display.print(F("Gain: "));
    if (specGainDb > 0) display.print(F("+"));
    display.print(specGainDb);
    display.println(F(" dB  (U/D)"));

    display.setCursor(0, 55);
    display.print(F("Hold L to Exit"));
    display.display();
  }
}
