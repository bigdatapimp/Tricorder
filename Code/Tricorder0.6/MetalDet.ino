// ------------- Metal Detector code ----------
//
// The colour screen (LCARS look, see Lcars.ino) shows the deviation from the calibrated baseline in big digits, a
// 20-segment bar (10 uT per segment, 0 to 200 uT) that turns red at DETECTION_THRESHOLD, and METAL DETECTED / NO METAL.
// The OLED shows the title, the field, the deviation and the same verdict.
// Hold UP to calibrate the baseline (keep away from metal).
#define METAL_UPDATE_MS 200    // the screen updates 5 times a second
#define METAL_SEGS      20     // bar segments
#define METAL_SEG_UT    10.0   // uT per segment: the bar spans 0 to 200 uT
#define METAL_BAR_X     12
#define METAL_BAR_Y     27
#define METAL_BAR_PITCH 4
#define METAL_BAR_W     3
#define METAL_BAR_H     9

// The first segment that belongs to the "metal detected" part of the bar (2 for a threshold of 20 uT)
int metalRedFrom() {
  return (int)(DETECTION_THRESHOLD / METAL_SEG_UT);
}

// How many segments are lit for a deviation: one per started 10 uT. It always agrees with the verdict: more than
// metalRedFrom() segments are lit exactly when metal is detected, whatever rounding the floating point does.
int metalLitSegments(float deviation, bool detected) {
  int redFrom = metalRedFrom();
  int n = 0;
  if (deviation > 0.0) n = (int)ceilf(deviation / METAL_SEG_UT);          // NaN gives 0 here
  if (n > METAL_SEGS) n = METAL_SEGS;
  if (detected && n <= redFrom) n = redFrom + 1;
  if (!detected && n > redFrom) n = redFrom;
  if (n > METAL_SEGS) n = METAL_SEGS;
  return n;
}

void metalDrawSegment(int i, bool lit) {
  lcSeg(METAL_BAR_X + i * METAL_BAR_PITCH, METAL_BAR_Y, METAL_BAR_W, METAL_BAR_H, (i < metalRedFrom()) ? lcBlue() : lcAlert(), lit);
}

// The METAL DETECTED / NO METAL block under the bar
void metalDrawVerdict(bool detected) {
  if (detected) lcPill(8, 48, 88, lcAlert(), "METAL DETECTED");
  else          lcPill(8, 48, 88, lcBlue(), "NO METAL");
}

// The parts of the colour screen that do not change: the frame, the unit, the scale with the threshold tick, a dark bar
void metalDrawStatic() {
  lcFrame("METAL DET", false);
  tft.setTextSize(2);
  tft.setTextColor(lcPeach(), BLACK);
  tft.setCursor(70, 10);
  tft.print("uT");

  tft.setTextSize(1);
  char buf[8];
  int barEnd = METAL_BAR_X + (METAL_SEGS - 1) * METAL_BAR_PITCH + METAL_BAR_W;
  int tickX = METAL_BAR_X + metalRedFrom() * METAL_BAR_PITCH - 1;           // the gap before the first red segment
  tft.fillRect(tickX, 36, 1, 3, lcPeach());
  snprintf(buf, sizeof(buf), "%d", (int)DETECTION_THRESHOLD);
  tft.setTextColor(lcPeach(), BLACK);
  tft.setCursor(tickX - 3 * (int)strlen(buf) + 1, 39);
  tft.print(buf);
  snprintf(buf, sizeof(buf), "%d", (int)(METAL_SEGS * METAL_SEG_UT));
  tft.setTextColor(lcGrey(), BLACK);
  tft.setCursor(barEnd - 6 * (int)strlen(buf), 39);
  tft.print(buf);

  tft.setTextColor(lcLilac(), BLACK);
  tft.setCursor(8, 56);
  tft.print("MAG");

  for (int i = 0; i < METAL_SEGS; i++) metalDrawSegment(i, false);
}

void MetalDetector(int pressType) {
  static bool tftReady = false;       // is the frame on the colour screen?
  static int litNow = 0;              // bar segments currently lit
  static int verdictNow = -1;         // 0 = NO METAL shown, 1 = METAL DETECTED shown, -1 = neither yet
  static char shownDev[8];            // the digits currently drawn
  static char shownMag[8];

  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    tftReady = false;
    currentState = GEO_MENU;
    return;
  }
  if (pressType == LONG_PRESS && lastButton == 1) {
    tft.fillRoundRect(8, 28, 88, 22, 4, lcGold());                // the colour screen says so too (it takes ten seconds)
    tft.setTextSize(1);
    tft.setTextColor(BLACK, lcGold());
    tft.setCursor(8 + (88 - 66) / 2, 31);
    tft.print("CALIBRATING");
    tft.setCursor(8 + (88 - 60) / 2, 40);
    tft.print("KEEP CLEAR");
    calibrateMetalDetector();
    tftReady = false;                                             // redraw the whole screen afterwards
    return;
  }

      
  // Update at most every METAL_UPDATE_MS, but never wait: return at once in between, so the buttons and
  // Bluetooth keep running (the old delay(500) froze everything for half a second at a time).
  static unsigned long lastUpdateMs = 0;
  if (millis() - lastUpdateMs < METAL_UPDATE_MS) return;
  lastUpdateMs = millis();

  float x, y, z;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    float currentMagnitude = sqrt(x * x + y * y + z * z);
    float deviation = abs(currentMagnitude - baselineMagnitude);
    bool metalDetected = (deviation > DETECTION_THRESHOLD);

    telMetalMag = currentMagnitude;                // published for Bluetooth (see Bluetooth.ino)
    telMetalDev = deviation;
    telMetalDetected = metalDetected;
    telMetalMs = millis();

    // --- colour screen: the frame once, then only what changed ---
    if (!tftReady) {
      metalDrawStatic();
      tftTextForget(shownDev, sizeof(shownDev));
      tftTextForget(shownMag, sizeof(shownMag));
      litNow = 0;
      verdictNow = -1;
      tftReady = true;
    }
    int lit = metalLitSegments(deviation, metalDetected);
    if (lit != litNow) {
      int from = (lit < litNow) ? lit : litNow;
      int to = (lit < litNow) ? litNow : lit;
      for (int i = from; i < to; i++) metalDrawSegment(i, i < lit);
      litNow = lit;
    }
    if ((metalDetected ? 1 : 0) != verdictNow) {
      verdictNow = metalDetected ? 1 : 0;
      metalDrawVerdict(metalDetected);
      tftTextForget(shownDev, sizeof(shownDev));                  // the digits change colour with the verdict
    }
    char buf[8];
    float devShown = (deviation > 999.9) ? 999.9 : deviation;
    snprintf(buf, sizeof(buf), "%5.1f", devShown);
    tftTextDiff(shownDev, 9, 10, 2, buf, metalDetected ? lcAlert() : lcPeach(), BLACK);
    float magShown = (currentMagnitude > 999.9) ? 999.9 : currentMagnitude;
    snprintf(buf, sizeof(buf), "%5.1f", magShown);
    tftTextDiff(shownMag, 33, 56, 1, buf, lcPeach(), BLACK);

    // --- OLED: the readings and the verdict as text ---
    display.clearDisplay();
    display.setCursor(0, 9);
    display.println(F("Metal Detector"));
    display.setCursor(0, 20);
    display.print(F("Mag: "));
    display.print(currentMagnitude, 1);
    display.println(F(" uT"));
    display.setCursor(0, 30);
    display.print(F("Dev: "));
    display.print(deviation, 1);
    display.println(F(" uT"));
    display.setCursor(0, 55);
    if (metalDetected) {
      display.println(F("METAL DETECTED!"));
    } else {
      display.println(F("No metal"));
    }
    display.display();
  }
}
