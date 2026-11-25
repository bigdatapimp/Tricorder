// ---------- EMF Meter code ----------
//
// The colour screen (LCARS look, see Lcars.ino) shows the reading in big digits, a 20-segment bar from blue to red
// (MAX_EMF / 20 = 10 uT per segment) and the usual range of the Earth's own field. The OLED shows the title and the reading.
#define EMF_UPDATE_MS 200      // the screen updates 5 times a second
#define EMF_SEGS      20       // bar segments, MAX_EMF / EMF_SEGS uT each
#define EMF_BAR_X     12
#define EMF_BAR_Y     27
#define EMF_BAR_PITCH 4
#define EMF_BAR_W     3
#define EMF_BAR_H     9

// How many bar segments are lit for a reading in uT: rounded to the nearest, none for zero or an unusable reading
int emfLitSegments(float uT) {
  if (!(uT > 0.0)) return 0;                       // also catches NaN
  int n = (int)(uT * EMF_SEGS / MAX_EMF + 0.5);
  if (n > EMF_SEGS) n = EMF_SEGS;
  return n;
}

void emfDrawSegment(int i, bool lit) {
  lcSeg(EMF_BAR_X + i * EMF_BAR_PITCH, EMF_BAR_Y, EMF_BAR_W, EMF_BAR_H, lcHeat(i, EMF_SEGS), lit);
}

// The parts of the colour screen that do not change: the frame, the unit, the scale, the Earth's-field note, a dark bar
void emfDrawStatic() {
  lcFrame("EMF METER", false);
  tft.setTextSize(2);
  tft.setTextColor(lcPeach(), BLACK);
  tft.setCursor(70, 10);
  tft.print("uT");

  char buf[8];
  tft.setTextSize(1);
  tft.setTextColor(lcGrey(), BLACK);
  int barEnd = EMF_BAR_X + (EMF_SEGS - 1) * EMF_BAR_PITCH + EMF_BAR_W;        // x of the bar's last pixel + 1
  tft.setCursor(EMF_BAR_X, 37);
  tft.print("0");
  snprintf(buf, sizeof(buf), "%d", (int)(MAX_EMF / 2.0));
  tft.setCursor((EMF_BAR_X + barEnd) / 2 - 3 * (int)strlen(buf), 37);
  tft.print(buf);
  snprintf(buf, sizeof(buf), "%d", (int)MAX_EMF);
  tft.setCursor(barEnd - 6 * (int)strlen(buf), 37);
  tft.print(buf);

  lcPill(8, 50, 34, lcLilac(), "EARTH");
  tft.setTextColor(lcPeach(), BLACK);
  tft.setCursor(45, 50);
  tft.print("25-65uT");

  for (int i = 0; i < EMF_SEGS; i++) emfDrawSegment(i, false);
}

void EMFMeter(int pressType) {
  static bool tftReady = false;       // is the frame on the colour screen?
  static int litNow = 0;              // bar segments currently lit
  static char shown[8];               // the digits currently drawn

  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    tftReady = false;
    currentState = GEO_MENU;
    return;
  }


  // Update at most every EMF_UPDATE_MS, but never wait: return at once in between, so the buttons and
  // Bluetooth keep running (the old delay(500) froze everything for half a second at a time).
  static unsigned long lastUpdateMs = 0;
  if (millis() - lastUpdateMs < EMF_UPDATE_MS) return;
  lastUpdateMs = millis();

  float x, y, z;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    float emfMagnitude = sqrt(x * x + y * y + z * z);

    telEmfMag = emfMagnitude;                      // published for Bluetooth (see Bluetooth.ino)
    telEmfMs = millis();

    // --- colour screen: the frame once, then only the digits and the segments that changed ---
    if (!tftReady) {
      emfDrawStatic();
      tftTextForget(shown, sizeof(shown));
      litNow = 0;
      tftReady = true;
    }
    int lit = emfLitSegments(emfMagnitude);
    if (lit != litNow) {
      int from = (lit < litNow) ? lit : litNow;
      int to = (lit < litNow) ? litNow : lit;
      for (int i = from; i < to; i++) emfDrawSegment(i, i < lit);
      litNow = lit;
    }
    float digits = (emfMagnitude > 999.9) ? 999.9 : emfMagnitude;
    char buf[8];
    snprintf(buf, sizeof(buf), "%5.1f", digits);
    tftTextDiff(shown, 9, 10, 2, buf, lcPeach(), BLACK);

    // --- OLED: the title and the reading ---
    display.clearDisplay();
    display.setCursor(0, 9);
    display.println(F("EMF Meter"));
    display.setCursor(0, 20);
    display.print(F("EMF: "));
    display.print(emfMagnitude, 1);
    display.println(F(" uT"));
    display.display();
  }
}
