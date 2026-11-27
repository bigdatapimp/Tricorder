// ----------- Atmospheric code ------------

// ---- the colour screen (LCARS look, see Lcars.ino): blue (low) to red (high) bars for the temperature, the dew point and
// the pressure, with the same scales and colours as the phone app. Each row is a label block with the reading on the
// same line, and a bar under it: 22 segments of 4 pixels fill the area beside the frame; the segments up to the reading
// are lit, the rest are dim. Layout (y of the top of each item):
//   10  TEMP    label block, the reading at the right, the bar at +8 (6 pixels high)
//   25  DEW
//   40  PRESS
//   55  the ends of the pressure scale (29.04 ... 31.44 inHg, or 97.0 ... 105.0 kPa)
#define ATMO_TEMP_LOW_C     -10.0    // the ends of the temperature scale, in Celsius
#define ATMO_TEMP_HIGH_C     45.0
#define ATMO_PRESS_LOW_HPA   970.0   // the ends of the pressure scale
#define ATMO_PRESS_HIGH_HPA 1050.0
#define ATMO_INHG_PER_HPA    0.02994 // hPa -> inHg, the factor used for the numbers on the OLED
#define ATMO_BAR_SEGMENTS    22
#define ATMO_BAR_X           8
#define ATMO_BAR_PITCH       4
#define ATMO_BAR_HEIGHT      6
#define ATMO_ROW_TEMP        10
#define ATMO_ROW_DEW         25
#define ATMO_ROW_PRESS       40
#define ATMO_SCALE_PRESS     55

bool atmoTftReady = false;           // is the frame on the colour screen? (redrawn on entry and when the pressure unit changes)

// A usable number: not NaN (a NaN never equals itself) and not infinite
bool atmoFinite(float v) {
  return v == v && v > -3.0e38 && v < 3.0e38;
}

// How many segments are lit for a value on a scale: none at the low end or below, all of them at the high end or above
int atmoLit(float value, float low, float high) {
  if (!atmoFinite(value)) return 0;
  float f = (value - low) / (high - low);
  if (f < 0.0) f = 0.0;
  if (f > 1.0) f = 1.0;
  return (int)(f * ATMO_BAR_SEGMENTS + 0.5);
}

int atmoLitSegments(float tempC) { return atmoLit(tempC, ATMO_TEMP_LOW_C, ATMO_TEMP_HIGH_C); }
int atmoLitPressure(float hPa) { return atmoLit(hPa, ATMO_PRESS_LOW_HPA, ATMO_PRESS_HIGH_HPA); }

void atmoDrawBar(int y, int lit) {
  for (int i = 0; i < ATMO_BAR_SEGMENTS; i++) {
    lcSeg(ATMO_BAR_X + i * ATMO_BAR_PITCH, y, ATMO_BAR_PITCH - 1, ATMO_BAR_HEIGHT, lcHeat(i, ATMO_BAR_SEGMENTS), i < lit);
  }
}

// The ends of the pressure scale in the unit being shown, e.g. "29.04" and "31.44" (inHg) or "97.0" and "105.0" (kPa)
void atmoDrawPressScaleEnds(int y) {
  char low[10], high[10];
  if (useKPa) {
    snprintf(low, sizeof(low), "%.1f", ATMO_PRESS_LOW_HPA * 0.1);
    snprintf(high, sizeof(high), "%.1f", ATMO_PRESS_HIGH_HPA * 0.1);
  } else {
    snprintf(low, sizeof(low), "%.2f", ATMO_PRESS_LOW_HPA * ATMO_INHG_PER_HPA);
    snprintf(high, sizeof(high), "%.2f", ATMO_PRESS_HIGH_HPA * ATMO_INHG_PER_HPA);
  }
  tft.setTextSize(1);
  tft.setTextColor(lcGrey(), BLACK);
  tft.setCursor(ATMO_BAR_X, y);
  tft.print(low);
  tft.setCursor(96 - 6 * (int)strlen(high), y);
  tft.print(high);
}

// The parts of the colour screen that do not change: the frame, the label blocks and the pressure scale ends
void atmoDrawTftStatic() {
  lcFrame("ATMOSPHERIC", false);
  lcPill(8, ATMO_ROW_TEMP, 26, lcOrange(), "TEMP");
  lcPill(8, ATMO_ROW_DEW, 20, lcLilac(), "DEW");
  lcPill(8, ATMO_ROW_PRESS, 32, lcBlue(), "PRESS");
  atmoDrawPressScaleEnds(ATMO_SCALE_PRESS);
}

// A reading of 'width' characters against the right edge, drawn over a black background so that it replaces the last
// reading without clearing anything
void atmoDrawReading(int y, const char* text, int width) {
  tft.setTextSize(1);
  tft.setTextColor(lcPeach(), BLACK);
  tft.setCursor(96 - 6 * width, y);
  tft.print(text);
}

// A temperature row: the reading (" 22.5C") and its bar
void atmoDrawTftRow(int y, float shown, float tempC, bool valid) {
  char text[12];
  if (valid) snprintf(text, sizeof(text), "%5.1f%c", shown, useCelsius ? 'C' : 'F');
  else       snprintf(text, sizeof(text), "%6s", "--");
  atmoDrawReading(y, text, 6);
  atmoDrawBar(y + 8, valid ? atmoLitSegments(tempC) : 0);
}

// The pressure row: the reading ("30.34inHg" or "101.30kPa", 9 characters) and its bar
void atmoDrawPressRow(int y, float shown, float hPa, bool valid) {
  char text[16];
  if (valid) snprintf(text, sizeof(text), useKPa ? "%6.2f%s" : "%5.2f%s", shown, useKPa ? "kPa" : "inHg");
  else       snprintf(text, sizeof(text), "%9s", "--");
  atmoDrawReading(y, text, 9);
  atmoDrawBar(y + 8, valid ? atmoLitPressure(hPa) : 0);
}

// ---- the OLED (LCARS look, see LcarsOled.ino): the humidity bar and a weather icon under it ----
//
// The icon follows a "cloudiness" score from 0 to 1 that adds up three things:
//   humidity                       40 %   nothing at 30 % or less, all of it at 100 %
//   the dew point near the air     40 %   nothing at a gap of 12 C or more, all of it when they meet
//   falling pressure               20 %   nothing at 1025 hPa or more, all of it at 995 hPa or less
// and shows, from clear to overcast:
//   0 a sun (score under 0.30)   1 the sun behind a cloud (to 0.51)   2 a cloud, 9/16 solid (to 0.71)
//   3 a cloud, 12/16 solid (to 0.88)   4 a solid cloud
float atmoCloudScore(float humidity, float tempC, float dewC, bool dewOk, float hPa) {
  float h = (humidity - 30.0) / 70.0;
  if (h < 0.0) h = 0.0;
  if (h > 1.0) h = 1.0;
  float d = 0.0;                                   // no usable dew point counts as nothing
  if (dewOk && atmoFinite(tempC)) {
    d = 1.0 - (tempC - dewC) / 12.0;
    if (d < 0.0) d = 0.0;
    if (d > 1.0) d = 1.0;
  }
  float p = 0.0;                                   // no usable pressure counts as nothing
  if (atmoFinite(hPa) && hPa > 0.0) {
    p = (1025.0 - hPa) / 30.0;
    if (p < 0.0) p = 0.0;
    if (p > 1.0) p = 1.0;
  }
  return 0.4 * h + 0.4 * d + 0.2 * p;
}

int atmoCloudStage(float score) {
  if (score < 0.30) return 0;
  if (score < 0.51) return 1;
  if (score < 0.71) return 2;
  if (score < 0.88) return 3;
  return 4;
}

// The shapes, in a slot of 62 x 32 pixels: a cloud made of three circles on a base, and a smaller one in front of the sun
bool atmoInCircle(int x, int y, int cx, int cy, int r) {
  return (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r;
}

bool atmoBigCloud(int x, int y) {
  return atmoInCircle(x, y, 13, 19, 9) || atmoInCircle(x, y, 31, 12, 12) || atmoInCircle(x, y, 49, 17, 10) ||
         (x >= 13 && x <= 49 && y >= 19 && y <= 28);
}

bool atmoSmallCloud(int x, int y) {
  x -= 18;
  y -= 9;
  return atmoInCircle(x, y, 10, 15, 6) || atmoInCircle(x, y, 22, 10, 9) || atmoInCircle(x, y, 33, 14, 7) ||
         (x >= 10 && x <= 33 && y >= 15 && y <= 21);
}

// A cloud: its outline, and the inside filled with a dot pattern that gets denser with 'level' (0 empty to 16 solid)
void atmoDrawCloud(int x0, int y0, int level) {
  static const uint8_t bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
  for (int y = 0; y < 32; y++) {
    for (int x = 0; x < 62; x++) {
      if (!atmoBigCloud(x, y)) continue;
      bool edge = !atmoBigCloud(x - 1, y) || !atmoBigCloud(x + 1, y) || !atmoBigCloud(x, y - 1) || !atmoBigCloud(x, y + 1);
      if (edge || bayer[y & 3][x & 3] < level) display.drawPixel(x0 + x, y0 + y, SSD1306_WHITE);
    }
  }
}

// A sun: a disc with eight rays from 'rayFrom' to 'rayTo' pixels out from the centre (the four straight ones are 2 pixels thick)
void atmoDrawSun(int cx, int cy, int r, int rayFrom, int rayTo) {
  static const float dirX[8] = {1.0, 0.7071, 0.0, -0.7071, -1.0, -0.7071, 0.0, 0.7071};
  static const float dirY[8] = {0.0, 0.7071, 1.0, 0.7071, 0.0, -0.7071, -1.0, -0.7071};
  display.fillCircle(cx, cy, r, SSD1306_WHITE);
  for (int k = 0; k < 8; k++) {
    for (int t = rayFrom; t <= rayTo; t++) {
      int px = cx + (int)round(t * dirX[k]);
      int py = cy + (int)round(t * dirY[k]);
      display.drawPixel(px, py, SSD1306_WHITE);
      if (k == 0 || k == 4) display.drawPixel(px, py + 1, SSD1306_WHITE);
      else if (k == 2 || k == 6) display.drawPixel(px + 1, py, SSD1306_WHITE);
    }
  }
}

// The sun with a small cloud in front of it: the cloud is an outline with nothing of the sun showing through
void atmoDrawSunBehindCloud(int x0, int y0) {
  atmoDrawSun(x0 + 16, y0 + 14, 6, 9, 12);
  for (int y = 0; y < 32; y++) {
    for (int x = 0; x < 62; x++) {
      if (!atmoSmallCloud(x, y)) continue;
      bool edge = !atmoSmallCloud(x - 1, y) || !atmoSmallCloud(x + 1, y) || !atmoSmallCloud(x, y - 1) || !atmoSmallCloud(x, y + 1);
      display.drawPixel(x0 + x, y0 + y, edge ? SSD1306_WHITE : SSD1306_BLACK);
    }
  }
}

// The icon for a stage (0 to 4), centred under the humidity bar
void atmoDrawIcon(int stage) {
  int x0 = 36;
  int y0 = 31;
  if (stage == 0)      atmoDrawSun(x0 + 27, y0 + 15, 8, 11, 15);
  else if (stage == 1) atmoDrawSunBehindCloud(x0, y0);
  else                 atmoDrawCloud(x0, y0, stage == 2 ? 9 : (stage == 3 ? 12 : 16));
}

// The whole OLED screen: the HUM block with its 14-segment bar and the reading, and the icon. Without a humidity reading there
// is no bar and no icon (the reading shows --).
void atmoDrawOled(float humidity, float tempC, float dewC, bool dewOk, float hPa) {
  char buf[8];
  display.clearDisplay();
  lcoFrame("ATMOSPHERIC", NULL);
  display.setTextSize(1);

  lcoPill(9, 18, 24, 8, "HUM");
  bool humOk = atmoFinite(humidity);
  int lit = humOk ? (int)(humidity * 14.0 / 100.0 + 0.5) : 0;
  if (lit < 0) lit = 0;
  if (lit > 14) lit = 14;
  lcoBar(40, 19, 14, lit, 3, 6, 4);
  if (humOk) snprintf(buf, sizeof(buf), "%.0f%%", humidity); else snprintf(buf, sizeof(buf), "--");
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(127 - 6 * (int)strlen(buf), 18);
  display.print(buf);

  if (humOk) atmoDrawIcon(atmoCloudStage(atmoCloudScore(humidity, tempC, dewC, dewOk, hPa)));
  display.display();
}

void Atmospheric(int pressType) {

  // --- Handle Button Inputs ---
  if (pressType == LONG_PRESS) {
    // Long Press Right (Button 3) -> Toggle Temp F/C
    if (lastButton == 3) {
      useCelsius = !useCelsius;
      // Force an immediate redraw so the user sees the change instantly
      lastAtmosphericUpdate = 0; 
    }
    // Long Press Down (Button 4) -> Toggle Pressure inHg/kPa
    else if (lastButton == 4) {
      useKPa = !useKPa;
      atmoTftReady = false;                  // the pressure scale ends change with the unit
      // Force an immediate redraw
      lastAtmosphericUpdate = 0;
    }
    // Long Press Left (Button 5) -> Exit to Menu
    else if (lastButton == 5) {
      tft.fillScreen(BLACK);                 // the colour screen is ours only while this screen is open
      atmoTftReady = false;
      currentState = ENV_MENU;
      return;
    }
  }

  // --- Update Screen every 2 seconds (or if settings changed) ---
  if (millis() - lastAtmosphericUpdate > 2000) {
    
    // 1. Read Sensors
    float temperatureC = HS300x.readTemperature();
    float humidity = HS300x.readHumidity();
    float pressurehPa = BARO.readPressure(MILLIBAR); // Sensor returns hPa (same as mbar)

    // 2. Calculate Dew Point (Magnus Formula)
    // Note: Magnus formula requires Celsius input
    double alpha = log(humidity / 100.0) + (17.27 * temperatureC) / (237.7 + temperatureC);
    double dewPointC = (237.7 * alpha) / (17.27 - alpha);

    // 3. Handle Unit Conversions
    float displayTemp;
    float displayDewPt;
    String tempUnitStr;
    String dewUnitStr;

    if (useCelsius) {
      displayTemp = temperatureC;
      displayDewPt = dewPointC;
      tempUnitStr = "C";
      dewUnitStr = "C";
    } else {
      displayTemp = temperatureC * 9.0 / 5.0 + 32.0;
      displayDewPt = dewPointC * 9.0 / 5.0 + 32.0;
      tempUnitStr = "F";
      dewUnitStr = "F";
    }

    float displayPressure;
    String presUnitStr;
    if (useKPa) {
      displayPressure = pressurehPa * 0.1; // Convert hPa to kPa
      presUnitStr = "kPa";
    } else {
      displayPressure = pressurehPa * ATMO_INHG_PER_HPA; // Convert hPa to inHg
      presUnitStr = "inHg";
    }

    // 4. Draw to OLED
    atmoDrawOled(humidity, temperatureC, (float)dewPointC, humidity > 0.0 && atmoFinite((float)dewPointC), pressurehPa);

    // 5. Draw to the colour screen: the scale ends once, then only the numbers and the bars (temperature, dew point, pressure)
    if (!atmoTftReady) {
      atmoDrawTftStatic();
      atmoTftReady = true;
    }
    atmoDrawTftRow(ATMO_ROW_TEMP, displayTemp, temperatureC, atmoFinite(temperatureC));
    bool dewValid = (humidity > 0.0) && atmoFinite((float)dewPointC);          // no humidity reading: no dew point
    atmoDrawTftRow(ATMO_ROW_DEW, displayDewPt, dewPointC, dewValid);
    atmoDrawPressRow(ATMO_ROW_PRESS, displayPressure, pressurehPa, atmoFinite(pressurehPa) && pressurehPa > 0.0);

    lastAtmosphericUpdate = millis();
  }
}
