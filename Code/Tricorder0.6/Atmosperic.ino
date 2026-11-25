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

// ---- the OLED (LCARS look, see LcarsOled.ino): temperature, dew point, humidity with a bar, pressure, and the keys that change
// the units. Readings that are not usable show --.
void atmoDrawOled(float temp, const char* tempUnit, float dew, bool dewOk, float humidity, float pressure, const char* presUnit) {
  char buf[20];
  display.clearDisplay();
  lcoFrame("ATMOSPHERIC", NULL);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  lcoPill(9, 18, 30, 8, "TEMP");
  if (atmoFinite(temp)) snprintf(buf, sizeof(buf), "%.1f %s", temp, tempUnit); else snprintf(buf, sizeof(buf), "--");
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(127 - 6 * (int)strlen(buf), 18);
  display.print(buf);

  lcoPill(9, 27, 24, 8, "DEW");
  if (dewOk) snprintf(buf, sizeof(buf), "%.1f %s", dew, tempUnit); else snprintf(buf, sizeof(buf), "--");
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(127 - 6 * (int)strlen(buf), 27);
  display.print(buf);

  lcoPill(9, 36, 24, 8, "HUM");                                  // the humidity bar: 14 segments of 7 %
  bool humOk = atmoFinite(humidity);
  int lit = humOk ? (int)(humidity * 14.0 / 100.0 + 0.5) : 0;
  if (lit < 0) lit = 0;
  if (lit > 14) lit = 14;
  lcoBar(40, 37, 14, lit, 3, 6, 4);
  if (humOk) snprintf(buf, sizeof(buf), "%.0f%%", humidity); else snprintf(buf, sizeof(buf), "--");
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(127 - 6 * (int)strlen(buf), 36);
  display.print(buf);

  lcoPill(9, 45, 36, 8, "PRESS");
  if (atmoFinite(pressure) && pressure > 0.0) snprintf(buf, sizeof(buf), "%.2f %s", pressure, presUnit); else snprintf(buf, sizeof(buf), "--");
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(127 - 6 * (int)strlen(buf), 45);
  display.print(buf);

  display.setCursor(9, 55);                                      // which buttons change the units
  display.print("HOLD RT=C/F DN=kPa");
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
    atmoDrawOled(displayTemp, tempUnitStr.c_str(), displayDewPt, humidity > 0.0 && atmoFinite((float)dewPointC), humidity,
                 displayPressure, presUnitStr.c_str());

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
