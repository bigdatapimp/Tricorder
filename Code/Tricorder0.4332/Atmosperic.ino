// ----------- Atmospheric code ------------
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
      // Force an immediate redraw
      lastAtmosphericUpdate = 0;
    }
    // Long Press Left (Button 5) -> Exit to Menu
    else if (lastButton == 5) {
      currentState = MET_MENU;
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
      displayPressure = pressurehPa * 0.02994; // Convert hPa to inHg
      presUnitStr = "inHg";
    }

    // 4. Draw to OLED
    display.clearDisplay();
    display.setCursor(0, 9);
    display.println(F("Weather Data"));

    // -- Temperature --
    display.setCursor(0, 20);
    display.print(F("Temp: "));
    display.print(displayTemp, 1);
    display.print(F(" "));
    display.println(tempUnitStr);

    // -- Dew Point (Below Temp) --
    display.setCursor(0, 30);
    display.print(F("Dew Pt: "));
    display.print(displayDewPt, 1);
    display.print(F(" "));
    display.println(dewUnitStr);

    // -- Humidity --
    display.setCursor(0, 40);
    display.print(F("Humidity: "));
    display.print(humidity, 1);
    display.println(F(" %"));

    // -- Pressure --
    display.setCursor(0, 50);
    display.print(F("Pressure: "));
    // Adjust decimal places based on unit for better readability
    if(useKPa) display.print(displayPressure, 2); 
    else       display.print(displayPressure, 2);
    
    display.print(F(" "));
    display.println(presUnitStr);

    display.display();

    lastAtmosphericUpdate = millis();
  }
}
