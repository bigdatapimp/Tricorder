// ---- GEO Submenu ----
void handleGeoMenu(int pressType) {
  display.clearDisplay();
  display.setCursor(10, 9);
  display.println(F("GEOLOGICAL"));

  
  for (int i = 0; i < 3; i++) {
    int y = 20 + (i * 8);
    display.setCursor(20, y);
    if (i == subMenuCursor) {
      display.print("> ");
    } else {
      display.print("  ");
    }
    display.println(geoMenuOptions[i]);
  }
  displayGPSClock();
  display.display();
  
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      subMenuCursor = (subMenuCursor - 1 + 3) % 3;
    } else if (lastButton == 4) { // Down
      subMenuCursor = (subMenuCursor + 1) % 3;
    } else if (lastButton == 1) { // Enter
      tft.fillScreen(BLACK);
      switch (subMenuCursor) {
        case 0: currentState = METALDETECTOR; break;
        case 1: currentState = EMFMETER; break;
        case 2: currentState = NAVIGATION; break;
      }
    }
  } else if (pressType == LONG_PRESS && lastButton == 5) { // Long Left to exit
    currentState = MAIN_MENU;
    mainMenuCursor = 0;
    subMenuCursor = 0;
  }
}

// ---- MET Submenu ----
void handleMetMenu(int pressType) {
  display.clearDisplay();
  display.setCursor(10, 9);
  display.println(F("METEOROLIGICAL"));
  
  for (int i = 0; i < 2; i++) {
    int y = 20 + (i * 8);
    display.setCursor(20, y);
    if (i == subMenuCursor) {
      display.print("> ");
    } else {
      display.print("  ");
    }
    display.println(metMenuOptions[i]);
  }
  displayGPSClock();
  display.display();
  
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      subMenuCursor = (subMenuCursor - 1 + 2) % 2;
    } else if (lastButton == 4) { // Down
      subMenuCursor = (subMenuCursor + 1) % 2;
    } else if (lastButton == 1) { // Enter
      tft.fillScreen(BLACK);
      switch (subMenuCursor) {
        case 0: currentState = ATMOSPHERIC; break;
        case 1: currentState = DBMETER; break;
      }
    }
  } else if (pressType == LONG_PRESS && lastButton == 5) { // Long Left to exit
    currentState = MAIN_MENU;
    mainMenuCursor = 1;
    subMenuCursor = 0;
  }
}

// ---- BIO Submenu ----
void handleBioMenu(int pressType) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(10, 9);
  display.println(F("BIOLOGICAL"));
  display.setCursor(10, 35);
  display.println(F("Waiting for"));
  display.setCursor(15, 45);
  display.println(F("Update"));

  displayGPSClock();
  display.display();
  
  if (pressType == LONG_PRESS && lastButton == 5) { // Long Left to exit
    currentState = MAIN_MENU;
    mainMenuCursor = 2;
    subMenuCursor = 0;
  }
}

// ---- SYS Submenu ----
void handleSysMenu(int pressType) {
  display.clearDisplay();
  display.setCursor(10, 9);
  display.println(F("SYSTEM"));
  
  int y = 20;
  display.setCursor(20, y);
  if (subMenuCursor == 0) {
    display.print("> ");
  } else {
    display.print("  ");
  }
  display.println(sysMenuOptions[0]);
  
  displayGPSClock();
  display.display();
  
  if (pressType == SHORT_PRESS) {
    if (lastButton == 1) { // Enter
      tft.fillScreen(BLACK);
      currentState = BATTERY;
    }
  } else if (pressType == LONG_PRESS && lastButton == 5) { // Long Left to exit
    currentState = MAIN_MENU;
    mainMenuCursor = 3;
    subMenuCursor = 0;
  }
}

// ---- UTIL Submenu ----
void handleUtilMenu(int pressType) {
display.clearDisplay();
  display.setCursor(10, 9);
  display.println(F("UTILITY"));
  
  for (int i = 0; i < 2; i++) {
    int y = 20 + (i * 8);
    display.setCursor(20, y);
    if (i == subMenuCursor) {
      display.print("> ");
    } else {
      display.print("  ");
    }
    display.println(utilMenuOptions[i]);
  }
  
  displayGPSClock();
  display.display();
  
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      subMenuCursor = (subMenuCursor - 1 + 2) % 2;
    } else if (lastButton == 4) { // Down
      subMenuCursor = (subMenuCursor + 1) % 2;
    } else if (lastButton == 1) { // Enter
      tft.fillScreen(BLACK);
      switch (subMenuCursor) {
        case 0: currentState = IRSEND; break;
        case 1: currentState = IRRECEIVER; break;
      }
    }
  } else if (pressType == LONG_PRESS && lastButton == 5) { // Long Left to exit
    currentState = MAIN_MENU;
    mainMenuCursor = 1;
    subMenuCursor = 0;
  }
}

