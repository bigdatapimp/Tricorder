// Submenus only redraw the OLED when menuDirty is set (cursor moved, or the screen was just entered).
// The GPS clock on the TFT updates itself, and only when the time changes.

// ---- GEO Submenu ----
void handleGeoMenu(int pressType) {
  if (menuDirty) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
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
    display.display();
    menuDirty = false;
  }
  displayGPSClock();

  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      subMenuCursor = (subMenuCursor - 1 + 3) % 3;
      menuDirty = true;
    } else if (lastButton == 4) { // Down
      subMenuCursor = (subMenuCursor + 1) % 3;
      menuDirty = true;
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

// ---- ENV Submenu ----
void handleEnvMenu(int pressType) {
  if (menuDirty) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 9);
    display.println(F("ENVIRONMENTAL"));

    for (int i = 0; i < 3; i++) {
      int y = 20 + (i * 8);
      display.setCursor(20, y);
      if (i == subMenuCursor) {
        display.print("> ");
      } else {
        display.print("  ");
      }
      display.println(envMenuOptions[i]);
    }
    display.display();
    menuDirty = false;
  }
  displayGPSClock();

  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      subMenuCursor = (subMenuCursor - 1 + 3) % 3;
      menuDirty = true;
    } else if (lastButton == 4) { // Down
      subMenuCursor = (subMenuCursor + 1) % 3;
      menuDirty = true;
    } else if (lastButton == 1) { // Enter
      tft.fillScreen(BLACK);
      switch (subMenuCursor) {
        case 0: currentState = ATMOSPHERIC; break;
        case 1: currentState = DBMETER; break;
        case 2: currentState = SPECTRUM; break;
      }
    }
  } else if (pressType == LONG_PRESS && lastButton == 5) { // Long Left to exit
    currentState = MAIN_MENU;
    mainMenuCursor = 1;
    subMenuCursor = 0;
  }
}

// ---- COMMS Submenu ----
void handleCommsMenu(int pressType) {
  if (menuDirty) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);   // IR Send leaves its text colour black if its last item was selected
    display.setCursor(10, 9);
    display.println(F("COMMUNICATIONS"));

    for (int i = 0; i < COMMS_MENU_COUNT; i++) {
      int y = 20 + (i * 8);
      display.setCursor(20, y);
      if (i == subMenuCursor) {
        display.print("> ");
      } else {
        display.print("  ");
      }
      display.println(commsMenuOptions[i]);
    }
    display.display();
    menuDirty = false;
  }
  displayGPSClock();

  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      subMenuCursor = (subMenuCursor - 1 + COMMS_MENU_COUNT) % COMMS_MENU_COUNT;
      menuDirty = true;
    } else if (lastButton == 4) { // Down
      subMenuCursor = (subMenuCursor + 1) % COMMS_MENU_COUNT;
      menuDirty = true;
    } else if (lastButton == 1) { // Enter
      tft.fillScreen(BLACK);
      switch (subMenuCursor) {
        case 0: currentState = BLUETOOTH_SCREEN; break;
        case 1: currentState = IRSEND; break;
        case 2: currentState = IRRECEIVER; break;
      }
    }
  } else if (pressType == LONG_PRESS && lastButton == 5) { // Long Left to exit
    currentState = MAIN_MENU;
    mainMenuCursor = 2;
    subMenuCursor = 0;
  }
}

// ---- SYS Submenu ----
void handleSysMenu(int pressType) {
  if (menuDirty) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 9);
    display.println(F("SYSTEM"));

    for (int i = 0; i < 4; i++) {
      int y = 20 + (i * 8);
      display.setCursor(20, y);
      if (i == subMenuCursor) {
        display.print("> ");
      } else {
        display.print("  ");
      }
      display.println(sysMenuOptions[i]);
    }
    display.display();
    menuDirty = false;
  }
  displayGPSClock();

  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      subMenuCursor = (subMenuCursor - 1 + 4) % 4;
      menuDirty = true;
    } else if (lastButton == 4) { // Down
      subMenuCursor = (subMenuCursor + 1) % 4;
      menuDirty = true;
    } else if (lastButton == 1) { // Enter
      tft.fillScreen(BLACK);
      switch (subMenuCursor) {
        case 0: currentState = BATTERY; break;
        case 1: currentState = SYSINFO; break;
        case 2: currentState = TIMEZONE; break;
        case 3: currentState = REBOOT; break;
      }
    }
  } else if (pressType == LONG_PRESS && lastButton == 5) { // Long Left to exit
    currentState = MAIN_MENU;
    mainMenuCursor = 4;
    subMenuCursor = 0;
  }
}

// ---- UTIL Submenu ----
void handleUtilMenu(int pressType) {
  if (menuDirty) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 9);
    display.println(F("UTILITY"));

    for (int i = 0; i < 3; i++) {
      int y = 20 + (i * 8);
      display.setCursor(20, y);
      if (i == subMenuCursor) {
        display.print("> ");
      } else {
        display.print("  ");
      }
      display.println(utilMenuOptions[i]);
    }
    display.display();
    menuDirty = false;
  }
  displayGPSClock();

  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      subMenuCursor = (subMenuCursor - 1 + 3) % 3;
      menuDirty = true;
    } else if (lastButton == 4) { // Down
      subMenuCursor = (subMenuCursor + 1) % 3;
      menuDirty = true;
    } else if (lastButton == 1) { // Enter
      tft.fillScreen(BLACK);
      switch (subMenuCursor) {
        case 0: currentState = SHOTTIMER; break;
        case 1: currentState = STOPWATCH; break;
        case 2: currentState = COUNTDOWNTIMER; break;
      }
    }
  } else if (pressType == LONG_PRESS && lastButton == 5) { // Long Left to exit
    currentState = MAIN_MENU;
    mainMenuCursor = 3;
    subMenuCursor = 0;
  }
}
