// --------- IR Receiver code ----------
void irReceiver(int pressType) {
  
  // --- Handle Exit (Long Press Left) ---
  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    display.clearDisplay();
    display.display();
    irDataReceived = false;
    currentState = UTIL_MENU;
    return;
  }

  // --- Wait for signal (LOW) ---
  if (digitalRead(IR_RECV_PIN) == LOW) {
    //Serial.println("--- Signal Detected ---");
    
    // Wait for HIGH (end of header mark)
    while (digitalRead(IR_RECV_PIN) == LOW) {}
    
    // Measure header space
    unsigned long start = micros();
    while (digitalRead(IR_RECV_PIN) == HIGH) {}
    unsigned long headerSpace = micros() - start;
    
    //Serial.print("Header Space: ");
   // Serial.println(headerSpace);
    
    // Read 32 bits
   // Serial.print("Data: ");
    uint32_t data = 0;
    
    for (int i = 0; i < 32; i++) {
      // Measure mark (HIGH)
      start = micros();
      while (digitalRead(IR_RECV_PIN) == LOW) {}
      unsigned long mark = micros() - start;
      
      // Measure space (HIGH)
      start = micros();
      while (digitalRead(IR_RECV_PIN) == HIGH && micros() - start < 3000) {}
      unsigned long space = micros() - start;
      
      // Shift data
      data <<= 1;
      if (space > 1000) {
        data |= 1;
        //Serial.print("1");
      } else {
       // Serial.print("0");
      }
    }
    //Serial.println("");
    
    //Serial.print("Value: 0x");
    //Serial.println(data, HEX);
    
    // Extract parts
    uint8_t addr = (data >> 24) & 0xFF;
    uint8_t cmd = (data >> 8) & 0xFF;
    
    /* - Serial.print("Address: 0x");
    Serial.println(addr, HEX);
    Serial.print("Command: 0x");
    Serial.println(cmd, HEX); - */
    
    // --- Store data and update display ONCE ---
    storedIrData = data;
    storedIrAddr = addr;
    storedIrCmd = cmd;
    irDataReceived = true;
    
    // Display on SSD1331
    tft.fillScreen(BLACK);
    tft.setTextSize(1);
    tft.setTextColor(GREEN);
    tft.setCursor(0, 0);
    tft.println("SIGNAL RECEIVED");
    
    tft.setTextColor(WHITE);
    tft.setCursor(0, 12);
    tft.print("Data: 0x");
    char buffer[20];
    sprintf(buffer, "%08lX", storedIrData);
    tft.println(buffer);
    
    tft.setCursor(0, 24);
    tft.print("Addr: 0x");
    sprintf(buffer, "%02X", storedIrAddr);
    tft.println(buffer);
    
    tft.setCursor(0, 36);
    tft.print("Cmd: 0x");
    sprintf(buffer, "%02X", storedIrCmd);
    tft.println(buffer);
       
    delay(500);
  }
  // --- Show waiting message if no signal received yet ---
  if (!irDataReceived) {
    tft.fillScreen(BLACK);
    /*tft.setTextSize(1);
    tft.setTextColor(CYAN);
    tft.setCursor(0, 0);
    tft.println("IR RECEIVER");
    tft.setTextColor(WHITE);
    tft.setCursor(0, 10);
    tft.println("Waiting for");
    tft.println("signal...");
    tft.setTextColor(YELLOW);
    tft.setCursor(0, 50);
    tft.println("Press LEFT");
    tft.println("to exit");*/

    display.clearDisplay();
    display.setCursor(0, 9);
    display.println(F("IR Receiver"));
    display.setCursor(0, 20);
    display.println(F("Waiting for"));
    display.println(F("signal..."));
    display.setCursor(0, 40);
    display.print("Hold L to Exit");
    display.display();
  }
}