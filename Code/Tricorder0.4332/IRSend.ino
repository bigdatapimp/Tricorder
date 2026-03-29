// --------- IR Sender code ----------
void IrSend(int pressType) {
  static int lastIrSendCursor = -1; 
  
  uint8_t sRepeats = 1;
  
  // --- Original Options ---
  uint16_t pwr_Address = 0x4;
  uint8_t pwr_Command = 0x8;
  uint16_t vup_Address = 0x4;
  uint8_t vup_Command = 0x2;
  uint16_t vdn_Address = 0x4;
  uint8_t vdn_Command = 0x3;
  
  // --- New Options ---
  uint16_t ok_Address = 0x4;
  uint8_t ok_Command = 0x5A;
  
  uint16_t dlt_Address = 0x4;
  uint8_t dlt_Command = 0x58;
  
  uint16_t drt_Address = 0x4;
  uint8_t drt_Command = 0x59;
  
  uint16_t ddn_Address = 0x4;
  uint8_t ddn_Command = 0x57;
  
  uint16_t dup_Address = 0x4;
  uint8_t dup_Command = 0x56;
  
  uint16_t amzn_Address = 0x4;
  uint8_t amzn_Command = 0x47;
  
  uint16_t yt_Address = 0x4;
  uint8_t yt_Command = 0x49;

  uint16_t bck_Address = 0x4;
  uint8_t bck_Command = 0x4;

  uint16_t ext_Address = 0x4;
  uint8_t ext_Command = 0x4D;
  
  // --- Handle Button Inputs ---
  
  // Exit: Long Press Left (Button 5)
  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = UTIL_MENU;
    lastIrSendCursor = -1;
    return;
  }

  // Navigation: Short Press Enter (Button 1)
  if (pressType == SHORT_PRESS && lastButton == 1) {
    tft.fillScreen(BLACK);
    tft.setCursor(0, 20);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    
    switch (irSendCursor) {
      case 0: // Power
        tft.println("Sending PWR...");
        IrSender.sendNEC(pwr_Address, pwr_Command, 1);                
        break;
      case 1: // Vol UP
        tft.println("Sending VOL+");              
        IrSender.sendNEC(vup_Address, vup_Command, 1);              
        break;
      case 2: // Vol DN
        tft.println("Sending VOL-");
        IrSender.sendNEC(vdn_Address, vdn_Command, 1);                
        break;
      case 3: // OK
        tft.println("Sending OK...");
        IrSender.sendNEC(ok_Address, ok_Command, 1);                
        break;      
      case 4: // BACK
        tft.println("Sending BACK...");
        IrSender.sendNEC(bck_Address, bck_Command, 1);                
        break;
      case 5: // EXIT
        tft.println("Sending EXIT...");
        IrSender.sendNEC(ext_Address, ext_Command, 1);                
        break;   
      case 6: // D-Lt
        tft.println("Sending D-Lt...");
        IrSender.sendNEC(dlt_Address, dlt_Command, 1);                
        break;
      case 7: // D-Rt
        tft.println("Sending D-Rt...");
        IrSender.sendNEC(drt_Address, drt_Command, 1);                
        break;
      case 8: // D-Dn
        tft.println("Sending D-Dn...");
        IrSender.sendNEC(ddn_Address, ddn_Command, 1);                
        break;
      case 9: // D-Up
        tft.println("Sending D-Up...");
        IrSender.sendNEC(dup_Address, dup_Command, 1);                
        break;
      case 10: // Amzn
        tft.println("Sending Amzn...");
        IrSender.sendNEC(amzn_Address, amzn_Command, 1);                
        break;
      case 11: // YT
        tft.println("Sending YT...");
        IrSender.sendNEC(yt_Address, yt_Command, 1);                
        break;
    }
    
    // Visual feedback on SSD1306
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("Signal Sent!");
    display.display();
    
    delay(500); 
    
    lastIrSendCursor = -1; 
  }

  // Scrolling: Up (Button 2), Down (Button 4), Left (Button 5), Right (Button 3)
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      irSendCursor = (irSendCursor - 1 + 12) % 12;
    } else if (lastButton == 4) { // Down
      irSendCursor = (irSendCursor + 1) % 12;
    } else if (lastButton == 3) { // Right - Jump to next column (Jump +4)
      // If we are in the first or second column (indices 0-7), jump to the next column
      if (irSendCursor < 8) {
        irSendCursor = irSendCursor + 4; 
      }
    } else if (lastButton == 5) { // Left - Jump to previous column (Jump -4)
      // If we are in the second or third column (indices 4-11), jump to the previous column
      if (irSendCursor >= 4) {
        irSendCursor = irSendCursor - 4; 
      }
    }
  }

  // --- Only update display if something changed ---
  if (irSendCursor != lastIrSendCursor) {
    
    // --- Draw to SSD1306 (Three Columns) ---
    display.clearDisplay();
    display.setTextSize(0);
    display.setCursor(0, 0);
    
    const char* options[12] = {
      "1.PWR", "2.VUP", "3.VDN", "4.OK", 
      "5.BACK", "6.EXIT", "7.Lft", "8.Rt", 
      "9.Dn", "10.Up", "11.Amz", "12.YT"
    };

    for (int i = 0; i < 12; i++) {
      // Calculate column and row (3 cols, 4 rows)
      int col = i / 4;      
      int row = i % 4;      
      
      // Screen width 128. 128/3 = ~42 pixels per column
      int xPos = col * 42; 
      int yPos = 9 + (row * 10);
      
      display.setCursor(xPos, yPos);
      
      if (i == irSendCursor) {
        display.print(F(">"));
        display.setTextColor(BLACK);
        // Width changed from 64 to 42 to fit 3 columns
        display.fillRect(xPos, yPos - 1, 42, 10, SSD1306_WHITE);
      } else {
        display.print(F(" "));
        display.setTextColor(SSD1306_WHITE);
      }
      
      display.println(options[i]);
    }
    
    
    // --- Draw instructions on Color Screen ---
    tft.fillScreen(BLACK);
    tft.setCursor(0, 0);
    tft.setTextSize(1);
    tft.setTextColor(CYAN);
    tft.println("IR REMOTE");
    tft.setTextSize(0);
    tft.setTextColor(WHITE);
    tft.setCursor(0, 20);
    tft.println("Hold LT to exit");
    tft.setTextColor(GREEN);
    tft.println("Press ENTER to");
    tft.print("Select Code:");
    
    lastIrSendCursor = irSendCursor;
  }
  
  display.display();
  delay(50);
}