//------------- dB Meter code ---------------
void DBMeter(int pressType) {

  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = MET_MENU;
    return;
  }

  
  if (samplesRead) {
    long long sum = 0;
    int maxSample = 0;
    for (int i = 0; i < samplesRead; i++) {
      sum += (long long)sampleBuffer[i] * sampleBuffer[i];
      if (abs(sampleBuffer[i]) > maxSample) maxSample = abs(sampleBuffer[i]);
    }
    double rms = sqrt((double)sum / samplesRead);  // Cast to double for precision
    float normalizedRMS = rms / 32768.0;
    
    float dB;
    if (normalizedRMS <= 0.0) {
      dB = 0.0;  // Prevent nan from log10(0)
    } else {
      dB = 20.0 * log10(normalizedRMS / REFERENCE_RMS);
    }
    if (dB < 0) dB = 0;  // Floor at 0 dB


    // Update peak dB
    if (dB > max_dB) {
      max_dB = dB;
    }

    if (dB >= 95) {
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDG, HIGH);
      digitalWrite(LEDB, HIGH);
    } else if (dB >= 70) {
      digitalWrite(LEDB, HIGH);
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDG, LOW);
    } else {
      digitalWrite(LEDG, LOW);
      digitalWrite(LEDR, HIGH);
      digitalWrite(LEDB, HIGH);
    }

    display.clearDisplay(); 
    display.setTextSize(2);   
    display.setCursor(0, 9);
    display.println(F("Sound LVL"));
    
        
    int barWidth = map(dB, 0, 160, 0, SCREEN_WIDTH - 20);
   
    if (dB >= 95) {
      tft.fillRect(0, 20, 96, 10, WHITE);
      tft.fillRect(0, 20, barWidth, 10, RED);
    } else if (dB >= 70) {
      tft.fillRect(0, 20, 96, 10, WHITE);
      tft.fillRect(0, 20, barWidth, 10, YELLOW);
    } else {
      tft.fillRect(0, 20, 96, 10, WHITE);
      tft.fillRect(0, 20, barWidth, 10, GREEN);
    }
    

    tft.setCursor(0, 35);
    tft.setTextSize(2);
    tft.setTextColor(BLUE, BLACK);
    char buffer[10];
    sprintf(buffer, "%.1f", dB);
    tft.print(buffer);
    tft.print(" dB");

    display.setTextSize(1);
    display.setCursor(0, 55);
    display.print(F("Raw:"));
    display.print(maxSample);
    
        // Add peak dB to bottom right
    display.setCursor(55, 55);  // Positioned to the right; adjust if needed for exact alignment
    display.print(F("Max:"));
    sprintf(buffer, "%.1f", max_dB);
    display.print(buffer);
    display.print("dB");

    display.display();

    samplesRead = 0;
  }
  
}

