/*
// -- Tricorder v0.4 - By Tony Dillberg
// -- This code is written for the Arduino Nano 33 BLE sense Rev 2
// -- This is for a compact, portable device that utilizes a variety of sensors for basic measurements and readings of various
// -- environmental elements as well as location and direction. It is loosely based on the Tricorder concept from the Star Trek 
// -- franchise.
// --
// -- Added Beeper for start tones
// -- Added IR receive
// -- Added 2nd display SSD1331
*/

#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_BMI270_BMM150.h>
#include <PDM.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SSD1331.h>
#include <Adafruit_GFX.h>
#include <TinyGPSPlus.h>
#include <SPI.h>



// --- OLED settings - Screen 1---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- OLED settings - Screen 2---
// Pin definitions
#define CS_PIN   10
#define DC_PIN   9
#define RST_PIN  8
// RGB565 Color definitions
#define BLACK    0x0000
#define WHITE    0xFFFF
#define RED      0xF800
#define GREEN    0x07E0
#define BLUE     0x001F
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define ORANGE   0xFC00
// Initialize display (96x64 resolution is default for SSD1331)
Adafruit_SSD1331 tft = Adafruit_SSD1331(CS_PIN, DC_PIN, RST_PIN);


// --- Button settings (analog ranges on A7) ---
#define BUTTON_E_MIN 770
#define BUTTON_E_MAX 805
#define BUTTON_UP_MIN 520
#define BUTTON_UP_MAX 550
#define BUTTON_RT_MIN 315
#define BUTTON_RT_MAX 355
#define BUTTON_DN_MIN 0
#define BUTTON_DN_MAX 25
#define BUTTON_LT_MIN 660
#define BUTTON_LT_MAX 700

// --- Button press types ---
#define SHORT_PRESS 1
#define LONG_PRESS 2
#define NO_PRESS 0

// --- Long press duration (ms) ---
#define LONG_PRESS_DURATION 1000

// --- Debounce duration (ms) ---
#define DEBOUNCE_DURATION 50

// --- Beeper settings ---
#define BEEPER_PIN 6

// --- States ---
enum State {
  MENU,
  ATMOSPHERIC,
  NAVIGATION,
  DBMETER,
  METALDETECTOR,
  EMFMETER,
  IRRECEIVER,
  BATTERY
};

// --- Menu options ---
const char* menuOptions[7] = {"1.Atmo", "2.Nav", "3.dB", "4.MDet", "5.EMF", "6.irRec", "7.Batt"};
int menuCursor = 0; // 0 to 6

// --- Global state ---
State currentState = MENU;

// --- Button handling variables ---
int lastButton = 0;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;
int stableButton = 0;
unsigned long stableStart = 0;

// --- Navigation specific ---
#define COMPASS_RADIUS 12
#define CENTER_X 48
#define CENTER_Y 25
#define NEEDLE_LENGTH 9
#define DECLINATION 183.44 

// --- Global variables for sensor fusion ---
float fusedHeading = 0.0;
unsigned long lastFusionTime = 0;
bool useDMS = true; // true = Degrees Minutes Seconds, false = Decimal Degrees
bool isLevel = false; // Tracks if device is level enough for accurate heading

// --- GPS specific ---
TinyGPSPlus gps;


// --- dB Meter specific ---
short sampleBuffer[256];
volatile int samplesRead;
#define REFERENCE_RMS 0.0001
float max_dB = 0.0; // Variable to track peak dB

// --- Metal Detector specific ---
#define CALIBRATION_SAMPLES 100
#define DETECTION_THRESHOLD 20.0
float baselineMagnitude = 0.0;

// --- EMF Meter specific ---
#define MAX_EMF 200.0

// --- Atmospheric specific ---
unsigned long lastAtmosphericUpdate = 0;
bool useCelsius = false;       // false = Fahrenheit, true = Celsius
bool useKPa = false;            // false = inHg, true = kPa

// --- irReceiver specific ---
const int IR_RECV_PIN = 2;
bool irDataReceived = false;        // Flag to track if data has been received
uint32_t storedIrData = 0;
uint8_t storedIrAddr = 0;
uint8_t storedIrCmd = 0;


// --- Battery specific ---
// VIn is read via analog pin A0. Voltage Divider is using two 10kOhm resistors, thus ratio of 2.0
#define VOLTAGE_PIN A0
#define VOLTAGE_DIVIDER_RATIO 2.0  
#define BATTERY_MIN_V 3.35
#define BATTERY_MAX_V 4.25
unsigned long lastBatteryUpdate = 0;





// ---------- Setup Funtion ----------
void setup() {
  Serial.begin(9600);
  //while (!Serial);

  pinMode(A7, INPUT_PULLUP); // input pin for buttons
  pinMode(IR_RECV_PIN, INPUT); // input pin for IR Receiver
  
  // Initialize OLED - Screen 1
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize OLED - Screen 2
  /*if (!tft.begin(8000000)) {
    Serial.println(F("SSD1331 allocation failed"));
    while (true);
  }*/
  tft.begin(8000000);
  tft.setRotation(0); // Set rotation (0-3)


  // ------- Initialize sensors used across functions ---- 
  if (!HS300x.begin()) {
    Serial.println("Failed to initialize HS300x sensor!");
  }
  if (!BARO.begin()) {
    Serial.println("Failed to initialize LPS22HB sensor!");
  }
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }

  // --- Initialize PDM for dB Meter ---
  PDM.onReceive(onPDMdata);
  PDM.setGain(70);
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
  }

  // --- Initialize Serial1 for GPS ---
  Serial1.begin(9600);

  // --- Splash Screen - Screen 2---
    tft.fillScreen(BLACK);
    tft.setCursor(0, 8);
    tft.setTextColor(MAGENTA);
    tft.println("________________");
    tft.setTextColor(YELLOW);
    tft.setCursor(0, 16);
    tft.println("Tricorder - v0.4");
    tft.setTextColor(BLUE);
    tft.setCursor(0, 30);
    tft.println("By Tony Dillberg");
    tft.setCursor(0, 33);
    tft.println("________________");
    

  // --- Splash Screen - Screen 1---
    display.setCursor(0, 8);
    display.println(" ___________________");
    display.setCursor(0, 16);
    display.println("  Tricorder - v0.4");
    display.setCursor(0, 30);
    display.println("  By Tony Dillberg");
    display.setCursor(0, 33);
    display.println(" ___________________");
    display.display();
    delay(3500);

    // clear Screen 2
    tft.fillScreen(BLACK);

  // --- Initialize Beeper ---
  pinMode(BEEPER_PIN, OUTPUT);
  playStartupTone();

  // --- Calibrate Metal Detector baseline ---
  calibrateMetalDetector();

  Serial.println("All initialized.");
  Serial.println("Tricorder - v0.4");
  Serial.println("By Tony Dillberg");
}




// ---------- Main Loop ----------
void loop() {
  int button = getPressedButton(analogRead(A7));
  int pressType = handleButtonPress(button);
  //tft.fillScreen(BLACK);

  switch (currentState) {
    case MENU:
      handleMenu(pressType);
      break;
    case ATMOSPHERIC:
      runAtmospheric(pressType);
      break;
    case NAVIGATION:
      runNavigation(pressType);
      break;
    case DBMETER:
      runDBMeter(pressType);
      break;
    case METALDETECTOR:
      runMetalDetector(pressType);
      break;
    case EMFMETER:
      runEMFMeter(pressType);
      break;
    case IRRECEIVER:
      irReceiver(pressType);
      break;  
    case BATTERY:  
      runBattery(pressType);
      break;
  }
}

void handleMenu(int pressType) {
  // -- Display menu in two columns
  display.clearDisplay();
  display.setCursor(0, 9);
  display.println(F("Select Function"));

  // -- Column 1
  display.setCursor(10, 20);
  display.println(menuOptions[0]);
  display.setCursor(10, 30);
  display.println(menuOptions[1]);
  display.setCursor(10, 40);
  display.println(menuOptions[2]);
  display.setCursor(10, 50);
  display.println(menuOptions[3]);

  // -- Column 2
  display.setCursor(64, 20);
  display.println(menuOptions[4]);
  display.setCursor(64, 30);
  display.println(menuOptions[5]);
  display.setCursor(64, 40);  
  display.println(menuOptions[6]);

  // Highlight cursor
  int col = menuCursor / 4; // 0 or 1
  int row = menuCursor % 4; // 0,1,2,3
  int x = col * 54 + 10; // Start at 10 or 64
  int y = 20 + row * 10; // 20, 30, 40
  display.setCursor(x - 10, y);
  display.print(">");

  display.display();

  // --- Handle presses ---
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      menuCursor = (menuCursor - 1 + 7) % 7;  // Changed 6 to 7
    } else if (lastButton == 4) { // Down
      menuCursor = (menuCursor + 1) % 7;     // Changed 6 to 7
    } else if (lastButton == 5) { // Left
      if (menuCursor >= 4) menuCursor -= 4;   // Changed 3 to 4
    } else if (lastButton == 3) { // Right
      if (menuCursor < 4) menuCursor += 4;    // Changed 3 to 4
    } else if (lastButton == 1) { // Enter
      //playClimbingTones(); // Play the function start tone
      currentState = (State)(menuCursor + 1); // MENU=0, so +1
    }
  }
  
}



// ----------- Atmospheric code ------------
void runAtmospheric(int pressType) {

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
      currentState = MENU;
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




// ----------- Navigation code ---------
void runNavigation(int pressType) {
  // --- Handle Button Inputs ---
  if (pressType == LONG_PRESS) {
    if (lastButton == 5) { // Left: Exit to Menu
      tft.fillScreen(BLACK);
      currentState = MENU;
      return;
    }
    if (lastButton == 1) { // Enter: Calibrate Magnetometer
      calibrateMagnetometer();
      return;
    }
    if (lastButton == 3) { // Right: Toggle DMS / Decimal Degrees
      useDMS = !useDMS;
    }
  }

  // --- Read GPS data ---
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // --- Read IMU data ---
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    // --- Calculate total acceleration magnitude ---
    // Should be ~1.0g when stationary. Large deviations indicate movement.
    float totalAccel = sqrt(ax * ax + ay * ay + az * az);
    
    // --- Calculate pitch and roll from accelerometer ---
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    float roll = atan2(ay, az);

    // --- Check if device is level enough for accurate heading ---
    // If pitch or roll exceeds ~15 degrees, heading becomes unreliable
    float pitchDeg = pitch * 180.0 / PI;
    float rollDeg = roll * 180.0 / PI;
    
    // Only consider level if acceleration magnitude is close to 1g (not moving)
    // AND pitch/roll are within reasonable bounds
    isLevel = (totalAccel > 0.9 && totalAccel < 1.1) && 
              (fabs(pitchDeg) < 20.0) && 
              (fabs(rollDeg) < 20.0);

    // --- TILT-COMPENSATED MAGNETOMETER READING ---
    // Rotate magnetometer readings to horizontal plane based on pitch and roll
    float cos_pitch = cos(pitch);
    float sin_pitch = sin(pitch);
    float cos_roll = cos(roll);
    float sin_roll = sin(roll);

    // Tilt-compensated magnetometer components
    float mx_comp = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
    float my_comp = my * cos_roll - mz * sin_roll;

    // --- Calculate raw magnetic heading ---
    float mag_heading = atan2(-my_comp, mx_comp) * 180.0 / PI;
    mag_heading += DECLINATION;
    
    // Normalize to 0-360
    while (mag_heading < 0) mag_heading += 360;
    while (mag_heading >= 360) mag_heading -= 360;

    // --- Sensor Fusion with Complementary Filter ---
    if (lastFusionTime == 0) {
      fusedHeading = mag_heading;
      lastFusionTime = millis();
    } else {
      unsigned long now = millis();
      float dt = (now - lastFusionTime) / 1000.0; // Time delta in seconds
      lastFusionTime = now;

      // Integrate gyroscope for short-term stability
      // Note: gz is in degrees/sec on this sensor
      fusedHeading += gz * dt;

      // Normalize fused heading
      while (fusedHeading < 0) fusedHeading += 360;
      while (fusedHeading >= 360) fusedHeading -= 360;

      // Complementary filter: blend gyro with magnetometer
      // When level: trust magnetometer more (0.95 gyro, 0.05 mag)
      // When tilted: trust gyro only (1.0 gyro, 0.0 mag) to prevent wild swings
      float alpha = isLevel ? 0.98 : 1.0; 
      
      float diff = mag_heading - fusedHeading;
      
      // Handle 360-degree wrap-around for difference calculation
      if (diff > 180) diff -= 360;
      if (diff < -180) diff += 360;
      
      fusedHeading += (1 - alpha) * diff;

      // Normalize again
      while (fusedHeading < 0) fusedHeading += 360;
      while (fusedHeading >= 360) fusedHeading -= 360;
    }

    // --- EXTREME SMOOTHING FILTER ---
    // This is a separate low-pass filter specifically for display
    // It creates a very "heavy" feel that ignores rapid changes
    static float displayHeading = 0;
    
    if (displayHeading == 0) {
      displayHeading = fusedHeading;
    }
    
    // Smoothing factor: 0.95 = very smooth, 0.80 = more responsive
    // When not level, increase smoothing dramatically to hide jitter
    float smoothing = isLevel ? 0.80 : 0.95;
    displayHeading = displayHeading * smoothing + fusedHeading * (1.0 - smoothing);

    // Normalize display heading
    while (displayHeading < 0) displayHeading += 360;
    while (displayHeading >= 360) displayHeading -= 360;

    // --- Draw to OLEDs ---
    display.clearDisplay();
    

    // --- Draw Compass Circle ---
    tft.drawCircle(CENTER_X, CENTER_Y, COMPASS_RADIUS, MAGENTA);

    // --- Draw Cardinal Direction Labels ---
    tft.setTextSize(1);
    tft.setCursor(CENTER_X - 3, CENTER_Y - COMPASS_RADIUS - 8);
    tft.setTextColor(RED);
    tft.println("N");
    tft.setCursor(CENTER_X - 3, CENTER_Y + COMPASS_RADIUS + 3);
    tft.setTextColor(YELLOW);
    tft.println("S");
    tft.setCursor(CENTER_X - COMPASS_RADIUS - 8, CENTER_Y - 3);
    tft.println("W");
    tft.setCursor(CENTER_X + COMPASS_RADIUS + 2, CENTER_Y - 3);
    tft.println("E");

        // Clear just the needle area first (small rectangle)
    tft.fillRect(CENTER_X - NEEDLE_LENGTH, CENTER_Y - NEEDLE_LENGTH, NEEDLE_LENGTH * 2, NEEDLE_LENGTH * 2, BLACK);

    // --- Draw Compass Needle ---
    float rad = displayHeading * PI / 180.0;
    int endX = CENTER_X + NEEDLE_LENGTH * sin(rad);
    int endY = CENTER_Y - NEEDLE_LENGTH * cos(rad);
    tft.drawLine(CENTER_X, CENTER_Y, endX, endY, ORANGE);

    // --- Draw Level Indicator ---
    // Shows "LVL" when level, "TILT" when tilted
    display.setCursor(105, 20);
    display.setTextSize(1);
    if (isLevel) {
      display.println("LVL");
    } else {
      display.println("TLT");
    }

    // --- Display Heading Text ---
    tft.fillRect(23, 55, 80, 10, BLACK);  // Clear old text
    tft.setTextSize(1);
    tft.setTextColor(BLUE);
    tft.setCursor(0, 55);
    tft.print("Hdg:");
    char buffer[10];
    sprintf(buffer, "%.0f", displayHeading);
    tft.print(buffer);
    tft.print((char)247); // Degree symbol

    // --- Display GPS Coordinates ---
    display.setCursor(0, 16);
    if (gps.location.isValid()) {
      if (useDMS) {
        // --- DMS Format ---
        int lat_deg = abs(gps.location.rawLat().deg);
        double lat_min_full = gps.location.rawLat().billionths / 1000000000.0 * 60.0;
        int lat_min = (int)lat_min_full;
        double lat_sec = (lat_min_full - lat_min) * 60.0;
        char lat_dir = gps.location.rawLat().negative ? 'S' : 'N';
        
        display.print("Lat:");
        display.print(lat_deg);
        display.print((char)247);
        display.print(lat_min);
        display.print("'");
        display.print(lat_sec, 1);
        display.print("''");
        display.println(lat_dir);

        display.setCursor(0, 26);
        int lon_deg = abs(gps.location.rawLng().deg);
        double lon_min_full = gps.location.rawLng().billionths / 1000000000.0 * 60.0;
        int lon_min = (int)lon_min_full;
        double lon_sec = (lon_min_full - lon_min) * 60.0;
        char lon_dir = gps.location.rawLng().negative ? 'W' : 'E';
        
        display.print("Lon:");
        display.print(lon_deg);
        display.print((char)247);
        display.print(lon_min);
        display.print("'");
        display.print(lon_sec, 1);
        display.print("''");
        display.println(lon_dir);
      } else {
        // --- Decimal Degrees Format ---
        display.print("Lat: ");
        display.println(gps.location.lat(), 5);
        
        display.setCursor(0, 26);
        display.print("Lon: ");
        display.println(gps.location.lng(), 5);
      }

      display.setCursor(0, 36);
      display.print("GPS Alt:");
      display.print((int)gps.altitude.feet());
      display.println("ft");
    } else {
      display.println("GPS: No Fix");
      display.println("");
      display.println("");
    }

    display.setCursor(0, 46);
    float pressure = BARO.readPressure(MILLIBAR);
    float baro_alt_ft = 44330.0 * (1.0 - pow(pressure / 1013.25, 1.0 / 5.255)) * 3.28084;
    display.print("Baro Alt:");
    display.print((int)baro_alt_ft);
    display.print("ft");

    display.display();
  }

  delay(50);
}




//------------- dB Meter code ---------------
void runDBMeter(int pressType) {

  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = MENU;
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
    //tft.fillRect(0, 20, barWidth, 10, WHITE);
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
    
    //tft.fillRect(0, 20, 96, 10, WHITE);
    //tft.fillRect(0, 20, barWidth, 10, GREEN);

    tft.fillRect(0, 35, 60, 20, BLACK);   // Clear dB text area
    tft.setCursor(0, 35);
    tft.setTextSize(2);
    tft.setTextColor(BLUE);
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
  //delay(50);
}




// ------------- Metal Detector code ----------
void runMetalDetector(int pressType) {

  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = MENU;
    return;
  }
  if (pressType == LONG_PRESS && lastButton == 1) {
    calibrateMetalDetector();
    return;
  }

  
  float x, y, z;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    float currentMagnitude = sqrt(x * x + y * y + z * z);
    float deviation = abs(currentMagnitude - baselineMagnitude);
    bool metalDetected = (deviation > DETECTION_THRESHOLD);

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
    int barWidth = map(deviation, 0, 200, 0, SCREEN_WIDTH - 20);
    display.fillRect(0, 40, barWidth, 10, SSD1306_WHITE);
    display.setCursor(0, 55);
    if (metalDetected) {
      display.println(F("METAL DETECTED!"));
    } else {
      display.println(F("No metal"));
    }
    display.display();
  }

  delay(500);
}





// ---------- EMF Meter code ----------
void runEMFMeter(int pressType) {

  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = MENU;
    return;
  }


  float x, y, z;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    float emfMagnitude = sqrt(x * x + y * y + z * z);

    display.clearDisplay();
    display.setCursor(0, 9);
    display.println(F("EMF Meter"));
    display.setCursor(0, 20);
    display.print(F("EMF: "));
    display.print(emfMagnitude, 1);
    display.println(F(" uT"));
    int barWidth = map(emfMagnitude, 0, MAX_EMF, 0, SCREEN_WIDTH - 20);
    if (barWidth > 0) {
      display.fillRect(0, 30, barWidth, 10, SSD1306_WHITE);
    }
    display.setCursor(0, 45);
    display.print(F("0"));
    display.setCursor(SCREEN_WIDTH - 50, 45);
    display.print(MAX_EMF, 0);
    display.print(F("uT"));
    display.display();
  }

  delay(500);

}




// --------- IR Receiver code ----------
void irReceiver(int pressType) {
  
  // --- Handle Exit (Long Press Left) ---
  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    display.clearDisplay();
    display.display();
    irDataReceived = false;
    currentState = MENU;
    return;
  }

  // --- Wait for signal (LOW) ---
  if (digitalRead(IR_RECV_PIN) == LOW) {
    Serial.println("--- Signal Detected ---");
    
    // Wait for HIGH (end of header mark)
    while (digitalRead(IR_RECV_PIN) == LOW) {}
    
    // Measure header space
    unsigned long start = micros();
    while (digitalRead(IR_RECV_PIN) == HIGH) {}
    unsigned long headerSpace = micros() - start;
    
    Serial.print("Header Space: ");
    Serial.println(headerSpace);
    
    // Read 32 bits
    Serial.print("Data: ");
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
        Serial.print("1");
      } else {
        Serial.print("0");
      }
    }
    Serial.println("");
    
    Serial.print("Value: 0x");
    Serial.println(data, HEX);
    
    // Extract parts
    uint8_t addr = (data >> 24) & 0xFF;
    uint8_t cmd = (data >> 8) & 0xFF;
    
    Serial.print("Address: 0x");
    Serial.println(addr, HEX);
    Serial.print("Command: 0x");
    Serial.println(cmd, HEX);
    
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
    
    /*tft.setTextColor(YELLOW);
    tft.setCursor(0, 48);
    tft.print("Bits: ");
    for (int i = 31; i >= 0; i--) {
      tft.print((storedIrData >> i) & 1);
      if (i == 16) tft.print(" ");
    }*/
    
    /*// Display on SSD1306
    display.clearDisplay();
    display.setCursor(0, 9);
    display.println(F("IR Signal"));
    display.setCursor(0, 20);
    display.print(F("Addr: 0x"));
    display.println(storedIrAddr, HEX);
    display.setCursor(0, 30);
    display.print(F("Cmd: 0x"));
    display.println(storedIrCmd, HEX);
    display.setCursor(0, 40);
    display.print(F("Raw: 0x"));
    display.println(storedIrData, HEX);
    display.display(); */
    
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




// --------- Battery Monitor code ---------
void runBattery(int pressType) {
  if (pressType == LONG_PRESS && lastButton == 5) {
    tft.fillScreen(BLACK);
    currentState = MENU;
    return;
  }

  // Update every 2 seconds, similar to atmospheric
  if (millis() - lastBatteryUpdate > 2000) {
    // Read voltage from VIn pin (assumed A0 with divider)
    int adcValue = analogRead(VOLTAGE_PIN);
    float voltage = (adcValue / 1023.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;

    // Estimate battery percentage for 1S LiPo (3.0V = 0%, 4.2V = 100%)
    float percentage = ((voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V)) * 100.0;
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;

    display.clearDisplay();
    display.setCursor(0, 9);
    display.println(F("Battery Status"));
    display.setCursor(0, 20);
    display.print(F("Voltage: "));
    display.print(voltage, 2);
    display.println(F(" V"));
    display.setCursor(0, 35);
    display.print(F("Battery: "));
    display.print(percentage, 1);
    display.println(F(" %"));

    // Optional: Add a battery bar
    int barWidth = map(percentage, 0, 100, 0, SCREEN_WIDTH - 20);
    display.fillRect(0, 50, barWidth, 10, SSD1306_WHITE);

    display.display();
    lastBatteryUpdate = millis();
  }
  
  

}


// ---------- Button Press Handling ----------
int getPressedButton(int value) {
  if (value >= BUTTON_E_MIN && value <= BUTTON_E_MAX) return 1;
  if (value >= BUTTON_UP_MIN && value <= BUTTON_UP_MAX) return 2;
  if (value >= BUTTON_RT_MIN && value <= BUTTON_RT_MAX) return 3;
  if (value >= BUTTON_DN_MIN && value <= BUTTON_DN_MAX) return 4;
  if (value >= BUTTON_LT_MIN && value <= BUTTON_LT_MAX) return 5;
  return 0;
}

int handleButtonPress(int currentButton) {
  if (currentButton != 0) {
    if (stableButton == 0) {
      stableButton = currentButton;
      stableStart = millis();
    } else if (stableButton == currentButton && millis() - stableStart >= DEBOUNCE_DURATION) {
      // Button is stable, now handle press
      if (!buttonPressed) {
        buttonPressed = true;
        buttonPressTime = millis();
        lastButton = currentButton;
      } else {
        if (millis() - buttonPressTime >= LONG_PRESS_DURATION) {
          buttonPressed = false;
          stableButton = 0;
          return LONG_PRESS;
        }
      }
    } else if (stableButton != currentButton) {
      // Button changed, reset debounce
      stableButton = currentButton;
      stableStart = millis();
    }
  } else {
    if (buttonPressed) {
      buttonPressed = false;
      unsigned long duration = millis() - buttonPressTime;
      if (duration < LONG_PRESS_DURATION) {
        stableButton = 0;
        return SHORT_PRESS;
      }
    }
    stableButton = 0;
  }
  return NO_PRESS;
}


// ---------- Calibration ----------
void calibrateMagnetometer() {
  Serial.println("Calibrating magnetometer... Rotate the device in all directions.");
  display.clearDisplay();
  display.setCursor(0, 15);
  display.println(F("Calibrating..."));
  display.println(F("Rotate device"));
  display.display();

  float minX = 1000, maxX = -1000, minY = 1000, maxY = -1000, minZ = 1000, maxZ = -1000;
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    if (IMU.magneticFieldAvailable()) {
      float x, y, z;
      IMU.readMagneticField(x, y, z);
      minX = min(minX, x); maxX = max(maxX, x);
      minY = min(minY, y); maxY = max(maxY, y);
      minZ = min(minZ, z); maxZ = max(maxZ, z);
    }
    delay(100);
  }

  float offsetX = (maxX + minX) / 2.0;
  float offsetY = (maxY + minY) / 2.0;
  float offsetZ = (maxZ + minZ) / 2.0;

  Serial.print("Offsets: X=");
  Serial.print(offsetX);
  Serial.print(" Y=");
  Serial.print(offsetY);
  Serial.print(" Z=");
  Serial.println(offsetZ);

  display.clearDisplay();
  display.setCursor(0, 15);
  display.println(F("Calibration Done"));
  display.display();
  delay(2000);
}

void calibrateMetalDetector() {
  float sum = 0.0;
  int count = 0;
  Serial.println("Calibrating baseline... Keep away from metal.");
  display.clearDisplay();
  display.setCursor(0, 16);
  display.println(F("Calibrating..."));
  display.display();

  while (count < CALIBRATION_SAMPLES) {
    if (IMU.magneticFieldAvailable()) {
      float x, y, z;
      IMU.readMagneticField(x, y, z);
      float magnitude = sqrt(x * x + y * y + z * z);
      sum += magnitude;
      count++;
      delay(100);
    }
  }

  baselineMagnitude = sum / CALIBRATION_SAMPLES;
  Serial.print("Baseline calibrated: ");
  Serial.print(baselineMagnitude);
  Serial.println(" uT");
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

// --- Beeper Helper Functions ---

void playTone(int frequency, int duration) {
  // Play a tone on the beeper pin
  tone(BEEPER_PIN, frequency);
  delay(duration);
  noTone(BEEPER_PIN);
  delay(50); // Short pause between notes
}

void playStartupTone() {
  // Play a nice major chord arpeggio (C -> E -> G -> C)
  playTone(65, 100);  // C2 - Click
  playTone(65, 100);  // C2 - Click
  playTone(65, 100);  // C2 - Click
  playTone(294, 400); // D4
  playTone(392, 100);  // G4
  playTone(523, 400);  // C5
  playTone(494, 200);  // B4
  playTone(392, 100); // G4
  playTone(330, 100);  // E4
  playTone(440, 100);  // A4
  playTone(587, 600); // D5
}

void playClimbingTones() {
  // Play 3 short beeps climbing in pitch
  playTone(440, 80);  // A4
  playTone(554, 80);  // C#5
  playTone(659, 100); // E5
}