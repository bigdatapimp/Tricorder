/*
// -- Tricorder v0.35.2 - By Tony Dillberg
// -- This code is written for the Arduino Nano 33 BLE sense Rev 2
// -- This is for a compact, portable device that utilizes a variety of sensors for basic measurements and readings of various
// -- environmental elements as well as location and direction. It is loosely based on the Tricorder concept from the Star Trek 
// -- franchise.
// --
// -- Minor UI updates for menu and display issues. 
// -- Added ability to switch between F and C for temp. 
// -- Added ability to switch between
// -- inHg and kPa for air pressure.
// -- Updated LED for dB Meter to change based on dB instead of Raw samples.
*/

#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_BMI270_BMM150.h>
#include <PDM.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <TinyGPSPlus.h>



// --- OLED settings ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Button settings (analog ranges on A7) ---
#define BUTTON_E_MIN 775
#define BUTTON_E_MAX 795
#define BUTTON_UP_MIN 525
#define BUTTON_UP_MAX 545
#define BUTTON_RT_MIN 320
#define BUTTON_RT_MAX 350
#define BUTTON_DN_MIN 0
#define BUTTON_DN_MAX 20
#define BUTTON_LT_MIN 665
#define BUTTON_LT_MAX 695

// --- Button press types ---
#define SHORT_PRESS 1
#define LONG_PRESS 2
#define NO_PRESS 0

// --- Long press duration (ms) ---
#define LONG_PRESS_DURATION 1000

// --- Debounce duration (ms) ---
#define DEBOUNCE_DURATION 50

// --- States ---
enum State {
  MENU,
  ATMOSPHERIC,
  NAVIGATION,
  DBMETER,
  METALDETECTOR,
  EMFMETER,
  BATTERY
};

// --- Menu options ---
const char* menuOptions[6] = {"1.Atmo", "2.Nav", "3.dB", "4.MDet", "5.EMF", "6.Batt"};
int menuCursor = 0; // 0 to 5

// --- Global state ---
State currentState = MENU;

// --- Button handling variables ---
int lastButton = 0;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;
int stableButton = 0;
unsigned long stableStart = 0;

// --- Navigation specific ---
#define COMPASS_RADIUS 8
#define CENTER_X 112
#define CENTER_Y 47
#define NEEDLE_LENGTH 5
#define DECLINATION 90.0

// --- Global variables for sensor fusion ---
float fusedHeading = 0.0;
unsigned long lastFusionTime = 0;

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

// --- Battery specific ---
// VIn is read via analog pin A0. Voltage Divider is using two 10kOhm resistors, thus ratio of 2.0
#define VOLTAGE_PIN A0
#define VOLTAGE_DIVIDER_RATIO 2.0  
#define BATTERY_MIN_V 3.0
#define BATTERY_MAX_V 4.2
unsigned long lastBatteryUpdate = 0;


// ---------- Setup Funtion ----------
void setup() {
  Serial.begin(9600);
  //while (!Serial);

  pinMode(A7, INPUT_PULLUP);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

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


  // --- Splash Screen ---
    display.setCursor(0, 8);
    display.println(" ___________________");
    display.setCursor(0, 16);
    display.println(" Tricorder - v0.35.2");
    display.setCursor(0, 30);
    display.println("  By Tony Dillberg");
    display.setCursor(0, 33);
    display.println(" ___________________");
    display.display();
    delay(3500);

  // --- Calibrate Metal Detector baseline ---
  calibrateMetalDetector();

  Serial.println("All initialized.");
  Serial.println("Tricorder - v0.35.2");
  Serial.println("By Tony Dillberg");
}

// ---------- Main Loop ----------
void loop() {
  int button = getPressedButton(analogRead(A7));
  int pressType = handleButtonPress(button);

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
    case BATTERY:  // Added case for battery
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

  // -- Column 2
  display.setCursor(64, 20);
  display.println(menuOptions[3]);
  display.setCursor(64, 30);
  display.println(menuOptions[4]);
  display.setCursor(64, 40);  
  display.println(menuOptions[5]);

  // Highlight cursor
  int col = menuCursor / 3; // 0 or 1
  int row = menuCursor % 3; // 0,1,2
  int x = col * 54 + 10; // Start at 10 or 64
  int y = 20 + row * 10; // 20, 30, 40
  display.setCursor(x - 10, y);
  display.print(">");

  display.display();

  // --- Handle presses ---
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      menuCursor = (menuCursor - 1 + 6) % 6;
    } else if (lastButton == 4) { // Down
      menuCursor = (menuCursor + 1) % 6;
    } else if (lastButton == 5) { // Left
      if (menuCursor >= 3) menuCursor -= 3;
    } else if (lastButton == 3) { // Right
      if (menuCursor < 3) menuCursor += 3;
    } else if (lastButton == 1) { // Enter
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
  if (pressType == LONG_PRESS && lastButton == 5) {
    currentState = MENU;
    return;
  }
  if (pressType == LONG_PRESS && lastButton == 1) {
    calibrateMagnetometer();
    return;
  }

  // --- Read GPS data ---
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // --- Read IMU data for sensor fusion (accelerometer, gyroscope, magnetometer) ---
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    // Normalize accelerometer vector
    float norm_a = sqrt(ax * ax + ay * ay + az * az);
    if (norm_a > 0) {
      ax /= norm_a;
      ay /= norm_a;
      az /= norm_a;
    }

    // Calculate pitch and roll from accelerometer (assuming standard orientation: x forward, y right, z up)
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    float roll = atan2(ay, az);

    // Rotate magnetometer vector to compensate for tilt
    float cos_pitch = cos(pitch);
    float sin_pitch = sin(pitch);
    float cos_roll = cos(roll);
    float sin_roll = sin(roll);
    float mx_h = mx * cos_pitch + mz * sin_pitch;
    float my_h = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;

    // Calculate tilt-compensated magnetic heading
    float mag_heading = atan2(my_h, mx_h) * 180.0 / PI;
    mag_heading += DECLINATION;
    if (mag_heading < 0) mag_heading += 360;
    if (mag_heading >= 360) mag_heading -= 360;

    // Sensor fusion using complementary filter with gyroscope
    if (lastFusionTime == 0) {
      // Initialize fused heading on first run
      fusedHeading = mag_heading;
      lastFusionTime = millis();
    } else {
      unsigned long now = millis();
      float dt = (now - lastFusionTime) / 1000.0;  // Time delta in seconds
      lastFusionTime = now;

      // Integrate gyroscope (z-axis for yaw, assuming gx, gy, gz in rad/s)
      fusedHeading += gz * dt * 180.0 / PI;

      // Normalize fused heading to 0-360
      if (fusedHeading < 0) fusedHeading += 360;
      if (fusedHeading >= 360) fusedHeading -= 360;

      // Complementary filter: blend gyro-integrated heading with magnetometer heading
      float alpha = 0.98;  // High weight on gyro for short-term stability, low on mag for long-term correction
      float diff = mag_heading - fusedHeading;
      // Handle 360-degree wrap-around
      if (diff > 180) diff -= 360;
      if (diff < -180) diff += 360;
      fusedHeading += (1 - alpha) * diff;  // Equivalent to alpha * gyro + (1-alpha) * mag

      // Normalize again
      if (fusedHeading < 0) fusedHeading += 360;
      if (fusedHeading >= 360) fusedHeading -= 360;
    }

    display.clearDisplay();

    // --- Draw compass in bottom-right ---
    display.drawCircle(CENTER_X, CENTER_Y, COMPASS_RADIUS, SSD1306_WHITE);
    display.setCursor(CENTER_X - 3, CENTER_Y - COMPASS_RADIUS - 8);
    display.println("N");
    display.setCursor(CENTER_X - 3, CENTER_Y + COMPASS_RADIUS + 3);
    display.println("S");
    display.setCursor(CENTER_X - COMPASS_RADIUS - 8, CENTER_Y - 3);
    display.println("W");
    display.setCursor(CENTER_X + COMPASS_RADIUS + 2, CENTER_Y - 3);
    display.println("E");

    // Draw needle using fused heading
    float rad = fusedHeading * PI / 180.0;
    int endX = CENTER_X + NEEDLE_LENGTH * sin(rad);
    int endY = CENTER_Y - NEEDLE_LENGTH * cos(rad);
    display.drawLine(CENTER_X, CENTER_Y, endX, endY, SSD1306_WHITE);

    // Display heading at bottom-left (unchanged position)
    display.setCursor(0, 56);
    display.print("Hdg:");
    char buffer[10];
    sprintf(buffer, "%.0f", fusedHeading);
    display.print(buffer);
    display.print((char)247);

    // --- Display GPS and altitude on the left (unchanged) ---
    display.setCursor(0, 16);
    if (gps.location.isValid()) {
      // Latitude in DMS
      int lat_deg = abs(gps.location.rawLat().deg);
      double lat_min_full = gps.location.rawLat().billionths / 1000000000.0 * 60.0;
      int lat_min = (int)lat_min_full;
      double lat_sec = (lat_min_full - lat_min) * 60.0;
      char lat_dir = gps.location.rawLat().negative ? 'S' : 'N';
      display.print("Lat:");
      display.print(lat_deg);
      display.print((char)247); // Degree symbol (°)
      display.print(lat_min);
      display.print("'");
      display.print(lat_sec, 2);
      display.print("''");
      display.println(lat_dir);

      display.setCursor(0, 26);
      // Longitude in DMS
      int lon_deg = abs(gps.location.rawLng().deg);
      double lon_min_full = gps.location.rawLng().billionths / 1000000000.0 * 60.0;
      int lon_min = (int)lon_min_full;
      double lon_sec = (lon_min_full - lon_min) * 60.0;
      char lon_dir = gps.location.rawLng().negative ? 'W' : 'E';
      display.print("Lon:");
      display.print(lon_deg);
      display.print((char)247); // Degree symbol (°)
      display.print(lon_min);
      display.print("'");
      display.print(lon_sec, 2);
      display.print("''");
      display.println(lon_dir);

      display.setCursor(0, 36);
      // GPS Altitude in feet
      display.print("GPS Alt:");
      display.print((int)gps.altitude.feet());
      display.println("ft");
    } else {
      display.println("GPS: No Fix");
      display.println("");
      display.println("");
    }

    display.setCursor(0, 46);
    // Barometric Altitude in feet
    float pressure = BARO.readPressure(MILLIBAR);
    float baro_alt_ft = 44330.0 * (1.0 - pow(pressure / 1013.25, 1.0 / 5.255)) * 3.28084;
    display.print("Baro Alt:");
    display.print((int)baro_alt_ft);
    display.print("ft");

    display.display();
  }

  delay(100);
}

//------------- dB Meter code ---------------
void runDBMeter(int pressType) {

  if (pressType == LONG_PRESS && lastButton == 5) {
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
    display.setCursor(0, 9);
    display.println(F("Sound Level"));
    int barWidth = map(dB, 0, 160, 0, SCREEN_WIDTH - 20);
    display.fillRect(0, 20, barWidth, 10, SSD1306_WHITE);
    display.setCursor(0, 35);
    display.setTextSize(2);
    char buffer[10];
    sprintf(buffer, "%.1f", dB);
    display.print(buffer);
    display.print(" dB");
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

// ------------- Metal Detector code ----------
void runMetalDetector(int pressType) {

  if (pressType == LONG_PRESS && lastButton == 5) {
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

// --------- Battery Monitor ---------
void runBattery(int pressType) {
  if (pressType == LONG_PRESS && lastButton == 5) {
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