/*
// -- Tricorder v0.6 - By Tony Dillberg
// -- This code is written for the Arduino Nano 33 BLE sense Rev 2
// -- This is for a compact, portable device that utilizes a variety of sensors for basic measurements and readings of various
// -- environmental elements as well as location and direction. It is loosely based on the Tricorder concept from the Star Trek 
// -- franchise.
// --
// -- Menus/GPS clock now redraw only when something changes (fixes laggy buttons)
// -- Button handler rewritten (debounced press AND release, no phantom short press after a long press)
// -- Compass rewritten: calibration is now applied, tilt-compensated, wrap-safe smoothing
// -- 
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
#define IR_SEND_PIN 3
#include <IRremote.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <ArduinoBLE.h>

// --- FW Ver ---
float FWver = 0.6;

// --- OLED settings - Screen 1---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- TFT settings - Screen 2---
#define CS_PIN 10
#define DC_PIN 9
#define RST_PIN 8
// RGB565 Color definitions
#define BLACK 0x0000
#define WHITE 0xFFFF
#define RED 0xF800
#define PINK 0xe8b8
#define PURPLE 0x8010
#define GREEN 0x07E0
#define GREEN2 0x1b62
#define BLUE 0x001F
#define BLUE2 0x6433
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define GOLD 0xd5c6
#define ORANGE 0xFC00

// Initialize display (96x64 res default for SSD1331)
Adafruit_SSD1331 tft = Adafruit_SSD1331(CS_PIN, DC_PIN, RST_PIN);

// --- Button settings (analog ranges on A7) ---
#define BUTTON_E_MIN 750
#define BUTTON_E_MAX 825
#define BUTTON_UP_MIN 500
#define BUTTON_UP_MAX 570
#define BUTTON_RT_MIN 295
#define BUTTON_RT_MAX 375
#define BUTTON_DN_MIN 0
#define BUTTON_DN_MAX 45
#define BUTTON_LT_MIN 640
#define BUTTON_LT_MAX 725

// --- Button press types ---
#define SHORT_PRESS 1
#define LONG_PRESS 2
#define NO_PRESS 0

#define LONG_PRESS_DURATION 1000  // --- Long press duration (ms) ---
#define DEBOUNCE_DURATION 25      // --- Debounce duration (ms) --- 50 default

// --- Beeper settings ---
#define BEEPER_PIN 6

// --- Enhanced State enum for new menu structure ---
enum State {
  MAIN_MENU,
  GEO_MENU,
  ENV_MENU,
  COMMS_MENU,
  UTIL_MENU,
  SYS_MENU,
  METALDETECTOR,
  EMFMETER,
  NAVIGATION,
  ATMOSPHERIC,
  DBMETER,
  SPECTRUM,
  IRRECEIVER,
  IRSEND,
  BATTERY,
  SYSINFO,
  REBOOT,
  TIMEZONE,
  DECLINATION_SETUP,
  SHOTTIMER,
  STOPWATCH,
  COUNTDOWNTIMER,
  BLUETOOTH_SCREEN,
  AUDIO_STREAM,
  SEISMOMETER
};


int mainMenuCursor = 0;  // 0-4 for main menu
int subMenuCursor = 0;   // For submenu cursors

// --- Global state ---
State currentState = MAIN_MENU;

// --- Button handling variables ---
int lastButton = 0;

// --- Menu redraw control ---
bool menuDirty = true;            // set true whenever a menu needs to be redrawn
State lastLoopState = MAIN_MENU;  // used by loop() to detect state changes

// --- Navigation specific ---
#define COMPASS_RADIUS 14
#define CENTER_X 68
#define CENTER_Y 25
#define NEEDLE_LENGTH 9
// Magnetic declination: east is POSITIVE, west is NEGATIVE. Middle Tennessee is west of the zero line,
// so this should be NEGATIVE (a few degrees). Check your exact value at ngdc.noaa.gov/geomag/calculators
#define DECLINATION -3.46  // Local Compass Magnetic Declination
#define COMPASS_TAU 0.25  // heading smoothing time constant in seconds (bigger = smoother but slower)
#define ACCEL_ALPHA 0.2   // accelerometer low-pass (0-1, smaller = smoother)
#define MAG_CAL_MS 15000  // how long the magnetometer calibration samples for

// Axis fix-ups, in case the magnetometer axes don't line up with the accelerometer axes.
// Set one to -1.0 if the COMPASS_DEBUG test (see Navigation) says the frame is inconsistent.
#define MAG_SIGN_X -1.0
#define MAG_SIGN_Y 1.0
#define MAG_SIGN_Z 1.0
// Direction of the "front" of the device, in sensor axes (1,0,0 = sensor +X points where you point)
#define FWD_X 0.0
#define FWD_Y -1.0
#define FWD_Z 0.0
// Uncomment to print heading diagnostics to the Serial Monitor while in Navigation
//#define COMPASS_DEBUG
int speedUnit = 0;    // 0 = MPH, 1 = KPH, 2 = Knots (long UP cycles)
bool useFeet = true;  // Default to Feet, toggle with DOWN
int coordFormat = 1;  // 0 = Decimal Degrees, 1 = DMS, 2 = MGRS (long RIGHT cycles)
bool isLevel = false;

// --- Magnetometer calibration (filled in by calibrateMagnetometer(); paste Serial output here to make permanent) ---
float magOffX = 0.0, magOffY = 0.0, magOffZ = 0.0;        // hard-iron offsets (uT)
float magScaleX = 1.0, magScaleY = 1.0, magScaleZ = 1.0;  // soft-iron scale
// --- GPS specific ---
TinyGPSPlus gps;
// US Central Time zone definition
TimeChangeRule usCDT = { "CDT", Second, Sun, Mar, 2, -300 };
TimeChangeRule usCST = { "CST", First, Sun, Nov, 2, -360 };
Timezone US_Central(usCDT, usCST);

// --- dB Meter specific ---
short sampleBuffer[256];
volatile int samplesRead;
#define REFERENCE_RMS 0.0001


// --- Metal Detector specific ---
#define CALIBRATION_SAMPLES 100
#define DETECTION_THRESHOLD 20.0
float baselineMagnitude = 0.0;

// --- EMF Meter specific ---
#define MAX_EMF 200.0

// --- Atmospheric specific ---
unsigned long lastAtmosphericUpdate = 0;
bool useCelsius = false;  // false = Fahrenheit, true = Celsius
bool useKPa = false;      // false = inHg, true = kPa

// --- irReceiver specific ---
const int IR_RECV_PIN = 2;
bool irDataReceived = false;  // Flag to track if data has been received
uint32_t storedIrData = 0;
uint8_t storedIrAddr = 0;
uint8_t storedIrCmd = 0;

// --- irSend specific ---
int irSendCursor = 0;

// --- Battery specific ---
// VIn is read via analog pin A0. Voltage Divider is using two 10kOhm resistors, thus ratio of 2.0
#define VOLTAGE_PIN A0
#define VOLTAGE_DIVIDER_RATIO 2.0
#define BATTERY_MIN_V 3.35
#define BATTERY_MAX_V 4.25
unsigned long lastBatteryUpdate = 0;

// ---------- Setup Function ----------
void setup() {
  Serial.begin(9600);
  pinMode(A7, INPUT_PULLUP);    // input pin for buttons
  pinMode(IR_RECV_PIN, INPUT);  // input pin for IR Receiver
  IrSender.begin(IR_SEND_PIN);  // output pin for IR Send

  // Initialize OLED - Screen 1
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true)
      ;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize OLED - Screen 2 (SSD1331)
  tft.begin(8000000);
  tft.setRotation(0);

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

  //--- Set Time Zone ---
  tzLoad();

  //--- Set Compass Declination ---
  declinationLoad();

  // --- Splash Screen - Screen 2---
    char splashVersion[8];
  snprintf(splashVersion, sizeof(splashVersion), "v%.1f", FWver);
  lcSplash(splashVersion, "By Tony Dillberg");

  // --- Splash Screen - Screen 1---
  lcoPowerOn("v0.6", 3500);

  // clear Screen 2
  tft.fillScreen(BLACK);

  // --- Initialize Beeper ---
  pinMode(BEEPER_PIN, OUTPUT);
  playStartupTone();

  // --- Calibrate Metal Detector baseline ---
  calibrateMetalDetector();

  Serial.println("All initialized.");
  Serial.println("Tricorder - v0.6");
  Serial.println("By Tony Dillberg");
}

// ---------- Main Loop ----------
void loop() {
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  bleUpdate();  // Bluetooth: services the radio and sends data (does nothing while Bluetooth is off)

  // When the screen/state changes, force menus to redraw and the clock to repaint
  if (currentState != lastLoopState) {
    lastLoopState = currentState;
    menuDirty = true;
    invalidateClockCache();
  }

  int button = getPressedButton(analogRead(A7));
  int pressType = handleButtonPress(button);
  pressType = remoteApply(pressType, button);  // the phone's remote control (Remote.ino)

  switch (currentState) {
    case MAIN_MENU:
      handleMainMenu(pressType);
      break;
    case GEO_MENU:
      handleGeoMenu(pressType);
      break;
    case ENV_MENU:
      handleEnvMenu(pressType);
      break;
    case COMMS_MENU:
      handleCommsMenu(pressType);
      break;
    case UTIL_MENU:
      handleUtilMenu(pressType);
      break;
    case SYS_MENU:
      handleSysMenu(pressType);
      break;
    case METALDETECTOR:
      //tft.fillScreen(BLACK);
      MetalDetector(pressType);
      break;
    case EMFMETER:
      //tft.fillScreen(BLACK);
      EMFMeter(pressType);
      break;
    case SEISMOMETER:
      Seismometer(pressType);
      break;
    case NAVIGATION:
      //tft.fillScreen(BLACK);
      Navigation(pressType);
      break;
    case ATMOSPHERIC:
      //tft.fillScreen(BLACK);
      Atmospheric(pressType);
      break;
    case DBMETER:
      //tft.fillScreen(BLACK);
      DBMeter(pressType);
      break;
    case SPECTRUM:
      SpectrumAnalyzer(pressType);
      break;
    case AUDIO_STREAM: 
      AudioStreamScreen(pressType);
      break;
    case IRRECEIVER:
      //tft.fillScreen(BLACK);
      irReceiver(pressType);
      break;
    case IRSEND:
      //tft.fillScreen(BLACK);
      IrSend(pressType);
      break;
    case SHOTTIMER:
      ShotTimer(pressType);
      break;
    case STOPWATCH:
      Stopwatch(pressType);
      break;
    case COUNTDOWNTIMER:
      CountdownTimer(pressType);
      break;
    case BLUETOOTH_SCREEN:
      BluetoothScreen(pressType);
      break;
    case BATTERY:
      //tft.fillScreen(BLACK);
      Battery(pressType);
      break;
    case SYSINFO:
      SystemInfo(pressType);
      break;
    case REBOOT:
      RebootDevice(pressType);
      break;
    case TIMEZONE:
      TimeZoneSetup(pressType);
      break;
    case DECLINATION_SETUP:
      DeclinationSetup(pressType);
      break;

  }
}

// ---- Main Menu ----
void handleMainMenu(int pressType) {
    if (menuDirty) {
    lcoDrawMainMenu();
    menuDirty = false;
  }

  displayGPSClock();  // cheap: only touches the TFT when the GPS time actually changed

  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) {  // Up
      mainMenuCursor = (mainMenuCursor - 1 + 5) % 5;
      menuDirty = true;
    } else if (lastButton == 4) {  // Down
      mainMenuCursor = (mainMenuCursor + 1) % 5;
      menuDirty = true;
    } else if (lastButton == 1) {  // Enter
      switch (mainMenuCursor) {
        case 0: currentState = GEO_MENU; break;
        case 1: currentState = ENV_MENU; break;
        case 2: currentState = COMMS_MENU; break;
        case 3: currentState = UTIL_MENU; break;
        case 4: currentState = SYS_MENU; break;
      }
    }
  }
}
