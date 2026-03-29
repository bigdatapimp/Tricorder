/*
// -- Tricorder v0.43.32 - By Tony Dillberg
// -- This code is written for the Arduino Nano 33 BLE sense Rev 2
// -- This is for a compact, portable device that utilizes a variety of sensors for basic measurements and readings of various
// -- environmental elements as well as location and direction. It is loosely based on the Tricorder concept from the Star Trek 
// -- franchise.
// --
// -- Split Functions into individual files
// -- Other Minor Code Clean up
// -- 
// -- 
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
#define IR_USE_SAMD_TIMER
#include <IRremote.h>
#include "PinDefinitionsAndMore.h" // Set IR_SEND_PIN for different CPU's
#include <TimeLib.h>
#include <Timezone.h>

// --- OLED settings - Screen 1---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- TFT settings - Screen 2---
#define CS_PIN   10
#define DC_PIN   9
#define RST_PIN  8
// RGB565 Color definitions
#define BLACK    0x0000
#define WHITE    0xFFFF
#define RED      0xF800
#define PINK     0xe8b8
#define PURPLE   0x8010
#define GREEN    0x07E0
#define GREEN2   0x1b62
#define BLUE     0x001F
#define BLUE2    0x6433
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define GOLD     0xd5c6
#define ORANGE   0xFC00

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

#define LONG_PRESS_DURATION 1000 // --- Long press duration (ms) ---
#define DEBOUNCE_DURATION 25 // --- Debounce duration (ms) --- 50 default

// --- Beeper settings ---
#define BEEPER_PIN 6

// --- Enhanced State enum for new menu structure ---
enum State {
  MAIN_MENU,
  GEO_MENU,
  MET_MENU,
  BIO_MENU,
  SYS_MENU,
  UTIL_MENU,
  METALDETECTOR,
  EMFMETER,
  NAVIGATION,
  ATMOSPHERIC,
  DBMETER,
  IRRECEIVER,
  IRSEND,
  BATTERY
};

// --- Menu options ---
const char* mainMenuOptions[5] = {"GEO", "MET", "BIO", "SYS", "UTIL"};
const char* geoMenuOptions[3] = {"MDet", "EMF", "Nav"};
const char* metMenuOptions[2] = {"Atmo", "dB"};
const char* utilMenuOptions[2] = {"IR TX", "IR RX"};
const char* sysMenuOptions[1] = {"Batt"};

int mainMenuCursor = 0;  // 0-4 for main menu
int subMenuCursor = 0;   // For submenu cursors

// --- Global state ---
State currentState = MAIN_MENU;

// --- Button handling variables ---
int lastButton = 0;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;
int stableButton = 0;
unsigned long stableStart = 0;

// --- Navigation specific ---
#define COMPASS_RADIUS 14
#define CENTER_X 68
#define CENTER_Y 25
#define NEEDLE_LENGTH 9
#define DECLINATION 3.44 // board alignment +/- magnetic Declination
bool useMPH = true;   // Default to MPH, toggle with UP
bool useFeet = true;  // Default to Feet, toggle with DOWN
bool useDMS = true; // true = Degrees Minutes Seconds, false = Decimal Degrees
// --- Global variables for sensor fusion ---
float fusedHeading = 0.0;
unsigned long lastFusionTime = 0;
bool isLevel = false;
// --- GPS specific ---
TinyGPSPlus gps;
// US Central Time zone definition
TimeChangeRule usCDT = {"CDT", Second, Sun, Mar, 2, -300};
TimeChangeRule usCST = {"CST", First, Sun, Nov, 2, -360};
Timezone US_Central(usCDT, usCST);

// --- dB Meter specific ---
short sampleBuffer[256];
volatile int samplesRead;
#define REFERENCE_RMS 0.0001
float max_dB = 0.0;

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
bool irDataReceived = false; // Flag to track if data has been received
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
  pinMode(A7, INPUT_PULLUP); // input pin for buttons
  pinMode(IR_RECV_PIN, INPUT); // input pin for IR Receiver
  IrSender.begin(IR_SEND_PIN); // output pin for IR Send

  // Initialize OLED - Screen 1
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
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

  // --- Splash Screen - Screen 2---
  tft.fillScreen(BLACK);
  tft.setCursor(0, 8);
  tft.setTextColor(MAGENTA);
  tft.println("________________");
  tft.setTextColor(YELLOW);
  tft.setCursor(0, 16);
  tft.println("Tricorder v0.43");
  tft.setTextColor(BLUE);
  tft.setCursor(0, 30);
  tft.println("By Tony Dillberg");
  tft.setTextColor(PURPLE);
  tft.setCursor(0, 33);
  tft.println("________________");

  // --- Splash Screen - Screen 1---
  display.setCursor(0, 8);
  display.println(" ___________________");
  display.setCursor(30, 16);
  display.println("POWERING ON");
  display.setCursor(40, 30);
  display.println("Stand By");
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
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  int button = getPressedButton(analogRead(A7));
  int pressType = handleButtonPress(button);

  switch (currentState) {
    case MAIN_MENU:
      handleMainMenu(pressType);
      break;
    case GEO_MENU:
      handleGeoMenu(pressType);
      break;
    case MET_MENU:
      handleMetMenu(pressType);
      break;
    case BIO_MENU:
      handleBioMenu(pressType);
      break;
    case SYS_MENU:
      handleSysMenu(pressType);
      break;
    case UTIL_MENU:
      handleUtilMenu(pressType);
      break;
    case METALDETECTOR:
      //tft.fillScreen(BLACK);
      MetalDetector(pressType);
      break;
    case EMFMETER:
      //tft.fillScreen(BLACK);
      EMFMeter(pressType);
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
    case IRRECEIVER:
      //tft.fillScreen(BLACK);
      irReceiver(pressType);
      break;  
    case IRSEND:
      //tft.fillScreen(BLACK);
      IrSend(pressType);
      break;
    case BATTERY:
      //tft.fillScreen(BLACK);
      Battery(pressType);
      break;
  }
  
}

// ---- Main Menu ----
void handleMainMenu(int pressType) {
  display.clearDisplay();
  display.setCursor(20, 9);
  display.setTextSize(1);
  display.println(F("MAIN MENU"));
  
  // Display 5 main menu options
  for (int i = 0; i < 5; i++) {
    int y = 25 + (i * 8);
    display.setCursor(20, y);
    if (i == mainMenuCursor) {
      display.print("> ");
    } else {
      display.print("  ");
    }
    display.println(mainMenuOptions[i]);
  }
  displayGPSClock();
  display.display();
  
  if (pressType == SHORT_PRESS) {
    if (lastButton == 2) { // Up
      mainMenuCursor = (mainMenuCursor - 1 + 5) % 5;
    } else if (lastButton == 4) { // Down
      mainMenuCursor = (mainMenuCursor + 1) % 5;
    } else if (lastButton == 1) { // Enter
      switch (mainMenuCursor) {
        case 0: currentState = GEO_MENU; break;
        case 1: currentState = MET_MENU; break;
        case 2: currentState = BIO_MENU; break;
        case 3: currentState = SYS_MENU; break;
        case 4: currentState = UTIL_MENU; break;
      }
    }
  }
}
