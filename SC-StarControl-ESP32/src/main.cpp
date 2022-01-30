// ***---*** Project - Star Control - Host / Broker ***---***

// ***---*** Versions ***---***
/*--- Changelog ---
  24/09/21 - V1.1
  ~ Changed TFL automatic mode so that TFL is ON DURING RUNNING MOTOR (before just when active tfl was advised by SAM)
  25/09/21 - V1.2
  ~ Added "STROBE" options incl. http buttons (continuous & short strobe)
  - Defines: "strobePause" to adjust strobing-frequency | "strobeShortCycles" to adjust duration of short strobe impuls
  ~ Added 2 FUNCTIONS to set RELAIS OUTPUTS for later changes if inverted logic is needed, because of electrical aspects (tflRelais() & starRelais() | bool parameter)
  06/10/21 - V1.3
  ~ Added "DEBUG" option in define statements to reduce used serial resourcs in normal usecase (debug mode is only needed for doing service)
  - Therefore introduced debug(x) & debugln(x) instead of Serial.print(x) & Serial.println(x)
  ~ Added "STAR MOTOR RESTRICTION" option to toggle wheter star led should turn off in automatic mode if motorstart was noticed
  - Originated from problem: If you wanted star to be always on, it had to be "overwritten" and stayed on after locking the car until mcu power was cut by SAM
  - Now with this option disabled the star is on during running motor and fades out with the tfl after locking the car
  12/10/21 - V1.4
  ~ Fixed TFL input signal problem - during night car outputs pwm signal to tfl modules, which is to weak to trigger simple digitalRead() function
  - Therefore introduced "INTERUPT SERVICE ROUTINE" function (D5 attached to interrupt routine) which listens to rising and falling edges of incoming signal
  - Program is now able to determine type of incoming signal (pwm, continuous HIGH, continuous LOW)
  ~ Added "ISRDEBUG" option in define statements to output serial information about ISR processes
  ~ Changed Star and TFL serial output information from fadeStep lines to "started fade" and "finished fade" to stop spamming serial monitor
  21/10/21 - V1.5
  ~ Fixed TFL to be DIMMED when the motor is not running, because it was at full brightness, while it originally is running at 16% duty cycle
  - Introduced "tflDimTreshold" to set a value to which the tfl is up dimmed during a resting motor
  18/11/21 - V1.6
  ~ Added first API like functions, which are accessed by an external qC which connects with the AP and makes get requests, which then override the output system
  - Possibilty to use external clients to do an "emergency shutdown" of the light output from inside the car by a button press (-> project seperated in starhost & starclient)
  - Added GET URLs /API/lightsOff and /API/lightsOn to override by calling
  - Added "apiOverrideOff" to store an active blocking of the light output, as well as "outputParamsBefore" array to store previous led and relais states
  27/11/21 - V1.7
  ~ Discarded API HTTP calls and introduced a mqtt system to create a mesh like network between now starhost - starclient-emergency - starclient-underglow
  - Added uMQTTBroker library to have host act like a broker to distribute messages from emergency to underglow and vice versa
  07/12/21 - V1.8
  ~ Construction of a new website by using the ESPAsyncWebServer method
  - Fresh design with sliders, buttons and momentary buttons
  - If multiple clients are connected their inputs are synced by Interval-JavaScript functions
  ~ Introducing controls for underglow parameters (mode, color, brightness, speed)
  - Including MQTT broadcasting and website controls
  13/01/22 - V2.0
  ~ Added Github repository

  --- Bugs ---
  ~ ISR detection sometimes recognizes continuous HIGH before pwm detection has finished
  - Fix: NOT working all time, but prevents some occurrences - cont. HIGH has to recognize its pattern several times (7) before it is allowed to set final signal type
*/

//***** INCLUDES *****
#include "Arduino.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <AsyncMqttClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "htmlsite.h"
#include "esp_adc_cal.h"

//***** DEFINES *****
// Version
#define VERSION 1.8

// Debug
#define DEBUG 1
#define ISRDEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#if ISRDEBUG == 0
#define ISRln(x) Serial.println(x)
#else
#define ISRln(x)
#endif

// WiFi
#define APSSID "Mercedes Star Control"
#define APPSK "sploithd"

// EEPROM
#define starAdress 0
#define betriebsmodusAdress 1
#define fademodusAdress 2
#define tflAdress 3
#define tflModeAdress 4
#define starFadePauseAdress 5
#define tflFadePauseAdress 6
#define strobeAdress 7
#define starMotorRestrictionAdress 8
#define apiOverrideOffAdress 9
#define modeAdress 10
#define color1Adress 11 // 0x0000FF - Blue
#define color2Adress 12 // 0x00FF00 - Green
#define color3Adress 13 // 0xFF0000 - Red
#define brtnsAdress 14
#define speed1Adress 15 // 0x00FF
#define speed2Adress 16 // 0xFF00 - 0 to 65535
#define uglwMotorRestrictionAdress 17
#define batVoltThresAdress 18
#define batVoltOffsetAdress 22
#define favoriteModeAdress 26
#define favoriteColorAdress1 27
#define favoriteColorAdress2 28
#define favoriteColorAdress3 29
#define favoriteBrtnsAdress 30
#define favoriteSpeedAdress1 31
#define favoriteSpeedAdress2 32
#define uglwTFLRestrictionAdress 33
#define transitionCoefficientAdress 34
#define batVoltOffsetMotorAdress 38
// continue at adress 42

// Inputs
#define tflPin 14     // D5 - Tagfahrlicht             - Type: PWM
#define kl15Pin 35    // D6 - ZST 2                    - Type: Digital
#define kl50Pin 36    // D7 - Starter mit Impuls       - Type: Digital
#define batVoltPin 34 // xx - Battery Voltage          - Type: ADC Input

// Outputs
#define tflLPin 25       // D1 - TFL Links MOSFET (Weiß)  - Type: PWM
#define tflRPin 26       // D2 - TFL Rechts MOSFET (Blau) - Type: PWM
#define starPin 32       // D8 - Star MOSFET (Lila)       - Type: PWM
#define relaisTFLLPin 18 // D3 - TFL Links Relais         - Type: Digital
#define relaisTFLRPin 19 // D4 - TFL Rechts Relais        - Type: Digital
#define relaisStarPin 33 // D0 - Star Relais              - Type: Digital

// Fade-Pause Timings
#define fadePauseFast 1
#define fadePauseNormal 4
#define fadePauseSlow 6

// Strobe Timings & Cycles
#define strobePause 50
#define strobeShortCycles 20
#define led_mode_strobe 1
#define led_speed_strobe 120

// TFL dimmed treshold
#define tflDimTreshold 512

// MQTT
#define status_topic "status"
#define apiOvrOff_topic "api/ovroff"
#define alive_topic "alive"
#define mode_topic "mode"
#define color_topic "color"
#define brtns_topic "brtns"
#define speed_topic "speed"
#define fxmode_topic "fxmode"
#define motor_topic "motor"
#define transitionCoefficient_topic "transCoef"

#define emergency_avemsg "emeg_ave"
#define starhost_avemsg "host_ave"

//***** VARIABLES & OBJECTS *****
// Input State Variables
bool tflBool = false;
bool kl15Bool = false;
bool kl50Bool = false;

// Fade
bool fadeMode; // stores whether global fade mode is active

// Fade - Star
bool fadeInProcess = false;
bool fadeOutProcess = false;
int fadeStep = 0;
int starFadePause = fadePauseNormal;
long long starFadeMillis = 0;

// Fade - TFL
bool fadeInProcessTFL = false;
bool fadeOutProcessTFL = false;
int fadeStepTFL = 0;
int tflFadePause = fadePauseNormal;
long long tflFadeMillis = 0;

// Star Options
bool starSoll;             // stores whether star led on / off in manually mode
bool starMode;             // true = automatic - false = manually
bool starMotorRestriction; // stores whether star is restricted to be off, when motor is on

// TFL Options
bool tflSoll; // stores whether tfl led on / off in manually mode
bool tflMode; // true = automatic - false = manually

// Motor State
bool motorRunning = false; // stores motor running

// Strobe
bool strobeShort = false; // Short Strobe mode active
bool strobe = false;      // Continuous Strobe mode activce
bool httpStrobe = false;
unsigned long strobeMillis = 0; // Time since last strobe cycle
int strobeCycles = 0;           // Strobe Cycles already happend
bool strobeActive = false;      // Stores whether StrobeFct begun with work
bool strobeActiveMQTT = false;
int strobeDataBefore[6]; // Stores previous states of analog / digital outputs of relais and leds

// TFL ISR
portMUX_TYPE sync = portMUX_INITIALIZER_UNLOCKED;
volatile byte tflISRrisingState = 0;
volatile byte tflISRfallingState = 0;
unsigned long tflISRrisingCounter = 0;
unsigned long tflISRfallingCounter = 0;
unsigned long tflISRrisingCounterBefore = 0;
unsigned long tflISRlastRise = 0;
unsigned long tflISRlastFall = 0;
bool tflISRcontiDetected = false;
unsigned int tflISRedgeTreshold = 10 * 2; // ms - * 2 sothat 2 pulses could have passed
unsigned long tflISRlastModeCheck = 0;
unsigned int tflISRmodeCheckTreshold = 50 + (10 * 1);
bool tflISRpwmDetected = false;
unsigned long tflISRriseStart = 0;
bool tflISRriseStartBool = true;

void IRAM_ATTR tflISR()
{
  portENTER_CRITICAL(&sync);
  if (digitalRead(tflPin))
    tflISRrisingState = 1;
  else
    tflISRfallingState = 1;
  portEXIT_CRITICAL(&sync);
}

// LED Serial
HardwareSerial ledSerial(2);
unsigned long lastSAMSent = 0;
const unsigned int SAMTimeout = 5000;
String lastSerialMsg = "";
bool serialClientConnection = false;
bool serialClientInitConnection = false;

// MQTT Client - V1.7
AsyncMqttClient mqttClient;
#define MQTT_HOST IPAddress(192, 168, 4, 1)
#define MQTT_PORT 1883

// API Emergency Mode
bool apiOverrideOff;       // Stores whether API call was received to turn lights off
int outputParamsBefore[6]; // Stores previous states of analog / digital outputs of relais and leds
unsigned long myLastAveMsg = 0;
const unsigned int aveMsgIntervall = 2500;
unsigned long yourlastAveMsg = 0; // stores when last alive msg was received
unsigned const int aveMsgTimeout = 10000;
bool emergency = true;
bool lostEmergencyConnection = true; // stores wheter connection to emergency mqtt client was lost after timeout
bool emgcyWasConnected = false;

// AsyncWebServer
AsyncWebServer server(80);
const char *PARAM_INPUT_1 = "id";
const char *PARAM_INPUT_2 = "state";
const char *PARAM_INPUT_3 = "value";
bool buttonStates[5] = {false, false, false, false, false}; // 0 - Strobe, 1 - Fade, 2 - MREST Star, 3 - MREST UGLW, 4 - TFLREST UGLW
unsigned int sliderValues[6] = {3, 3, 2, 2, 0, 0};          // 0 - Star, 1 - TFL, 2 - Star Fade Time, 3 - TFL Fade Time, 4 - UGLW Brtns, 5 - UGLW Speed
float sliderValuesFloat[4] = {12.0, 0, 1.18, 0};            // 0 - batVoltTresh | 1 - batVoltOffset | 2 - TransCoef | 3 - batVoltOffsetMotor                     // 0 - BatVoltThreshold, 1 - BatVoltOffset

// WS2812 LEDs
unsigned int led_mode = 0;
unsigned int led_color = 0;
unsigned int led_brtns = 0;
unsigned int led_speed = 10000;
bool uglwMotorRestriction = true;
bool uglwMotorBlock = false;
bool uglwMotorBlockSent = false;
bool uglwTFLRestriction = true;
bool uglwTFLRestActive = false;
unsigned int selectedMode;
unsigned int selectedModeBef = 2;
unsigned int selectedModeBefStrobe;
unsigned int led_mode_before = 0;
unsigned int led_brtns_before = 0;
unsigned int favoriteMode;
unsigned int favoriteColor;
unsigned int favoriteBrtns;
unsigned int favoriteSpeed;
float transitionCoefficient;

// Battery Voltage
unsigned long lastADCVal = 0;
const unsigned int batVoltSamples = 40;
float batteryVoltage = 0.0;
float batteryVoltages[batVoltSamples] = {};
unsigned int batVoltMeasurements = 0;
float vref;
const float resistor1 = 325200; // 330k
const float resistor2 = 47340;  // 47k
float batVoltOffset;
bool batteryEmergency = false;
float batteryThreshold;
float motorVoltOffset;

// Startup
bool checkedInputs = false;
bool starStarted = false;
bool tflStarted = false;
bool uglwStarted = false;

// Task Handler (Multithreading)
/*void task1_handlers(void *pvParameters)
{
  while (true)
  {
    vTaskDelay(10);
  }
}*/

//***** PROTOTYPES *****
void handlers();
void startupHandler();
void initLastState();
void writeColorEEPROM(unsigned int, bool);
unsigned int readColorEEPROM(bool);
void writeSpeedEEPROM(unsigned int, bool);
unsigned int readSpeedEEPROM(bool);
void httpHandler();
void readInputs();
void batteryMonitoring();
void interpretInputs(); // check for engine star or stop
void writeOutput();
void automaticMode();
void manuallyMode();
void starOn();
void starOff();
void fadeReset();
void fadeIn();
void fadeOut();
void tflOutput();
void automaticTFL();
void manuallyTFL();
void tflOn();
void tflOff();
void fadeInTFL();
void fadeOutTFL();
void fadeResetTFL();
void strobeFct();
void strobeStart(bool);
void strobeStop(bool);
void tflRelais(bool);
void starRelais(bool);
void saveOutputParams();
void restoreOutputParams();
void tflISRInterpret();
String outputState(int);
String readStarState();
String readTFLState();
String readWebBtn(int);
void assignServerHandlers();
String readWebBtn(int);
String readWebSldr(int);
void interpretSlider(int);
void interpretButton(int);
void uglw_sendValue(unsigned int, float, bool);
void ledsCustomShow();
void mqttAliveMessage();
void onMqttMessage(char *, char *, AsyncMqttClientMessageProperties, size_t, size_t, size_t);
void onMqttConnect(bool);
bool checkMQTT();
bool checkWiFi();
void beginWiFi();
void setEmergencyMode();
void resetEmergencyMode();
void uglwWriteOutput();
void uglwTFLRestrictionHandler();
void serialLEDHandler();
void serialAliveMsg();

//***** SETUP *****
void setup()
{
  // Initialisation
  if (DEBUG || ISRDEBUG)
  {
    Serial.begin(115200);
  }
  ledSerial.begin(115200);
  ledSerial.setTimeout(3);
  EEPROM.begin(42);

  debugln("\n[StarControl-Host] Starting programm ~ by spl01t*#7");
  debugln("[StarControl-Host] You are running version " + String(VERSION) + "!");

  // WiFi
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin("NOBROWN", "OUT!");
  delay(100);
  WiFi.disconnect(true, true);

  // Serial "hello world"
  delay(50);
  ledSerial.print("status!host-wasborn$");

  // Get EEPROM memory
  initLastState();
  delay(10);

  // IOs
  pinMode(tflPin, INPUT); // interrupt
  attachInterrupt(digitalPinToInterrupt(tflPin), tflISR, CHANGE);
  pinMode(kl15Pin, INPUT);        // only digitalRead
  pinMode(kl50Pin, INPUT);        // only digitalRead
  pinMode(tflLPin, OUTPUT);       // pwm
  pinMode(tflRPin, OUTPUT);       // pwm
  pinMode(starPin, OUTPUT);       // pwm
  pinMode(relaisStarPin, OUTPUT); // only digitalRead/-Write
  pinMode(relaisTFLLPin, OUTPUT); // only digitalRead/-Write
  pinMode(relaisTFLRPin, OUTPUT); // only digitalRead/-Write
  pinMode(batVoltPin, INPUT);     // adc input

  // PWM Setup
  for (int i = 0; i < 3; i++)
  {
    ledcSetup(i, 100, 10);
  }
  ledcAttachPin(starPin, 0);
  ledcAttachPin(tflLPin, 1);
  ledcAttachPin(tflRPin, 2);
  ledcWrite(0, 0);                  // led star in beginning off
  ledcWrite(1, 0);                  // tfl l in beginning off
  ledcWrite(2, 0);                  // tfl r in beginning off
  digitalWrite(relaisStarPin, LOW); // relais in beginnning off
  digitalWrite(relaisTFLLPin, LOW);
  digitalWrite(relaisTFLRPin, LOW);

  // MQTT
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setClientId("starhost1");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onMessage(onMqttMessage);

  // ADC
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  vref = adc_chars.vref;

  debugln("\n[StarControl-Host] Initialization completed...starting programm loop!\n");
}

//***** LOOP *****
void loop()
{
  do
  {
    readInputs();

    interpretInputs();

    writeOutput();

    serialLEDHandler();

    uglwWriteOutput();

    startupHandler();
  } while (!starStarted || !tflStarted || !uglwStarted);

  mqttAliveMessage();

  checkMQTT();

  checkWiFi();

  handlers();
}

//***** FUNCTIONS *****
// Handler Functions
void handlers()
{
  ArduinoOTA.handle();
  yield();
}

void startupHandler()
{
  if (emergency)
  {
    starStarted = true;
    tflStarted = true;
    uglwStarted = true;
  }
  if (uglwTFLRestActive && starMode && !starStarted)
  {
    starStarted = true;
    debugln("[StartUp] TFL Restriction active - Star turned off!");
  }
  if (uglwTFLRestActive && tflMode && !tflStarted)
  {
    tflStarted = true;
    debugln("[StartUp] TFL Restriction active - TFL turned off!");
  }
  if (starMode == 0 && starSoll == 0 && starStarted)
  {
    starStarted = true;
    debugln("[Star] Turned off!");
  }
  if (tflMode == 0 && tflSoll == 0 && !tflStarted)
  {
    tflStarted = true;
    debugln("[TFL] Turned off!");
  }
}

// Input Signals
void readInputs()
{
  kl15Bool = digitalRead(kl15Pin);
  kl50Bool = digitalRead(kl50Pin);

  do
  {
    batteryMonitoring();
  } while (batteryVoltage == 0.0);
}

void batteryMonitoring()
{
  if (millis() >= (lastADCVal + 30) || batteryVoltage == 0.0) // ca every second 1 measurement
  {
    lastADCVal = millis();
    float anaVal = (float)analogRead(batVoltPin) * 0.0002442;
    float inputVoltage = anaVal * 3.3 * (1100 / vref);
    float realVoltage = inputVoltage * ((resistor1 + resistor2) / resistor2);
    batteryVoltages[batVoltMeasurements] = realVoltage + batVoltOffset;
    batVoltMeasurements++;
    /*debugln("Ana-Val: " + String(anaVal));
    debugln("Input-Voltage: " + String(inputVoltage));
    debugln("Real-Voltage: " + String(realVoltage));*/
    if (batVoltMeasurements >= batVoltSamples)
    {
      for (int i = 0; i < batVoltMeasurements; i++)
      {
        batteryVoltage += batteryVoltages[i];
      }
      batteryVoltage /= batVoltMeasurements; // Sum / batVoltSamples
      // debugln("Battery Voltage: " + String(batteryVoltage));
      batVoltMeasurements = 0;
    }
  }
}

void interpretInputs()
{
  if (kl50Bool && !motorRunning)
  {
    debugln("\n[IO] Detected motor start!\n");
    motorRunning = true;
    batVoltOffset += motorVoltOffset;
  }

  if (motorRunning && !kl15Bool && !kl50Bool)
  {
    debugln("\n[IO] Detected motor stop!\n");
    motorRunning = false;
    batVoltOffset -= motorVoltOffset;
  }

  do
  {
    tflISRInterpret(); // V1.4 - TFL ISR checking for input signal type
  } while (!checkedInputs);

  if (batteryVoltage <= batteryThreshold && !batteryEmergency) // Battery-Management
  {
    batteryEmergency = true;
    debugln("[BAT] Setting emergency mode, because voltage is " + String(batteryVoltage) + " Volt!");
    setEmergencyMode();
  }
  else if (batteryVoltage > batteryThreshold && batteryEmergency)
  {
    batteryEmergency = false;
    debugln("[BAT] Resetting emergency mode, because voltage is " + String(batteryVoltage) + " Volt!");
    resetEmergencyMode();
  }
}

// Star Functions
void writeOutput()
{
  if (!emergency)
  {
    if (strobe || strobeShort)
    {
      strobeFct();
    }
    else
    {
      // Stop Strobe and set old values
      if (!strobe && strobeActive && !strobeShort)
      {
        strobeStop(true);
      }

      if (starMode)
        automaticMode();
      else
        manuallyMode();

      tflOutput();
    }
  }
}

void automaticMode()
{
  if ((!motorRunning && tflBool) || (motorRunning && !starMotorRestriction) || selectedModeBef == 3)
    starOn();
  else
    starOff();
}

void manuallyMode()
{
  if (starSoll)
    starOn();
  else
    starOff();
}

void starOn()
{
  if (ledcRead(0) < 1023)
  {
    if (fadeMode)
    {
      fadeIn();
    }
    else
    {
      ledcWrite(0, 1023);
      starRelais(true);
      debugln("[Star] Turned on!");
      starStarted = true;
    }
  }
}

void starOff()
{
  if (ledcRead(0) > 0)
  {
    if (fadeMode)
    {
      fadeOut();
    }
    else
    {
      ledcWrite(0, 0);
      starRelais(false);
      debugln("[Star] Turned off!");
      starStarted = true;
    }
  }
}

void fadeIn()
{
  if (fadeInProcess && fadeStep <= 1023)
  {
    if (millis() > (starFadePause + starFadeMillis))
    {
      starFadeMillis = millis();
      if (fadeStep < 1018)
      {
        fadeStep++;
        ledcWrite(0, fadeStep);
      }
      else if (fadeStep >= 1018)
      {
        fadeStep = 1023;
        ledcWrite(0, fadeStep);
        fadeInProcess = false;
        debugln("[Star] Finished Up-Fade!");
        starStarted = true;
      }
      // debug("[Star] Step Up: ");
      // debugln(fadeStep);
    }
  }
  else
  {
    fadeInProcess = true;
    fadeOutProcess = false;
    starRelais(true);
    debugln("[Star] Started Up-Fade!");
  }
}

void fadeOut()
{
  if (fadeOutProcess && fadeStep >= 0)
  {
    if (millis() > (starFadePause + starFadeMillis))
    {
      starFadeMillis = millis();
      if (fadeStep > 5)
      {
        fadeStep--;
        ledcWrite(0, fadeStep);
      }
      else if (fadeStep <= 5)
      {
        fadeStep = 0;
        ledcWrite(0, fadeStep);
        starRelais(false);
        fadeOutProcess = false;
        debugln("[Star] Finished Down-Fade!");
        starStarted = true;
      }
      // debug("[Star] Step Down: ");
      // debugln(fadeStep);
    }
  }
  else
  {
    fadeOutProcess = true;
    fadeInProcess = false;
    if (fadeStep > 0)
    {
      starRelais(true);
    }
    debugln("[Star] Started Down-Fade!");
  }
}

void fadeReset()
{
  fadeInProcess = false;
  fadeOutProcess = false;
}

// TFL Functions
void tflOutput()
{
  if (tflMode)
  {
    automaticTFL();
  }
  else
  {
    manuallyTFL();
  }
}

void automaticTFL()
{
  if (tflBool || motorRunning) // since v1.1 also on w/ running motor
  {
    tflOn();
  }
  else
  {
    tflOff();
  }
}

void manuallyTFL()
{
  if (tflSoll)
  {
    tflOn();
  }
  else
  {
    tflOff();
  }
}

void tflOn()
{
  if (((ledcRead(1) < 1023 && motorRunning) || (ledcRead(2) < 1023 && motorRunning)) || ((ledcRead(1) < tflDimTreshold && !motorRunning) || (ledcRead(2) < tflDimTreshold && !motorRunning)))
  {
    if (fadeMode)
    {
      fadeInTFL();
    }
    else
    {
      if (motorRunning)
      {
        ledcWrite(1, 1023);
        ledcWrite(2, 1023);
      }
      else
      {
        ledcWrite(1, tflDimTreshold);
        ledcWrite(2, tflDimTreshold);
      }
      tflRelais(true);
      debugln("[TFL] Turned on!");
      tflStarted = true;
    }
  }
  else if ((!motorRunning && (ledcRead(1) > tflDimTreshold)) || (!motorRunning && (ledcRead(2) > tflDimTreshold)))
  {
    if (ledcRead(1) > tflDimTreshold || ledcRead(2) > tflDimTreshold)
    {
      if (fadeMode)
      {
        if (fadeOutProcessTFL && fadeStepTFL > tflDimTreshold)
        {
          if (millis() > (tflFadePause + tflFadeMillis))
          {
            tflFadeMillis = millis();
            if (fadeStepTFL > (tflDimTreshold + 1))
            {
              fadeStepTFL--;
              ledcWrite(1, fadeStepTFL);
              ledcWrite(2, fadeStepTFL);
            }
            else if (fadeStepTFL <= (tflDimTreshold + 1))
            {
              fadeStepTFL = tflDimTreshold;
              ledcWrite(1, fadeStepTFL);
              ledcWrite(2, fadeStepTFL);
              tflRelais(true);
              fadeOutProcessTFL = false;
              debugln("[TFL] Motor stop - dimmed!");
              tflStarted = true;
            }
            // debug("[TFL] Step Down: ");
            // debugln(fadeStepTFL);
          }
        }
        else
        {
          fadeOutProcessTFL = true;
          fadeInProcessTFL = false;
          if (fadeStepTFL > 0)
          {
            tflRelais(true);
          }
          debugln("[TFL] Started Dimmed-Fade!");
          if (fadeStepTFL == tflDimTreshold)
          {
            ledcWrite(1, fadeStepTFL);
            ledcWrite(2, fadeStepTFL);
            tflRelais(true);
            fadeOutProcessTFL = false;
            debugln("[TFL] Motor stop - dimmed!");
            tflStarted = true;
          }
        }
      }
      else
      {
        ledcWrite(1, tflDimTreshold);
        ledcWrite(2, tflDimTreshold);
        tflRelais(true);
        debugln("[TFL] Motor stop - dimmed!");
        tflStarted = true;
      }
    }
  }
}

void tflOff()
{
  if (ledcRead(1) > 0 || ledcRead(2) > 0)
  {
    if (fadeMode)
    {
      fadeOutTFL();
    }
    else
    {
      ledcWrite(1, 0);
      ledcWrite(2, 0);
      tflRelais(false);
      debugln("[TFL] Turned off!");
      tflStarted = true;
    }
  }
}

void fadeInTFL()
{
  if (fadeInProcessTFL && fadeStepTFL < 1023)
  {
    if (millis() > (tflFadePause + tflFadeMillis))
    {
      tflFadeMillis = millis();
      if ((fadeStepTFL < 1018 && motorRunning) || (fadeStepTFL < (tflDimTreshold - 1) && !motorRunning))
      {
        fadeStepTFL++;
        ledcWrite(1, fadeStepTFL);
        ledcWrite(2, fadeStepTFL);
      }
      else if (fadeStepTFL >= 1018 && motorRunning)
      {
        fadeStepTFL = 1023;
        ledcWrite(1, fadeStepTFL);
        ledcWrite(2, fadeStepTFL);
        fadeInProcessTFL = false;
        debugln("[TFL] Finished Up-Fade (full)!");
        tflStarted = true;
      }
      else if (fadeStepTFL >= (tflDimTreshold - 1) && !motorRunning)
      {
        fadeStepTFL = tflDimTreshold;
        ledcWrite(1, fadeStepTFL);
        ledcWrite(2, fadeStepTFL);
        fadeInProcessTFL = false;
        debugln("[TFL] Finished Up-Fade (dimmed)!");
        tflStarted = true;
      }
      // debug("[TFL] Step Up: ");
      // debugln(fadeStepTFL);
    }
  }
  else
  {
    fadeInProcessTFL = true;
    fadeOutProcessTFL = false;
    tflRelais(true);
    debugln("[TFL] Started Up-Fade!");
  }
}

void fadeOutTFL()
{
  if (fadeOutProcessTFL && fadeStepTFL > 0)
  {
    if (millis() > (tflFadePause + tflFadeMillis))
    {
      tflFadeMillis = millis();
      if (fadeStepTFL > 5)
      {
        fadeStepTFL--;
        ledcWrite(1, fadeStepTFL);
        ledcWrite(2, fadeStepTFL);
      }
      else if (fadeStepTFL <= 5)
      {
        fadeStepTFL = 0;
        ledcWrite(1, fadeStepTFL);
        ledcWrite(2, fadeStepTFL);
        tflRelais(false);
        fadeOutProcessTFL = false;
        debugln("[TFL] Finished Down-Fade!");
        tflStarted = true;
      }
      // debug("[TFL] Step Down: ");
      // debugln(fadeStepTFL);
    }
  }
  else
  {
    fadeOutProcessTFL = true;
    fadeInProcessTFL = false;
    if (fadeStepTFL > 0)
    {
      tflRelais(true);
    }
    debugln("[TFL] Started Down-Fade!");
  }
}

void fadeResetTFL()
{
  fadeInProcess = false;
  fadeOutProcess = false;
}

// Strobe Functions
void strobeFct()
{
  // Strobe Start
  if (!strobeActive)
  {
    strobeStart(true);
    starStarted = true;
    tflStarted = true;
    debugln("[Strobe] Starting...");
  }
  // Strobe Function
  if (millis() >= strobeMillis + strobePause)
  {
    strobeMillis = millis();
    strobeCycles++;
    tflRelais(true);
    starRelais(true);
    if (ledcRead(0) > 0)
    {
      ledcWrite(0, 0);
      ledcWrite(2, 0);
      ledcWrite(1, 0);
      starRelais(false);
      tflRelais(false);
    }
    else
    {
      ledcWrite(0, 1023);
      ledcWrite(2, 1023);
      ledcWrite(1, 1023);
      starRelais(true);
      tflRelais(true);
    }
  }

  // Strobe Short stop condition
  if (strobeShort && strobeCycles >= strobeShortCycles)
  {
    strobeShort = false;
    strobeCycles = 0;
    strobeStop(true);
  }
}

void strobeStart(bool execMQTT)
{
  strobe = true;
  strobeActive = true;
  buttonStates[0] = true;

  if (execMQTT)
  {
    strobeActiveMQTT = true;
    uglwStarted = true;
    selectedModeBefStrobe = selectedModeBef;
    char charMode = '4';
    char *payload = &charMode;
    mqttClient.publish(fxmode_topic, 0, false, payload);
    debugln("\n[STROBE] Current selected Mode: " + String(selectedModeBefStrobe));
  }

  strobeDataBefore[0] = ledcRead(0);
  strobeDataBefore[1] = ledcRead(1);
  strobeDataBefore[2] = ledcRead(2);
  strobeDataBefore[3] = digitalRead(relaisStarPin);
  strobeDataBefore[4] = digitalRead(relaisTFLLPin);
  strobeDataBefore[5] = digitalRead(relaisTFLRPin);

  EEPROM.write(strobeAdress, 1);
  EEPROM.commit();
}

void strobeStop(bool execMQTT)
{
  strobe = false;
  strobeActive = false;
  buttonStates[0] = false;

  if (execMQTT)
  {
    char charMode = '5';
    char *payload = &charMode;
    if (selectedModeBefStrobe == 1)
      *payload = '1';
    else if (selectedModeBefStrobe == 2)
      *payload = '2';
    else if (selectedModeBefStrobe == 3)
      *payload = '3';
    else if (selectedModeBefStrobe == 4)
      *payload = '4';

    debugln("\n[STROBE] Loading before selected Mode: " + String(*payload));
    mqttClient.publish(fxmode_topic, 0, false, payload);
  }

  ledcWrite(0, strobeDataBefore[0]);
  ledcWrite(1, strobeDataBefore[1]);
  ledcWrite(2, strobeDataBefore[2]);
  digitalWrite(relaisStarPin, strobeDataBefore[3]);
  digitalWrite(relaisTFLLPin, strobeDataBefore[4]);
  digitalWrite(relaisTFLRPin, strobeDataBefore[5]);

  EEPROM.write(strobeAdress, 0);
  EEPROM.commit();
}

// Relais Functions
void tflRelais(bool mode)
{
  if (mode)
  {
    digitalWrite(relaisTFLLPin, HIGH);
    digitalWrite(relaisTFLRPin, HIGH);
  }
  else
  {
    digitalWrite(relaisTFLLPin, LOW);
    digitalWrite(relaisTFLRPin, LOW);
  }
}

void starRelais(bool mode)
{
  if (mode)
  {
    digitalWrite(relaisStarPin, HIGH);
  }
  else
  {
    digitalWrite(relaisStarPin, LOW);
  }
}

// Save & Restore Output Parameters - V1.6 API Client Override Light Scene
void saveOutputParams()
{
  outputParamsBefore[0] = ledcRead(0);
  outputParamsBefore[1] = ledcRead(1);
  outputParamsBefore[2] = ledcRead(2);
  outputParamsBefore[3] = digitalRead(relaisStarPin);
  outputParamsBefore[4] = digitalRead(relaisTFLLPin);
  outputParamsBefore[5] = digitalRead(relaisTFLRPin);
}

void restoreOutputParams()
{
  if (motorRunning && starMotorRestriction)
    ledcWrite(0, 0);
  else
    ledcWrite(0, outputParamsBefore[0]);
  ledcWrite(1, outputParamsBefore[1]);
  ledcWrite(2, outputParamsBefore[2]);
  digitalWrite(relaisStarPin, outputParamsBefore[3]);
  digitalWrite(relaisTFLLPin, outputParamsBefore[4]);
  digitalWrite(relaisTFLRPin, outputParamsBefore[5]);
}

// API Override Light Scene
void apiOverrideLights()
{
  tflRelais(false);
  starRelais(false);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
}

// ISR for TFL Input Signal from car - V1.4
void tflISRInterpret()
{
  // Edge event handlers
  if (tflISRrisingState == 1)
  {
    tflISRrisingState = 0;
    tflISRcontiDetected = false;
    tflISRlastRise = millis();
    tflISRrisingCounter++;
    if (tflISRriseStartBool)
    {
      tflISRriseStart = tflISRrisingCounter;
      tflISRriseStartBool = false;
    }
    if (!tflISRpwmDetected)
      ISRln("[ISR] Rising Edge detected: " + String(tflISRrisingCounter));
  }

  if (tflISRfallingState == 1)
  {
    tflISRfallingState = 0;
    tflISRcontiDetected = false;
    tflISRlastFall = millis();
    tflISRfallingCounter++;
    if (!tflISRpwmDetected)
      ISRln("[ISR] Falling Edge detected: " + String(tflISRfallingCounter));
  }

  // Check signal for pwm
  if ((millis() > (tflISRlastModeCheck + tflISRmodeCheckTreshold)) && !tflISRpwmDetected)
  {
    tflISRlastModeCheck = millis();
    if (tflISRrisingCounterBefore + 5 <= tflISRrisingCounter)
    {
      tflBool = true;
      tflISRpwmDetected = true;
      ISRln("\n[ISR] TFL PWM signal detected!\n");
      checkedInputs = true;
    }
    tflISRrisingCounterBefore = tflISRrisingCounter;
  }

  // Check signal for continuous levels - compares signal after 2 pulse lenghts to the state before - if the same --> conti level
  if (((millis() > (tflISRlastRise + tflISRedgeTreshold)) && !tflISRcontiDetected) || ((millis() > (tflISRlastFall + tflISRedgeTreshold)) && !tflISRcontiDetected))
  {
    tflISRrisingCounterBefore = tflISRrisingCounter;
    if (digitalRead(tflPin))
    {
      if ((tflISRrisingCounter >= (tflISRriseStart + 10)) || (millis() > (tflISRlastRise + 200)))
      {
        tflBool = true;
        tflISRcontiDetected = true;
        tflISRpwmDetected = false;
        ISRln("\n[ISR] Continuous TFL HIGH detected!\n");
        checkedInputs = true;
      }
      else
        ISRln("[ISR] tflISRriseStart: " + String(tflISRriseStart) + " " + String(tflISRrisingCounter));
    }
    else
    {
      tflBool = false;
      tflISRcontiDetected = true;
      tflISRpwmDetected = false;
      tflISRriseStartBool = true;
      ISRln("\n[ISR] Continuous TFL LOW detected!\n");
      checkedInputs = true;
    }
  }
}

// EEPROM
void initLastState()
{
  debugln("\n[EEPROM] Starting data extratction!");

  // Betriebsmodus State
  int bmcontent = int(EEPROM.read(betriebsmodusAdress));

  debugln("[EEPROM] Star Modus: " + String(bmcontent));

  if (bmcontent == 1)
  {
    starMode = true; // Automatic
  }
  else if (bmcontent == 0)
  {
    starMode = false; // Manually
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - Star Modus");
    EEPROM.write(betriebsmodusAdress, 1); // then write mode automatic
    starMode = true;                      // Automatic
  }

  if (starMode)
  {
    sliderValues[0] = 1;
  }

  // LED Star State
  int starContent = int(EEPROM.read(starAdress)); // read EEPROM

  debugln("[EEPROM] Star LED: " + String(starContent));

  if (starContent == 1)
  {
    starSoll = true;
  }
  else if (starContent == 0)
  {
    starSoll = false;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - Star LED");
    EEPROM.write(starAdress, 0); // then write star off
    starSoll = false;
  }

  if (!starMode)
  {
    starSoll ? sliderValues[0] = 2 : sliderValues[0] = 3;
  }

  // TFL Mode State
  int tflModeContent = int(EEPROM.read(tflModeAdress)); // read EEPROM

  debugln("[EEPROM] TFL Modus: " + String(tflModeContent));

  if (tflModeContent == 1)
  {
    tflMode = true;
  }
  else if (tflModeContent == 0)
  {
    tflMode = false;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - TFL Modus");
    EEPROM.write(tflModeAdress, 0); // then don't override tfl
    tflMode = false;
  }

  if (tflMode)
  {
    sliderValues[1] = 1;
  }

  // TFL Override State
  int tflContent = int(EEPROM.read(tflAdress)); // read EEPROM

  debugln("[EEPROM] TFL LED: " + String(tflContent));

  if (tflContent == 1)
  {
    tflSoll = true;
  }
  else if (tflContent == 0)
  {
    tflSoll = false;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - TFL LED");
    EEPROM.write(tflAdress, 0); // then don't override tfl
    tflSoll = false;
  }

  if (!tflMode)
  {
    tflSoll ? sliderValues[1] = 2 : sliderValues[1] = 3;
  }

  // Fade Modus State
  int fadeContent = int(EEPROM.read(fademodusAdress)); // read EEPROM

  debugln("[EEPROM] Fade Modus: " + String(starContent));

  if (fadeContent == 1)
  {
    fadeMode = true;
  }
  else if (fadeContent == 0)
  {
    fadeMode = false;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - Fade Modus");
    EEPROM.write(fademodusAdress, 0); // then write star off
    fadeMode = false;
  }

  buttonStates[1] = fadeMode;

  // Star Fade Pause
  int stfpContent = int(EEPROM.read(starFadePauseAdress)); // read EEPROM

  debugln("[EEPROM] Star Fade-Pause: " + String(stfpContent));

  bool logicState = true;

  if (stfpContent == 3)
  {
    starFadePause = fadePauseFast;
  }
  else if (stfpContent == 2)
  {
    starFadePause = fadePauseNormal;
  }
  else if (stfpContent == 1)
  {
    starFadePause = fadePauseSlow;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - Star Fade-Pause");
    EEPROM.write(starFadePauseAdress, 2); // then set to 2 default fade pause
    starFadePause = 2;
    logicState = false;
  }

  logicState ? sliderValues[2] = stfpContent : logicState = false;

  // TFL Fade Pause
  int tflfpContent = int(EEPROM.read(tflFadePauseAdress)); // read EEPROM

  debugln("[EEPROM] TFL Fade-Pause: " + String(tflfpContent));

  logicState = true;

  if (tflfpContent == 3)
  {
    tflFadePause = fadePauseFast;
  }
  else if (tflfpContent == 2)
  {
    tflFadePause = fadePauseNormal;
  }
  else if (tflfpContent == 1)
  {
    tflFadePause = fadePauseSlow;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - TFL Fade-Pause");
    EEPROM.write(tflFadePauseAdress, 2); // then set to 2 default fade pause
    tflFadePause = 2;
    logicState = false;
  }

  logicState ? sliderValues[3] = tflfpContent : logicState = false;

  // Strobe
  int strobeContent = int(EEPROM.read(strobeAdress)); // read EEPROM

  debugln("[EEPROM] Strobe: " + String(strobeContent));

  if (strobeContent == 0)
  {
    strobe = false;
  }
  else if (strobeContent == 1)
  {
    strobe = true;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - Strobe");
    EEPROM.write(strobeAdress, 0); // then set to 0 default strobe off
    strobe = false;
  }

  buttonStates[0] = strobe;

  // Star Motor Restriction
  int restrictionContent = int(EEPROM.read(starMotorRestrictionAdress)); // read EEPROM

  debugln("[EEPROM] Star Motor Restriction: " + String(restrictionContent));

  if (restrictionContent == 0)
  {
    starMotorRestriction = false;
  }
  else if (restrictionContent == 1)
  {
    starMotorRestriction = true;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - Star Motor Restriction");
    EEPROM.write(starMotorRestrictionAdress, 1); // then set to 1 default star off on motor on
    starMotorRestriction = true;
  }

  buttonStates[2] = starMotorRestriction;

  // Prepare Button Array for Webserver
  if (starMotorRestriction)
    buttonStates[1] = 1;
  else
    buttonStates[1] = 0;

  // API Override Lights
  int apiOverrideContent = int(EEPROM.read(apiOverrideOffAdress)); // read EEPROM

  debug("[EEPROM] API Override Lights: " + String(apiOverrideContent));

  if (apiOverrideContent == 0)
  {
    apiOverrideOff = false;
    emergency = false;
    debugln(" - [EMERGENCY] Mode deactivated!");
  }
  else if (apiOverrideContent == 1)
  {
    apiOverrideOff = true;
    emergency = true;
    debugln(" - [EMERGENCY] Mode activated!");
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - API Override Lights - [EMERGENCY] Mode activated!");
    EEPROM.write(apiOverrideOffAdress, 1); // then set to 1 default - lights off
    apiOverrideOff = true;
    emergency = true;
  }

  // LEDs Mode
  int modeContent = int(EEPROM.read(modeAdress));

  debugln("[EEPROM] LEDs - Mode: " + String(modeContent));

  if (modeContent >= 0 && modeContent <= 56)
  {
    led_mode = modeContent;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Mode - Out of range (0-56)!");
    led_mode = 0;
    EEPROM.write(modeAdress, led_mode); // then static
  }

  // LEDs Color
  unsigned int colorContent = readColorEEPROM(false);

  debugln("[EEPROM] LEDs - Color: " + String(colorContent));

  if (colorContent >= 0 && colorContent <= 16777215)
  {
    led_color = colorContent;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Color - Out of range (0x0-0xFFFFFF)!");
    led_color = 0;
    EEPROM.write(color1Adress, led_color); // then no color
    EEPROM.write(color2Adress, led_color); // then no color
    EEPROM.write(color3Adress, led_color); // then no color
  }

  // LEDs Brightness
  int brtnsContent = int(EEPROM.read(brtnsAdress));

  debugln("[EEPROM] LEDs - Brightness: " + String(brtnsContent));

  if (brtnsContent >= 0 && brtnsContent <= 255)
  {
    led_brtns = brtnsContent;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Brightness - Out of range (0-255)!");
    led_brtns = 0;
    EEPROM.write(brtnsAdress, led_brtns); // then dark
  }

  sliderValues[4] = led_brtns;

  // LEDs Speed
  unsigned int speedContent = readSpeedEEPROM(false);

  debugln("[EEPROM] LEDs - Speed: " + String(speedContent));

  if (speedContent >= 0 && speedContent <= 65535)
  {
    led_speed = speedContent;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Speed - Out of range (0x0-0xFFFF)!");
    led_speed = 10000;
    writeSpeedEEPROM(led_speed, false);
  }

  sliderValues[5] = led_speed;

  // UGLW Motor Restriction
  int uglwMRESTContent = int(EEPROM.read(uglwMotorRestrictionAdress)); // read EEPROM

  debugln("[EEPROM] UGLW Motor Restriction: " + String(uglwMRESTContent));

  if (uglwMRESTContent == 0)
  {
    uglwMotorRestriction = false;
  }
  else if (uglwMRESTContent == 1)
  {
    uglwMotorRestriction = true;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - UGLW Motor Restriction");
    EEPROM.write(uglwMotorRestrictionAdress, 1); // then set to 1 default uglw off on motor on
    starMotorRestriction = true;
  }

  buttonStates[3] = uglwMotorRestriction;

  // Battery Voltage Threshold
  float batVoltThres;
  EEPROM.get(batVoltThresAdress, batVoltThres); // read EEPROM

  debugln("[EEPROM] Battery-Voltage Threshold: " + String(batVoltThres));

  if (batVoltThres >= 10.0 && batVoltThres <= 15.0)
  {
    batteryThreshold = batVoltThres;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - Battery-Voltage Threshold");
    EEPROM.put(batVoltThresAdress, 12.0); // then set to 12
    batteryThreshold = 12.0;
  }

  sliderValuesFloat[0] = batVoltThres;

  // Battery Voltage Offset
  float batVoltOs;
  EEPROM.get(batVoltOffsetAdress, batVoltOs); // read EEPROM

  debugln("[EEPROM] Battery-Voltage Offset: " + String(batVoltOs));

  if (batVoltOs >= -5.0 && batVoltOs <= 5.0)
  {
    batVoltOffset = batVoltOs;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - Battery-Voltage Offset");
    EEPROM.put(batVoltOffsetAdress, 1.8);
    batVoltOffset = 1.8;
  }

  sliderValuesFloat[1] = batVoltOffset;

  // Favorite UGLW Mode
  int favMode = EEPROM.read(favoriteModeAdress);

  debugln("[EEPROM] Favorite UGLW Mode: " + String(favMode));

  if (favMode >= 0 && favMode <= 56)
  {
    favoriteMode = favMode;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Favorite Mode - Out of range (0-56)!");
    EEPROM.write(favoriteModeAdress, 0);
    favoriteMode = 0;
  }

  // Favorite UGLW Color
  unsigned int favColor = readColorEEPROM(true);

  debugln("[EEPROM] Favorite UGLW Color: " + String(favColor));

  if (favColor >= 0 && favColor <= 16777215)
  {
    favoriteColor = favColor;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Favorite Color - Out of range (0x0-0xFFFFFF)!");
    writeColorEEPROM(0, true);
    favoriteColor = 0;
  }

  // Favorite UGLW Brtns
  int favBrtns = EEPROM.read(favoriteBrtnsAdress);

  debugln("[EEPROM] Favorite UGLW Brightness: " + String(favBrtns));

  if (favBrtns >= 0 && favBrtns <= 255)
  {
    favoriteBrtns = favBrtns;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Favorite Brightness - Out of range (0-255)!");
    EEPROM.write(favoriteBrtnsAdress, 0);
    favoriteBrtns = 0;
  }

  // Favorite UGLW Speed
  unsigned int favSpeed = readSpeedEEPROM(true);

  debugln("[EEPROM] Favorite UGLW Brightness: " + String(favSpeed));

  if (favSpeed >= 0 && favSpeed <= 65535)
  {
    favoriteSpeed = favSpeed;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Favorite Speed - Out of range (0x0-0xFFFF)!");
    favoriteSpeed = 10000;
    writeSpeedEEPROM(favoriteSpeed, true);
  }

  // UGLW TFL-REST.
  int uglwTFL = EEPROM.read(uglwTFLRestrictionAdress); // read EEPROM

  debugln("[EEPROM] UGLW TFL Restriction: " + String(uglwTFL));

  if (uglwTFL == 1)
    uglwTFLRestriction = true;
  else if (uglwTFL == 0)
    uglwTFLRestriction = false;
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - UGLW TFL Restriction");
    EEPROM.write(uglwTFLRestrictionAdress, 1);
    uglwTFLRestriction = true;
  }

  buttonStates[4] = uglwTFLRestriction;

  // UGLW Transition Coefficient
  float transCoef;
  EEPROM.get(transitionCoefficientAdress, transCoef); // read EEPROM

  debugln("[EEPROM] Transition Coefficient: " + String(transCoef));

  if (transCoef >= -5.0 && transCoef <= 5.0)
  {
    transitionCoefficient = transCoef;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - Transition Coefficient - Setting to 1.18");
    transCoef = 1.18;
    EEPROM.put(transitionCoefficientAdress, transCoef);
  }

  sliderValuesFloat[2] = transCoef;

  // Batvolt Motor Offset
  float batVoltOfMo;
  EEPROM.get(batVoltOffsetMotorAdress, batVoltOfMo); // read EEPROM

  debugln("[EEPROM] Battery-Voltage Motor Offset: " + String(batVoltOfMo));

  if (batVoltOfMo >= -5.0 && batVoltOfMo <= 5.0)
  {
    motorVoltOffset = batVoltOfMo;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading failed - Battery-Voltage Motor Offset");
    motorVoltOffset = 0.0;
    EEPROM.put(batVoltOffsetMotorAdress, motorVoltOffset);
  }

  sliderValuesFloat[3] = motorVoltOffset;

  EEPROM.commit();

  debugln("[EEPROM] Extraction completed!");
}

void writeColorEEPROM(unsigned int key, bool favorite)
{
  int lowbit = key & 255;
  int midbit = (key >> 8) & 255;
  int highbit = (key >> 16) & 255;

  /*debugln("\nKEY DEC: " + String(key));
  debugln("KEY HEX: " + String(key, HEX));
  debugln("KEY BIN: " + String(key, BIN));
  debugln("LOWBIT: " + String(lowbit, BIN));
  debugln("MIDBIT: " + String(midbit, BIN));
  debugln("HIGHBIT: " + String(highbit, BIN));*/

  EEPROM.write(favorite ? favoriteColorAdress1 : color1Adress, lowbit);  // Blue Bit
  EEPROM.write(favorite ? favoriteColorAdress2 : color2Adress, midbit);  // Green Bit
  EEPROM.write(favorite ? favoriteColorAdress3 : color3Adress, highbit); // Red Bit
  EEPROM.commit();
}

unsigned int readColorEEPROM(bool favorite)
{
  int key = EEPROM.read(favorite ? favoriteColorAdress3 : color3Adress) << 8;     // Red Bit
  key = (key + EEPROM.read(favorite ? favoriteColorAdress2 : color2Adress)) << 8; // Green Bit
  key += EEPROM.read(favorite ? favoriteColorAdress1 : color1Adress);             // Blue Bit

  /*debugln("\nKEY DEC: " + String(key));
  debugln("KEY HEX: " + String(key, HEX));
  debugln("KEY BIN: " + String(key, BIN));*/

  return (unsigned int)key;
}

void writeSpeedEEPROM(unsigned int key, bool favorite)
{
  int lowbit = key & 255;
  int highbit = (key >> 8) & 255;

  /*debugln("\nKEY DEC: " + String(key));
  debugln("KEY HEX: " + String(key, HEX));
  debugln("KEY BIN: " + String(key, BIN));
  debugln("LOWBIT: " + String(lowbit, BIN));
  debugln("HIGHBIT: " + String(highbit, BIN));*/

  EEPROM.write(favorite ? favoriteSpeedAdress1 : speed1Adress, lowbit);
  EEPROM.write(favorite ? favoriteSpeedAdress2 : speed2Adress, highbit);
  EEPROM.commit();
}

unsigned int readSpeedEEPROM(bool favorite)
{
  int key = EEPROM.read(favorite ? favoriteSpeedAdress2 : speed2Adress) << 8;
  key += EEPROM.read(favorite ? favoriteSpeedAdress1 : speed1Adress);

  /*debugln("\nKEY DEC: " + String(key));
  debugln("KEY HEX: " + String(key, HEX));
  debugln("KEY BIN: " + String(key, BIN));*/

  return (unsigned int)key;
}

// AsyncWebServer
String processor(const String &var)
{
  if (var == "STARSTATETEXT")
  {
    return readStarState();
  }
  if (var == "TFLSTATETEXT")
  {
    return readTFLState();
  }
  if (var == "BUTTONPLACEHOLDER0")
  {
    String buttons = "";
    buttons += "<h4>Fade</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"updateButton(this)\" id=\"button1\" " + outputState(1) + "><span class=\"btnslider\"></span></label>";
    return buttons;
  }
  if (var == "BUTTONPLACEHOLDER1")
  {
    String buttons = "";
    buttons += "<h4>Motor Restriction Star</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"updateButton(this)\" id=\"button2\" " + outputState(2) + "><span class=\"btnslider\"></span></label>";
    return buttons;
  }
  if (var == "BUTTONPLACEHOLDER2")
  {
    String buttons = "";
    buttons += "<h4>Motor Restriction UGLW</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"updateButton(this)\" id=\"button3\" " + outputState(3) + "><span class=\"btnslider\"></span></label>";
    return buttons;
  }
  if (var == "BUTTONPLACEHOLDER3")
  {
    String buttons = "";
    buttons += "<h4>TFL Restriction UGLW</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"updateButton(this)\" id=\"button4\" " + outputState(4) + "><span class=\"btnslider\"></span></label>";
    return buttons;
  }
  if (var == "SLIDERTEXT0")
  {
    switch (sliderValues[0])
    {
    case 1:
      return "Star - Automatik";
      break;
    case 2:
      return "Star - An";
      break;
    case 3:
      return "Star - Aus";
      break;

    default:
      break;
    }
  }
  if (var == "SLIDERVALUE0")
  {
    return String(sliderValues[0]);
  }
  if (var == "SLIDERTEXT1")
  {
    switch (sliderValues[1])
    {
    case 1:
      return "TFL - Automatik";
      break;
    case 2:
      return "TFL - An";
      break;
    case 3:
      return "TFL - Aus";
      break;

    default:
      break;
    }
  }
  if (var == "SLIDERVALUE1")
  {
    return String(sliderValues[1]);
  }
  if (var == "SLIDERTEXT2")
  {
    switch (sliderValues[2])
    {
    case 1:
      return "Star - Fade Slow";
      break;
    case 2:
      return "Star - Fade Normal";
      break;
    case 3:
      return "Star - Fade Fast";
      break;
    default:
      break;
    }
  }
  if (var == "SLIDERVALUE2")
  {
    return String(sliderValues[2]);
  }
  if (var == "SLIDERTEXT3")
  {
    switch (sliderValues[3])
    {
    case 1:
      return "TFL - Fade Slow";
      break;
    case 2:
      return "TFL - Fade Normal";
      break;
    case 3:
      return "TFL - Fade Fast";
      break;
    default:
      break;
    }
  }
  if (var == "SLIDERVALUE3")
  {
    return String(sliderValues[3]);
  }
  if (var == "SLIDERTEXT4")
  {
    String retval = "Brightness - " + String(sliderValues[4]);
    return retval;
  }
  if (var == "SLIDERVALUE4")
  {
    return String(sliderValues[4]);
  }
  if (var == "SLIDERTEXT5")
  {
    String retval = "Speed - " + String(sliderValues[5]);
    return retval;
  }
  if (var == "SLIDERVALUE5")
  {
    return String(sliderValues[5]);
  }
  if (var == "INFOTEXT1")
  {
    return "WiFi-RSSI: " + String(WiFi.RSSI());
  }
  if (var == "INFOTEXT2")
  {
    return "Version: " + String(VERSION);
  }
  if (var == "INFOTEXT3")
  {
    return "Battery-Voltage: " + String(batteryVoltage) + " Volt";
  }
  if (var == "SLIDERTEXT6")
  {
    String retval = "Battery-Voltage Threshold - " + String(sliderValuesFloat[0]) + " Volt";
    return retval;
  }
  if (var == "SLIDERVALUE6")
  {
    return String(sliderValuesFloat[0]);
  }
  if (var == "SLIDERTEXT7")
  {
    String retval = "Battery-Voltage Offset - " + String(sliderValuesFloat[1]) + " Volt";
    return retval;
  }
  if (var == "SLIDERTEXT8")
  {
    String retval = "Favorite Mode: " + String(favoriteMode);
    return retval;
  }
  if (var == "SLIDERTEXT9")
  {
    String retval = "Battery-Voltage Motor Offset: " + String(sliderValuesFloat[3]) + " Volt";
    return retval;
  }
  if (var == "SLIDERTEXT10")
  {
    String retval = "Transition Coefficient: " + String(sliderValuesFloat[2]);
    return retval;
  }
  return String();
}

String outputState(int key)
{
  if (buttonStates[key])
    return "checked";
  else
    return "";
}

void assignServerHandlers()
{
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html, processor); });

  // Send a GET request to <ESP_IP>/update?button=<inputMessage1>&state=<inputMessage2>
  server.on("/button", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String btnID;
              String btnState;
              // GET input1 value on <ESP_IP>/update?button=<inputMessage1>&state=<inputMessage2>
              if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2))
              {
                btnID = String(request->getParam(PARAM_INPUT_1)->value().charAt(6)); //Btn ID - 1,2,3,...
                btnState = request->getParam(PARAM_INPUT_2)->value();                //Checked or not - 1 or 0
                if (btnState == "1")
                  buttonStates[btnID.toInt()] = true;
                else
                  buttonStates[btnID.toInt()] = false;
              }
              else
              {
                btnID = "No message sent";
                btnState = "No message sent";
              }
              request->send(200, "text/plain", "OK");
              interpretButton(btnID.toInt()); });

  server.on("/slider", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String sliderID = "";
              String sliderValue = "";
              // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
              if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_3))
              {
                sliderID = String(request->getParam(PARAM_INPUT_1)->value().charAt(6)); //extract Slider ID from String
                if (String(request->getParam(PARAM_INPUT_1)->value()) == "uglw_speed")
                  sliderID = "5";
                else if (String(request->getParam(PARAM_INPUT_1)->value()) == "batvoltoffset")
                  sliderID = "7";
                else if (String(request->getParam(PARAM_INPUT_1)->value()) == "transcoef")
                  sliderID = "8";
                else if (String(request->getParam(PARAM_INPUT_1)->value()) == "batvoltoffsetmotor")
                  sliderID = "9";
                else if (String(request->getParam(PARAM_INPUT_1)->value()) == "favoriteUGLWMode")
                  sliderID = "11";
                sliderValue = request->getParam(PARAM_INPUT_3)->value();
                if (sliderID.toInt() == 6 || sliderID.toInt() == 7 || sliderID.toInt() == 8 || sliderID.toInt() == 9)
                  sliderValuesFloat[sliderID.toInt() - 6] = sliderValue.toFloat();
                else if (sliderID.toInt() == 11 && sliderValue.toInt() >= 0 && sliderValue.toInt() <= 56)
                {
                  favoriteMode = sliderValue.toInt();
                  favoriteColor = led_color;
                  favoriteBrtns = led_brtns;
                  favoriteSpeed = led_speed;
                }
                else
                  sliderValues[sliderID.toInt()] = sliderValue.toInt(); //update array
              }
              else
              {
                sliderID = "No message sent";
                sliderValue = "No message sent";
              }
              request->send(200, "text/plain", "OK");
              interpretSlider(sliderID.toInt()); });

  server.on("/momentaryButton", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String btnID;
              String btnState;
              if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2))
              {
                btnID = String(request->getParam(PARAM_INPUT_1)->value().charAt(6));
                ;                                                     //Btn ID - 1,2,3,...
                btnState = request->getParam(PARAM_INPUT_2)->value(); //Checked or not - 1 or 0
                if (btnState == "1")
                  buttonStates[btnID.toInt()] = true;
                else
                  buttonStates[btnID.toInt()] = false;
              }
              else
              {
                btnID = "No message sent";
                btnState = "No message sent";
              }
              request->send(200, "text/plain", "OK");
              interpretButton(btnID.toInt()); });

  server.on("/starStateText", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/plain", readStarState().c_str()); });

  server.on("/tflStateText", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/plain", readTFLState().c_str()); });

  server.on("/btnstate", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              if (request->hasParam(PARAM_INPUT_1))
              {
                String btnID = String(request->getParam(PARAM_INPUT_1)->value().charAt(6));
                request->send_P(200, "text/plain", readWebBtn(btnID.toInt()).c_str());
              } });

  server.on("/sldrstate", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              if (request->hasParam(PARAM_INPUT_1))
              {
                String sldrID = String(request->getParam(PARAM_INPUT_1)->value().charAt(6));
                request->send_P(200, "text/plain", readWebSldr(sldrID.toInt()).c_str());
              } });

  server.on("/dropdown", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_3))
              {
                unsigned int dropdownID = request->getParam(PARAM_INPUT_1)->value().toInt();
                unsigned int buttonID = request->getParam(PARAM_INPUT_3)->value().toInt();
                if (dropdownID == 0 || dropdownID == 1)       //dropdown: 0 - underglow modes | 1 - colors
                  uglw_sendValue(dropdownID, buttonID, true); //starting at 0
              }
              request->send(200, "text/plain", "OK"); });

  server.on("/infoText1", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "WiFi-RSSI: " + String(WiFi.RSSI());
              request->send_P(200, "text/plain", retval.c_str()); });

  server.on("/infoText3", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "Battery-Voltage: " + String(batteryVoltage) + " Volt";
              request->send_P(200, "text/plain", retval.c_str()); });

  server.on("/batVoltOffset", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "Battery-Voltage Offset: " + String(batVoltOffset) + " Volt";
              request->send_P(200, "text/plain", retval.c_str()); });

  server.on("/favoriteUGLWMode", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "Favorite Mode: " + String(favoriteMode);
              request->send_P(200, "text/plain", retval.c_str()); });

  server.on("/batVoltOffsetMotor", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "Battery-Voltage Motor Offset: " + String(motorVoltOffset) + " Volt";
              request->send_P(200, "text/plain", retval.c_str()); });

  server.on("/transCoef", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "Transition Coefficient: " + String(transitionCoefficient);
              request->send_P(200, "text/plain", retval.c_str()); });
}

String readStarState()
{
  String retval = "";
  if (apiOverrideOff)
    retval += "Star - BLOCKED";
  else if (batteryEmergency)
    retval += "Star - BAT LOW";
  else if (strobe || strobeShort)
    retval += "Star - Strobing";
  else if (ledcRead(0) > 0 && ledcRead(0) < 1023)
    retval += "Star - Fading";
  else if (ledcRead(0) == 0)
    retval += "Star - AUS";
  else if (ledcRead(0) == 1023)
    retval += "Star - AN";
  return retval;
}

String readTFLState()
{
  String retval = "";
  if (apiOverrideOff)
    retval += "TFL - BLOCKED";
  else if (batteryEmergency)
    retval += "TFL - BAT LOW";
  else if (strobe || strobeShort)
    retval += "TFL - Strobing";
  else if ((ledcRead(1) > 0 && ledcRead(1) < 1023 && motorRunning) || (ledcRead(1) > 0 && ledcRead(1) < tflDimTreshold && !motorRunning) || (ledcRead(1) > tflDimTreshold && !motorRunning))
    retval += "TFL - Fading";
  else if (ledcRead(1) == 0)
    retval += "TFL - AUS";
  else if ((ledcRead(1) == 1023 && motorRunning) || (ledcRead(1) == tflDimTreshold && !motorRunning))
    retval += "TFL - AN";
  return retval;
}

String readWebBtn(int id)
{
  if (buttonStates[id])
    return "true";
  else
    return "false";
}

String readWebSldr(int id)
{
  if (id == 6)
    return String(sliderValuesFloat[id - 6]); // Floats
  else
    return String(sliderValues[id]); // Ints
}

void interpretSlider(int id)
{
  if (id == 0) // *** STAR SLIDER ***
  {
    switch (sliderValues[id])
    {
    case 1:                                   // Automatik Star
      debugln("[HTTP] Star-Modus Automatik"); // Automatic Mode
      starMode = true;
      fadeReset();
      EEPROM.write(betriebsmodusAdress, 1);
      EEPROM.commit();
      break;
    case 2: // An Star
      if (starMode)
      {
        debugln("[HTTP] Star-Modus Manuell"); // Manually Mode
        starMode = false;
        fadeReset();
        EEPROM.write(betriebsmodusAdress, 0);
        EEPROM.commit();
      }
      debugln("[HTTP] Star ON");
      starSoll = true;
      fadeReset();
      EEPROM.write(starAdress, 1);
      EEPROM.commit();
      break;
    case 3: // Aus Star
      if (starMode)
      {
        debugln("[HTTP] Star-Modus Manuell"); // Manually Mode
        starMode = false;
        fadeReset();
        EEPROM.write(betriebsmodusAdress, 0);
        EEPROM.commit();
      }
      debugln("[HTTP] Star OFF");
      starSoll = false;
      fadeReset();
      EEPROM.write(starAdress, 0);
      EEPROM.commit();
      break;

    default:
      break;
    }
  }
  else if (id == 1) // *** TFL SLIDER ***
  {
    switch (sliderValues[id])
    {
    case 1: // Automatik Star
      debugln("[HTTP] TFL-Modus Automatik");
      tflMode = true;
      EEPROM.write(tflModeAdress, 1);
      EEPROM.commit();
      break;
    case 2: // An TFL
      if (tflMode)
      {
        debugln("[HTTP] TFL-Modus Manuell");
        tflMode = false;
        EEPROM.write(tflModeAdress, 0);
        EEPROM.commit();
      }
      debugln("[HTTP] TFL ON");
      tflSoll = true;
      EEPROM.write(tflAdress, 1);
      EEPROM.commit();
      break;
    case 3: // Aus TFL
      if (tflMode)
      {
        debugln("[HTTP] TFL-Modus Manuell");
        tflMode = false;
        EEPROM.write(tflModeAdress, 0);
        EEPROM.commit();
      }
      debugln("[HTTP] TFL OFF");
      tflSoll = false;
      EEPROM.write(tflAdress, 0);
      EEPROM.commit();
      break;

    default:
      break;
    }
  }
  else if (id == 2) // *** STAR FADE TIME SLIDER ***
  {
    switch (sliderValues[id])
    {
    case 1:
      debugln("[HTTP] Star Fade-Time: Slow");
      starFadePause = fadePauseSlow;
      EEPROM.write(starFadePauseAdress, 1);
      EEPROM.commit();
      break;
    case 2:
      debugln("[HTTP] Star Fade-Time: Normal");
      starFadePause = fadePauseNormal;
      EEPROM.write(starFadePauseAdress, 2);
      EEPROM.commit();
      break;
    case 3:
      debugln("[HTTP] Star Fade-Time: Fast");
      starFadePause = fadePauseFast;
      EEPROM.write(starFadePauseAdress, 3);
      EEPROM.commit();
      break;

    default:
      break;
    }
  }
  else if (id == 3) // *** TFL FADE TIME SLIDER ***
  {
    switch (sliderValues[id])
    {
    case 1:
      debugln("[HTTP] TFL Fade-Time: Slow");
      tflFadePause = fadePauseSlow;
      EEPROM.write(tflFadePauseAdress, 1);
      EEPROM.commit();
      break;
    case 2:
      debugln("[HTTP] TFL Fade-Time: Normal");
      tflFadePause = fadePauseNormal;
      EEPROM.write(tflFadePauseAdress, 2);
      EEPROM.commit();
      break;
    case 3:
      debugln("[HTTP] TFL Fade-Time: Fast");
      tflFadePause = fadePauseFast;
      EEPROM.write(tflFadePauseAdress, 3);
      EEPROM.commit();
      break;

    default:
      break;
    }
  }
  else if (id == 4) // *** UGLW BRTNS SLIDER ***
  {
    debugln("[HTTP] Underglow Brightness: " + String(sliderValues[4]) + " was selected!");
    uglw_sendValue(2, sliderValues[4], true);
  }
  else if (id == 5) // *** UGLW SPEED SLIDER ***
  {
    debugln("[HTTP] Underglow Speed: " + String(sliderValues[5]) + " was selected!");
    uglw_sendValue(3, sliderValues[5], true);
  }
  else if (id == 6) // *** BATVOLT THRESHOLD SLIDER ***
  {
    debugln("[HTTP] Battery-Voltage Threshold " + String(sliderValuesFloat[0]) + " Volt was selected!");
    batteryThreshold = sliderValuesFloat[0];
    EEPROM.put(batVoltThresAdress, batteryThreshold);
    EEPROM.commit();
  }
  else if (id == 7) // *** BATVOLT OFFSET INPUTTEXT ***
  {
    debugln("[HTTP] Battery-Voltage Offset " + String(sliderValuesFloat[1]) + " Volt was selected!");
    if (motorRunning)
      batVoltOffset = sliderValuesFloat[1] + motorVoltOffset;
    else
      batVoltOffset = sliderValuesFloat[1];
    EEPROM.put(batVoltOffsetAdress, batVoltOffset);
    EEPROM.commit();
  }
  else if (id == 8) // *** TRANSITION COEFFICIENT ***
  {
    debugln("[HTTP] Transition Coefficient " + String(sliderValuesFloat[2]) + " was selected!");
    transitionCoefficient = sliderValuesFloat[2];
    uglw_sendValue(6, transitionCoefficient, false);
    EEPROM.put(transitionCoefficientAdress, transitionCoefficient);
    EEPROM.commit();
  }
  else if (id == 9) // *** BATVOLT OFFSET MOTOR INPUTTEXT ***
  {
    debugln("[HTTP] Battery-Voltage Offset Motor " + String(sliderValuesFloat[3]) + " Volt was selected!");
    motorVoltOffset = sliderValuesFloat[3];
    EEPROM.put(batVoltOffsetMotorAdress, motorVoltOffset);
    EEPROM.commit();
  }
  else if (id == 11) // *** FAVORITE MODE INPUTTEXT ***
  {
    debugln("[HTTP] Favorite UGLW Mode " + String(favoriteMode) + " was selected!");
    EEPROM.write(favoriteModeAdress, favoriteMode);
    debugln("[HTTP] Favorite UGLW Color " + String(favoriteColor) + " was selected!");
    writeColorEEPROM(favoriteColor, true);
    debugln("[HTTP] Favorite UGLW Brightness " + String(favoriteBrtns) + " was selected!");
    EEPROM.write(favoriteBrtnsAdress, favoriteBrtns);
    debugln("[HTTP] Favorite UGLW Speed " + String(favoriteSpeed) + " was selected!");
    writeSpeedEEPROM(favoriteSpeed, true);
    EEPROM.commit();
  }
}

void interpretButton(int id)
{
  if (id == 0)
  {
    if (buttonStates[id])
    {
      debugln("[HTTP] Strobe: On");
      debugln("\n selectedModeBef = " + String(selectedModeBef) + " | selectedMode = " + String(selectedMode));
      if (selectedModeBef != 4)
      {
        strobe = true;
        httpStrobe = true;
        EEPROM.write(strobeAdress, 1);
      }
    }
    else
    {
      debugln("[HTTP] Strobe: Off");
      debugln("\n selectedModeBef = " + String(selectedModeBef) + " | selectedMode = " + String(selectedMode));
      if (selectedModeBef != 4)
      {
        strobe = false;
        httpStrobe = false;
        EEPROM.write(strobeAdress, 0);
      }
    }
    EEPROM.commit();
  }
  if (id == 1)
  {
    if (buttonStates[id])
    {
      debugln("[HTTP] Fade-Mode ON");
      fadeMode = true;
      fadeReset();
      EEPROM.write(fademodusAdress, 1);
    }
    else
    {
      debugln("[HTTP] Fade-Mode OFF");
      fadeMode = false;
      fadeReset();
      EEPROM.write(fademodusAdress, 0);
    }
    EEPROM.commit();
  }
  if (id == 2)
  {
    if (buttonStates[id])
    {
      debugln("[HTTP] Star Motor-Rest.: On");
      starMotorRestriction = true;
      EEPROM.write(starMotorRestrictionAdress, 1);
    }
    else
    {
      debugln("[HTTP] Star Motor-Rest.: Off");
      starMotorRestriction = false;
      EEPROM.write(starMotorRestrictionAdress, 0);
    }
    EEPROM.commit();
  }
  if (id == 3)
  {
    if (buttonStates[id])
    {
      debugln("[HTTP] UGLW Motor-Rest.: On");
      uglwMotorRestriction = true;
      EEPROM.write(uglwMotorRestrictionAdress, 1);
    }
    else
    {
      debugln("[HTTP] UGLW Motor-Rest.: Off");
      uglwMotorRestriction = false;
      EEPROM.write(uglwMotorRestrictionAdress, 0);
    }
    EEPROM.commit();
  }
  if (id == 4)
  {
    if (buttonStates[id])
    {
      debugln("[HTTP] UGLW TFL-Rest.: On");
      uglwTFLRestriction = true;
      EEPROM.write(uglwTFLRestrictionAdress, 1);
    }
    else
    {
      debugln("[HTTP] UGLW TFL-Rest.: Off");
      uglwTFLRestriction = false;
      EEPROM.write(uglwTFLRestrictionAdress, 0);
    }
    EEPROM.commit();
  }
}

// Underglow Handlers
void uglw_sendValue(unsigned int dropdown, float key, bool overwrite = false) // 0 - mode | 1 - color | 2 - brtns | 3 - speed | 4 - Motor-Restriction | 5 - Data-Transmission | 6 - Transition Coefficient
{
  String retval = "";
  unsigned int payload = 0;
  if (dropdown == 0 && key >= 0 && key <= 56) // Mode
  {
    unsigned int keyI = (unsigned int)key;
    switch (keyI)
    {
    case 0:
      retval = "Mode Static";
      break;
    case 1:
      retval = "Mode Blink";
      break;
    case 2:
      retval = "Mode Breath";
      break;
    case 3:
      retval = "Mode Color Wipe";
      break;
    case 7:
      retval = "Mode Color Wipe Random";
      break;
    case 8:
      retval = "Mode Random Color";
      break;
    case 9:
      retval = "Mode Single Dynamic";
      break;
    case 10:
      retval = "Mode Multi Dynamic";
      break;
    case 11:
      retval = "Mode Rainbow Even";
      break;
    case 12:
      retval = "Mode Rainbow Cycle";
      break;
    case 13:
      retval = "Mode Scan";
      break;
    case 14:
      retval = "Mode Dual Scan";
      break;
    case 15:
      retval = "Mode Breath Fast";
      break;
    case 16:
      retval = "Mode Theater Chase";
      break;
    case 17:
      retval = "Mode Theater Chase Rainbow";
      break;
    case 18:
      retval = "Mode Running Lights";
      break;
    case 19:
      retval = "Mode Twinkle";
      break;
    case 20:
      retval = "Mode Twinkle Random";
      break;
    case 21:
      retval = "Mode Twinkle Fade";
      break;
    case 22:
      retval = "Mode Twinkle Fade Random";
      break;
    case 23:
      retval = "Mode Sparkle";
      break;
    case 26:
      retval = "Mode Strobe";
      break;
    case 27:
      retval = "Mode Strobe Rainbow";
      break;
    case 29:
      retval = "Mode Blink Rainbow";
      break;
    case 32:
      retval = "Mode Chase Random";
      break;
    case 33:
      retval = "Mode Chase Rainbow";
      break;
    case 38:
      retval = "Mode Chase Blackout Rainbow";
      break;
    case 40:
      retval = "Mode Running Color";
      break;
    case 41:
      retval = "Mode Running Red-Blue";
      break;
    case 42:
      retval = "Mode Running Random";
      break;
    case 43:
      retval = "Mode Larson Scanner";
      break;
    case 44:
      retval = "Mode Comet";
      break;
    case 45:
      retval = "Mode Fireworks";
      break;
    case 46:
      retval = "Mode Fireworks Random";
      break;
    case 47:
      retval = "Mode Merry Christmas";
      break;
    case 49:
      retval = "Mode Fire Flicker";
      break;
    case 51:
      retval = "Mode Circus Combustus";
      break;
    case 52:
      retval = "Mode Halloween";
      break;
    case 55:
      retval = "Mode TwinkleFox";
      break;
    case 56:
      retval = "Mode Rain";
      break;

    default:
      retval = "Mode " + String(keyI);
      break;
    }
    if (overwrite)
    {
      debugln("\n[LED] Mode was changed!");
      led_mode = keyI;
      EEPROM.write(modeAdress, keyI);
      EEPROM.commit();
      if (selectedMode == 1)
        ledSerial.print("mode!" + String(keyI) + "$");
      else if (selectedMode == 2)
        mqttClient.publish(fxmode_topic, 0, 0, "2");
    }
    else
      ledSerial.print("mode!" + String(keyI) + "$");
  }
  else if (dropdown == 1 && key >= 0 && key <= 16777215) // Color
  {
    unsigned int keyI = (unsigned int)key;
    switch (keyI)
    {
    case 0:
      retval = "Color Red";
      payload = 16711680;
      break;
    case 1:
      retval = "Color Green";
      payload = 65280;
      break;
    case 2:
      retval = "Color Blue";
      payload = 255;
      break;
    case 3:
      retval = "Color White";
      payload = 16777215;
      break;
    case 4:
      retval = "Color Yellow";
      payload = 16776960;
      break;
    case 5:
      retval = "Color Cyan";
      payload = 65535;
      break;
    case 6:
      retval = "Color Magenta";
      payload = 16711935;
      break;
    case 7:
      retval = "Color Purple";
      payload = 4194432;
      break;
    case 8:
      retval = "Color Orange";
      payload = 16723968;
      break;
    case 9:
      retval = "Color Pink";
      payload = 16716947;
      break;

    default:
      retval = "Color " + String(keyI);
      payload = keyI;
      break;
    }
    if (overwrite)
    {
      debugln("\n[LED] Color was changed!");
      led_color = payload;
      writeColorEEPROM(payload, false);
      if (selectedMode == 1 || selectedMode == 2)
        ledSerial.print("color!" + String(payload) + "$");
    }
    else
      ledSerial.print("color!" + String(payload) + "$");
  }
  else if (dropdown == 2 && key >= 0 && key <= 255) // Brtns
  {
    unsigned int keyI = (unsigned int)key;
    retval = "Brightness " + String(keyI);
    if (overwrite)
    {
      debugln("\n[LED] Brtns was changed!");
      led_brtns = sliderValues[4];
      EEPROM.write(brtnsAdress, sliderValues[4]);
      EEPROM.commit();
      if (selectedMode == 1 || selectedMode == 2)
        ledSerial.print("brtns!" + String(keyI) + "$");
    }
    else
      ledSerial.print("brtns!" + String(keyI) + "$");
  }
  else if (dropdown == 3 && key >= 0 && key <= 65535) // Speed
  {
    unsigned int keyI = (unsigned int)key;
    retval = "Speed " + String(keyI);
    if (overwrite)
    {
      debugln("\n[LED] Speed was changed!");
      led_speed = sliderValues[5];
      writeSpeedEEPROM(sliderValues[5], false);
      if (selectedMode == 1 || selectedMode == 2)
        ledSerial.print("speed!" + String(keyI) + "$");
    }
    else
      ledSerial.print("speed!" + String(keyI) + "$");
  }
  else if (dropdown == 4 && key >= 0 && key <= 3)
  {
    unsigned int keyI = (unsigned int)key;
    ledSerial.print("motor!" + String(keyI) + "$");
    retval = "Motor-Restriction " + String(keyI);
  }
  else if (dropdown == 5)
  {
    unsigned int keyI = (unsigned int)key;
    ledSerial.print("trans!" + String(keyI) + "$");
    retval = "Data-Transmission " + String(keyI);
  }
  else if (dropdown == 6)
  {
    ledSerial.print(String(transitionCoefficient_topic) + "!" + String(key) + "$");
    retval = "Data-Transmission " + String(key);
  }
  debugln("\n[HTTP] Underglow " + retval + " was selected!");
}

// MQTT
void mqttAliveMessage()
{
  if (((millis() > (myLastAveMsg + aveMsgIntervall)) || (myLastAveMsg == 0U)) && mqttClient.connected())
  {
    myLastAveMsg = millis();
    mqttClient.publish(alive_topic, 0, false, starhost_avemsg);
  }

  if (millis() > (yourlastAveMsg + aveMsgTimeout) && emgcyWasConnected && !lostEmergencyConnection)
  {
    yourlastAveMsg = millis();
    setEmergencyMode();
    lostEmergencyConnection = true;
    mqttClient.disconnect();
    debugln("\n[Timeout-WD] Emergency MQTT Client timed out!");
    delay(20);
    checkMQTT();
  }
  else if (millis() == (yourlastAveMsg + aveMsgTimeout - 2000) && emgcyWasConnected)
    debugln("\n[Timeout-WD] Emergency MQTT Client silent -2 seconds...");
  else if (millis() == (yourlastAveMsg + aveMsgTimeout - 1000) && emgcyWasConnected)
    debugln("\n[Timeout-WD] Emergency MQTT Client silent -1 second...");
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  String topicStr = String(topic);
  String payloadStr = String(payload);

  debugln("\n[MQTT] Received message - topic: " + topicStr + " | payload: " + payloadStr);

  if (topicStr == status_topic) // subscribed topic defined in header ! Topic subscription is done in the reconnect() Function below !
  {
    debugln("\n[MQTT] Subscribed topic - Status: " + payloadStr);
  }
  else if (topicStr == apiOvrOff_topic)
  {
    if (payloadStr == "1")
    {
      selectedMode = 1;
      emgcyWasConnected = true;
      apiOverrideOff = true;
      EEPROM.write(apiOverrideOffAdress, 1);
      EEPROM.commit();
      setEmergencyMode();
      lostEmergencyConnection = false;
      yourlastAveMsg = millis();
    }
    else if (payloadStr == "0")
    {
      selectedMode = 2;
      emgcyWasConnected = true;
      apiOverrideOff = false;
      EEPROM.write(apiOverrideOffAdress, 0);
      EEPROM.commit();
      resetEmergencyMode();
      lostEmergencyConnection = false;
      yourlastAveMsg = millis();
    }
    debugln("\n[MQTT] Subscribed topic - Setting 'apiOverrideOff' to '" + payloadStr + "'");
  }
  else if (topicStr == alive_topic)
  {
    if (payloadStr == emergency_avemsg)
    {
      yourlastAveMsg = millis();
      lostEmergencyConnection = false;
      debugln("\n[MQTT] Subscribed topic - Alive Message!");
    }
  }
  else if (topicStr == fxmode_topic)
  {
    selectedMode = payloadStr.toInt();
    switch (selectedMode)
    {
    case 1:
      selectedMode = 1;
      if (selectedModeBef == 4)
        strobeStop(false);
      debugln("\n[MQTT] Subscribed topic - FX Mode Blackout");
      break;

    case 2:
      selectedMode = 2;
      if (selectedModeBef == 4)
        strobeStop(false);
      if (strobeActiveMQTT)
      {
        selectedModeBef = 4;
        strobeActiveMQTT = false;
      }
      if (selectedModeBef == 2 || selectedModeBef == 3 || selectedModeBef == 4)
        uglw_sendValue(5, 0); // init data transmission
      uglw_sendValue(3, led_speed);
      uglw_sendValue(2, led_brtns);
      uglw_sendValue(1, led_color);
      uglw_sendValue(0, led_mode);
      if (selectedModeBef == 2 || selectedModeBef == 3)
        uglw_sendValue(5, 5); // transition duration 5ms * 100
      else if (selectedModeBef == 4)
        uglw_sendValue(5, 999); // transition duration 0ms
      if (selectedModeBef == 2 || selectedModeBef == 3 || selectedModeBef == 4)
        uglw_sendValue(5, 1); // finish data transmission
      if (uglwMotorBlock && !uglwMotorBlockSent)
      {
        uglw_sendValue(4, 1U, true);
        uglwMotorBlockSent = true;
      }
      debugln("\n[MQTT] Subscribed topic - FX Mode HTML Ctrld");
      break;

    case 3:
      selectedMode = 3;
      if (selectedModeBef == 4)
        strobeStop(false);
      if (strobeActiveMQTT)
      {
        selectedModeBef = 4;
        strobeActiveMQTT = false;
      }
      if (selectedModeBef == 2 || selectedModeBef == 4)
        uglw_sendValue(5, 0); // init data transmission
      uglw_sendValue(3, favoriteSpeed);
      uglw_sendValue(2, favoriteBrtns);
      uglw_sendValue(1, favoriteColor);
      uglw_sendValue(0, favoriteMode);
      if (selectedModeBef == 2)
        uglw_sendValue(5, 5); // transition duration 5ms * 100
      else if (selectedModeBef == 4)
        uglw_sendValue(5, 999); // transition duration 0ms
      if (uglwMotorBlock)
      {
        uglw_sendValue(4, 2U, true);
        uglwMotorBlockSent = false;
      }
      if (selectedModeBef == 2 || selectedModeBef == 4)
        uglw_sendValue(5, 1); // finish data transmission
      debugln("\n[MQTT] Subscribed topic - FX Mode Favorite");
      break;

    case 4:
      selectedMode = 4;
      if (selectedModeBef == 2 || selectedModeBef == 3)
        uglw_sendValue(5, 0); // init data transmission
      uglw_sendValue(3, led_speed_strobe);
      uglw_sendValue(2, 255U);
      uglw_sendValue(1, 16777215);
      uglw_sendValue(0, led_mode_strobe);
      if (selectedModeBef == 2 || selectedModeBef == 3)
      {
        uglw_sendValue(5, 999); // transition duration 0ms
        if (uglwMotorBlock)
        {
          uglw_sendValue(4, 2U, true);
          uglwMotorBlockSent = false;
        }
        uglw_sendValue(5, 1); // finish data transmission
      }
      else
      {
        if (uglwMotorBlock)
        {
          uglw_sendValue(4, 2U, true);
          uglwMotorBlockSent = false;
        }
      }

      strobeStart(false);

      debugln("\n[MQTT] Subscribed topic - FX Mode Strobe");
      break;

    case 5:
      debugln("\n[MQTT] Subscribed topic - FX Mode - ERROR thrown mode 5 (strobestop)");
      break;

    default:
      break;
    }
    if (!httpStrobe)
      selectedModeBef = selectedMode;
  }
  else
  {
    debugln("\n[MQTT] Not a subscribed topic!");
  }
}

void onMqttConnect(bool sessionPresent)
{
  debugln("\n[MQTT] Connection succesful!");
  mqttClient.subscribe(status_topic, 0);
  mqttClient.subscribe(apiOvrOff_topic, 0);
  mqttClient.subscribe(alive_topic, 0);
  mqttClient.subscribe(mode_topic, 0);
  mqttClient.subscribe(brtns_topic, 0);
  mqttClient.subscribe(color_topic, 0);
  mqttClient.subscribe(speed_topic, 0);
  mqttClient.subscribe(fxmode_topic, 0);
  mqttClient.publish(status_topic, 0, false, "starhost1 active!");
  delay(100);
}

bool checkMQTT()
{
  if (!mqttClient.connected())
  {
    if (emgcyWasConnected)
      setEmergencyMode();

    while (!mqttClient.connected())
    {
      mqttClient.disconnect();
      WiFi.disconnect(true, true);
      while (WiFi.status() == WL_CONNECTED)
        ;
      checkWiFi();

      debugln("\n[MQTT] Disconnected, trying to reconnect!");

      mqttClient.clearQueue();
      mqttClient.connect();

      while (!mqttClient.connected())
      {
        delay(10);
        if (WiFi.status() != WL_CONNECTED)
        {
          debugln("\n[MQTT] Aborted reconnection - no WiFi-Connection!");
          checkWiFi();
          debugln("\n[MQTT] Continuing reconnection - WiFi connected again!");
          mqttClient.connect();
        }
      }

      if (mqttClient.connected())
      {
        yourlastAveMsg = millis();
        return true;
      }
      else
      {
        debugln("\n[MQTT] Reconnection failed - trying again!");
      }
    }
    if (!apiOverrideOff)
      resetEmergencyMode();
  }
  return true;
}

bool checkWiFi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    if (mqttClient.connected())
      mqttClient.disconnect();
    beginWiFi();
    debug("\n[WiFi] Establishing WiFi connection as " + String(WiFi.getHostname()));
    unsigned long counter = 0;
    int retryCounter = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
      if (millis() > (counter + 500))
      {
        serialAliveMsg();
        counter = millis();
        debug(".");
        retryCounter++;
        if (retryCounter > 40)
        {
          beginWiFi();
          retryCounter = 0;
          debug("\n[WiFi] Resetting module...");
        }
      }
      yield();
    }
    debug("\n[WiFi] Connection successful! (IP: ");
    debug(WiFi.localIP());
    debugln(")");

    // OTA
    ArduinoOTA.setHostname("starhost");
    ArduinoOTA.begin();
    debugln("\n[OTA] Service started!");

    // AsyncWebServer
    assignServerHandlers();
    server.begin();
    debugln("\n[HTTP] Webserver started!");
    return true;
  }
  else
    return true;
}

void beginWiFi()
{
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.mode(WIFI_MODE_STA);
  String hostname = "starhost-" + String(random(100));
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(APSSID, APPSK);
}

// Emergency Handlers
void setEmergencyMode()
{
  if (!emergency)
  {
    emergency = true;
    ledSerial.print(String(apiOvrOff_topic) + "!1$");
    saveOutputParams();
    apiOverrideLights();
    debugln("\n[EMERGENCY] Mode activated!");
  }
}

void resetEmergencyMode()
{
  if (!apiOverrideOff && !batteryEmergency && emergency)
  {
    emergency = false;
    ledSerial.print(String(apiOvrOff_topic) + "!0$");
    restoreOutputParams();
    debugln("\n[EMERGENCY] Mode deactivated!");
  }
  else if (apiOverrideOff)
    debugln("\n[EMERGENCY] Mode partly deactivated - Light override active!");
  else if (batteryEmergency)
    debugln("\n[EMERGENCY] Mode partly deactivated - Battery-emergency active!");
}

// Serial LED Handlers
void uglwWriteOutput()
{
  uglwTFLRestrictionHandler();
  if (((uglwMotorRestriction && motorRunning) || uglwTFLRestActive) && !uglwMotorBlock)
  {
    uglwMotorBlockSent = true;
    if (selectedModeBef == 1 || selectedModeBef == 2) // only transmit motor-blockage if motor-restricted modes are active
    {
      uglwMotorBlock = true;
      uglw_sendValue(5, 5);
      uglw_sendValue(4, 1U, true);
    }
  }
  else if (((!uglwMotorRestriction || (uglwMotorRestriction && !motorRunning)) && !uglwTFLRestActive) && uglwMotorBlock)
  {
    uglwMotorBlockSent = true;
    if (selectedModeBef == 1 || selectedModeBef == 2)
    {
      uglwMotorBlock = false;
      uglw_sendValue(5, 5);
      uglw_sendValue(4, 0U, true);
    }
  }
}

void uglwTFLRestrictionHandler()
{
  if (uglwTFLRestriction && !tflBool && !uglwTFLRestActive)
  {
    uglwTFLRestActive = true;
    debugln("\n[UGLW] TFL Rest active!");
  }
  else if ((!uglwTFLRestriction || (uglwTFLRestriction && tflBool)) && uglwTFLRestActive)
  {
    uglwTFLRestActive = false;
    debugln("\n[UGLW] TFL Rest inactive!");
  }
}

void serialLEDHandler()
{
  if (ledSerial.available())
  {
    String topic = ledSerial.readStringUntil('!');
    String payload = ledSerial.readStringUntil('$');

    if (String(topic) == "status")
    {
      if (String(payload) == "cnt-wtg")
      {
        if (serialClientConnection || !serialClientInitConnection)
        {
          serialClientConnection = false;
          debugln("\n[Serial] Initializing client communication!");
          uglwStarted = true;
        }
        ledSerial.print("status!host-alive$");
        if (emergency)
          lastSerialMsg = String(apiOvrOff_topic) + "!1$";
        else
          lastSerialMsg = String(apiOvrOff_topic) + "!0$";
        delay(10);
        ledSerial.print(lastSerialMsg);
        uglwTFLRestrictionHandler();
        uglw_sendValue(6, transitionCoefficient);
        if (uglwMotorBlock || uglwTFLRestActive)
          uglw_sendValue(4, 1U, true);
        else if (!uglwMotorBlock && !uglwTFLRestActive)
          uglw_sendValue(4, 0U, true);
        debugln("[Serial] Message arrived - Topic: '" + topic + "' - Payload: '" + payload + "'\n");
      }
      else if (String(payload) == "cnctd")
      {
        serialClientConnection = true;
        if (!serialClientInitConnection)
          serialClientInitConnection = true;
        debugln("\n[Serial] Client successfully connected!");
        debugln("[Serial] Message arrived - Topic: '" + topic + "' - Payload: '" + payload + "'\n");
      }
    }
  }

  serialAliveMsg();
}

void serialAliveMsg()
{
  if (millis() > lastSAMSent + SAMTimeout)
  {
    lastSAMSent = millis();
    ledSerial.print("status!host-alive$");
  }
}