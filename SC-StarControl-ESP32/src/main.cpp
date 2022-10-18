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
  18/02/22 - V2.0
  ~ Updated whole system (starhost + emergency + underglow) to new generation 2.0
  ~ Added Github repository to track changes
  ~ New features include: battery voltage monitoring, underglow communication via hardware serial, emergency communication via wifi, 4 fxmodes (emergency, default, favorite, strobe), ...
  ~ Reviewed wifi and mqtt functions and introduced new handler controlled functions to be more efficient
  ~ Web page: new fresh and compact design with lots of controls (added underglow-, battery-controls and info-panels)
  23/02/22 - V2.01
  ~ OTA fixed - Modified StarEmergency to not restart itself in favorite mode (selMode = 3) if Starhost lost its mqtt connection
  - This fixes the issue of not being able to do an OTA update to StarHost (via StarEmergency - its the "router")
  ~ Fixed html site error of starStateText not being displayed correctly if turned on

  --- Bugs ---
  ~ ISR detection sometimes recognizes continuous HIGH before pwm detection has finished
  - Fix: NOT working all time, but prevents some occurrences - cont. HIGH has to recognize its pattern several times (7) before it is allowed to set final signal type
  ~ Strobe: If triggered via web interface, the button sometimes "hangs", so strobe mode is not disabled
  - Fix: tbd.
  ~ WiFi Startup: On the first startup of the controller after a reset, it experiences a brownout during startup of wifi module
  - Fix: Trigger the brownout in the beginning, because strangely this problem doesn't occur during the second reset (triggerd by brownout detector)
*/

//***** INCLUDES *****
// Own code
#include <htmlsite.h>
#include <led.h>
#include <wifihandler.h>

// Libraries
#include <EEPROM.h>
#include <esp_adc_cal.h>
#include <ESPAsyncWebServer.h>

//***** VARIABLES & OBJECTS *****
// Input State Variables
bool tflBool = false;
bool tflBoolBef = false;
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
portMUX_TYPE ISRSync = portMUX_INITIALIZER_UNLOCKED;
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
  portENTER_CRITICAL(&ISRSync);
  if (digitalRead(pin_tfl))
    tflISRrisingState = 1;
  else
    tflISRfallingState = 1;
  portEXIT_CRITICAL(&ISRSync);
}

// Hardware Serials
HardwareSerial canSerial(1);
HardwareSerial ledSerial(2);

// LED Serial
unsigned long lastSAMSent = 0;
const unsigned int SAMTimeout = 5000;
String lastSerialMsg = "";
bool serialClientConnection = false;
bool serialClientInitConnection = false;

// Client timeout
bool connected_clients[3] = {false, false, false}; // 0 - Emeg | 1 - Shift | 2 - CANChild
bool con_clients_emegBefore = false;               // Stores whether StarEmeg was once connected
bool apiOverrideOff;                               // Stores whether API call was received to turn lights off
bool emergency = true;
int outputParamsBefore[6]; // Stores previous states of analog / digital outputs of relais and leds
unsigned const int aveMsgTimeout = 10000;
const unsigned int aveMsgIntervall = 2500; // interval in which host sends can rqst to ping alive
unsigned long myLastAveMsg = 0;            // timer to send can ping rqst

// Star Emergency - API Emergency Mode
unsigned long emeg_lastAveMsg = 0; // stores when last alive msg was received

// Star ShiftGuidance
unsigned long shift_lastAveMsg = 0; // stores when last alive msg was received

// CAN Child
unsigned long canChild_lastAveMsg = 0; // stores when last alive msg was received

// AsyncWebServer
AsyncWebServer server(80);
const char *PARAM_INPUT_1 = "id";
const char *PARAM_INPUT_2 = "state";
const char *PARAM_INPUT_3 = "value";
bool buttonStates[5] = {false, false, false, false, false}; // 0 - Strobe, 1 - Fade, 2 - MREST Star, 3 - MREST UGLW, 4 - TFLREST UGLW
unsigned int sliderValues[7] = {3, 3, 2, 2, 0, 0, 0};       // 0 - Star, 1 - TFL, 2 - Star Fade Time, 3 - TFL Fade Time, 4 - UGLW Brtns, 5 - UGLW Speed , 6 - UGLW FadeSize
float sliderValuesFloat[4] = {12.0, 0, 1.18, 0};            // 0 - batVoltTresh | 1 - batVoltOffset | 2 - TransCoef | 3 - batVoltOffsetMotor                     // 0 - BatVoltThreshold, 1 - BatVoltOffset

// WS2812 LEDs
unsigned int led_mode = 0;
unsigned int led_color1 = 0;
unsigned int led_color2 = 0;
unsigned int led_color3 = 0;
unsigned int led_brtns = 0;
unsigned int led_speed = 10000;
uint8_t fadeSize;
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
unsigned int favoriteColor1;
unsigned int favoriteColor2;
unsigned int favoriteColor3;
unsigned int favoriteBrtns;
unsigned int favoriteSpeed;
uint8_t favoriteFadeSize;
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

// CAN
char CAN_childMsg_alive[3] = {'C', 'A', '\0'};
char CAN_childMsg_lives[3] = {'C', 'L', '\0'};

//***** PROTOTYPES *****
void handlers();
void startupHandler();
void eeprom_initLastState();
void writeColorEEPROM(unsigned int, bool, unsigned int);
unsigned int readColorEEPROM(bool, unsigned int);
void writeSpeedEEPROM(unsigned int, bool);
unsigned int readSpeedEEPROM(bool);
void httpHandler();
void readInputs();
void batteryMonitoring();
void interpretInputs(); // check for engine star or stop
void writeOutput();
void automaticMode();
void manualMode();
void starOn();
void starOff();
void fadeReset();
void fadeIn();
void fadeOut();
void tflOutput();
void automaticTFL();
void manualTFL();
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
void server_init();
String uglwSelectedModeRqst();
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
void uglw_modesel1();
void uglw_modesel2();
void uglw_modesel3();
void uglw_modesel4();
void CAN_sendMessage(unsigned long, byte, byte[]);
void CAN_aliveMessage();
uint8_t CAN_checkMessages();
void CAN_onConnect_Emeg();
void CAN_onDisconnect_Emeg();
void wifi_startAP();
void wifi_eventHandler(WiFiEvent_t);
void setEmergencyMode();
void resetEmergencyMode();
void uglwWriteOutput();
void uglwTFLRestrictionHandler();
void serialLEDHandler();
void serialAliveMsg();
bool string_find(char *, char *);

//***** SETUP *****
void setup()
{
  // Initialisation
  if (DEBUG || ISRDEBUG)
  {
    Serial.begin(115200);
    while (!Serial)
      ;
  }

  debugln("\n[StarControl-Host] Starting programm ~ by spl01t*#7");
  debugln("[StarControl-Host] You are running version " + String(VERSION) + "!");

  // WiFi
  WiFi.onEvent(wifi_eventHandler);
  wifi_startAP();
  server_init();

  // UART Interfaces
  canSerial.begin(38400);
  ledSerial.begin(115200);
  canSerial.setTimeout(3);
  ledSerial.setTimeout(3);
  while (!canSerial || !ledSerial)
    ;

  // Get EEPROM memory
  EEPROM.begin(56);
  eeprom_initLastState();

  // IOs
  pinMode(pin_tfl, INPUT); // interrupt
  attachInterrupt(digitalPinToInterrupt(pin_tfl), tflISR, CHANGE);
  pinMode(pin_kl15, INPUT);            // only digitalRead
  pinMode(pin_kl50, INPUT);            // only digitalRead
  pinMode(pin_tflLeft, OUTPUT);        // pwm
  pinMode(pin_tflRight, OUTPUT);       // pwm
  pinMode(pin_star, OUTPUT);           // pwm
  pinMode(pin_starRelais, OUTPUT);     // only digitalRead/-Write
  pinMode(pin_tflLeftRelais, OUTPUT);  // only digitalRead/-Write
  pinMode(pin_tflRightRelais, OUTPUT); // only digitalRead/-Write
  pinMode(pin_batVolt, INPUT);         // adc input

  // PWM Setup
  for (int i = 0; i < 3; i++)
  {
    ledcSetup(i, 100, 10);
  }
  ledcAttachPin(pin_star, 0);
  ledcAttachPin(pin_tflLeft, 1);
  ledcAttachPin(pin_tflRight, 2);
  ledcWrite(0, 0);                   // led star in beginning off
  ledcWrite(1, 0);                   // tfl l in beginning off
  ledcWrite(2, 0);                   // tfl r in beginning off
  digitalWrite(pin_starRelais, LOW); // relais in beginnning off
  digitalWrite(pin_tflLeftRelais, LOW);
  digitalWrite(pin_tflRightRelais, LOW);

  // ADC
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  vref = adc_chars.vref;

  debugln("\n[StarControl-Host] Initialization completed...starting programm loop\n");
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

    CAN_aliveMessage();

    CAN_checkMessages();

    startupHandler();
  } while (!starStarted || !tflStarted || !uglwStarted);

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
  kl15Bool = digitalRead(pin_kl15);
  kl50Bool = digitalRead(pin_kl50);

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
    float anaVal = (float)analogRead(pin_batVolt) * 0.0002442;
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
    if (selectedModeBef != 3)
      tflISRInterpret(); // V1.4 - TFL ISR checking for input signal type
    else
    {
      if (!checkedInputs)
        checkedInputs = true;
      if (!tflBool)
        tflBool = true;
    }
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
        manualMode();

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

void manualMode()
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
    manualTFL();
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

void manualTFL()
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
    uglw_modesel4();
    debugln("\n[STROBE] Current selected Mode: " + String(selectedModeBefStrobe));
  }

  strobeDataBefore[0] = ledcRead(0);
  strobeDataBefore[1] = ledcRead(1);
  strobeDataBefore[2] = ledcRead(2);
  strobeDataBefore[3] = digitalRead(pin_starRelais);
  strobeDataBefore[4] = digitalRead(pin_tflLeftRelais);
  strobeDataBefore[5] = digitalRead(pin_tflRightRelais);

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
    debugln("\n[STROBE] Loading before selected Mode: " + String(selectedModeBefStrobe));
    if (selectedModeBefStrobe == 1)
      uglw_modesel1();
    else if (selectedModeBefStrobe == 2)
      uglw_modesel2();
    else if (selectedModeBefStrobe == 3)
      uglw_modesel3();
    else if (selectedModeBefStrobe == 4)
      uglw_modesel4();
    else
      debugln("\n[STROBE] FX Mode - ERROR thrown concerning mode (strobestop)");
  }

  ledcWrite(0, strobeDataBefore[0]);
  ledcWrite(1, strobeDataBefore[1]);
  ledcWrite(2, strobeDataBefore[2]);
  digitalWrite(pin_starRelais, strobeDataBefore[3]);
  digitalWrite(pin_tflLeftRelais, strobeDataBefore[4]);
  digitalWrite(pin_tflRightRelais, strobeDataBefore[5]);

  EEPROM.write(strobeAdress, 0);
  EEPROM.commit();
}

// Relais Functions
void tflRelais(bool mode)
{
  if (mode)
  {
    digitalWrite(pin_tflLeftRelais, HIGH);
    digitalWrite(pin_tflRightRelais, HIGH);
  }
  else
  {
    digitalWrite(pin_tflLeftRelais, LOW);
    digitalWrite(pin_tflRightRelais, LOW);
  }
}

void starRelais(bool mode)
{
  if (mode)
  {
    digitalWrite(pin_starRelais, HIGH);
  }
  else
  {
    digitalWrite(pin_starRelais, LOW);
  }
}

// Save & Restore Output Parameters - V1.6 API Client Override Light Scene
void saveOutputParams()
{
  outputParamsBefore[0] = ledcRead(0);
  outputParamsBefore[1] = ledcRead(1);
  outputParamsBefore[2] = ledcRead(2);
  outputParamsBefore[3] = digitalRead(pin_starRelais);
  outputParamsBefore[4] = digitalRead(pin_tflLeftRelais);
  outputParamsBefore[5] = digitalRead(pin_tflRightRelais);
}

void restoreOutputParams()
{
  if (motorRunning && starMotorRestriction)
    ledcWrite(0, 0);
  else
    ledcWrite(0, outputParamsBefore[0]);
  ledcWrite(1, outputParamsBefore[1]);
  ledcWrite(2, outputParamsBefore[2]);
  digitalWrite(pin_starRelais, outputParamsBefore[3]);
  digitalWrite(pin_tflLeftRelais, outputParamsBefore[4]);
  digitalWrite(pin_tflRightRelais, outputParamsBefore[5]);
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
    if (digitalRead(pin_tfl))
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
void eeprom_initLastState()
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

  debugln("[EEPROM] Fade Modus: " + String(fadeContent));

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
    EEPROM.write(fademodusAdress, 1); // then write fade on
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
    buttonStates[2] = 1;
  else
    buttonStates[2] = 0;

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

  // LEDs Color 1
  unsigned int colorContent1 = readColorEEPROM(false, 1);

  debugln("[EEPROM] LEDs - Color 1: " + String(colorContent1));

  if (colorContent1 >= 0 && colorContent1 <= 16777215)
  {
    led_color1 = colorContent1;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Color 1 - Out of range (0x0-0xFFFFFF)!");
    led_color1 = 0;
    EEPROM.write(color1_1Adress, led_color1); // then no color
    EEPROM.write(color1_2Adress, led_color1); // then no color
    EEPROM.write(color1_3Adress, led_color1); // then no color
  }

  // LEDs Color 2
  unsigned int colorContent2 = readColorEEPROM(false, 2);

  debugln("[EEPROM] LEDs - Color 2: " + String(colorContent2));

  if (colorContent2 >= 0 && colorContent2 <= 16777215)
  {
    led_color2 = colorContent2;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Color 2 - Out of range (0x0-0xFFFFFF)!");
    led_color2 = 0;
    EEPROM.write(color2_1Adress, led_color2); // then no color
    EEPROM.write(color2_2Adress, led_color2); // then no color
    EEPROM.write(color2_3Adress, led_color2); // then no color
  }

  // LEDs Color 3
  unsigned int colorContent3 = readColorEEPROM(false, 3);

  debugln("[EEPROM] LEDs - Color 3: " + String(colorContent3));

  if (colorContent3 >= 0 && colorContent3 <= 16777215)
  {
    led_color3 = colorContent3;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Color 3 - Out of range (0x0-0xFFFFFF)!");
    led_color3 = 0;
    EEPROM.write(color2_1Adress, led_color3); // then no color
    EEPROM.write(color2_2Adress, led_color3); // then no color
    EEPROM.write(color2_3Adress, led_color3); // then no color
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

  // Favorite UGLW Color 1
  unsigned int favColor1 = readColorEEPROM(true, 1);

  debugln("[EEPROM] Favorite UGLW Color 1: " + String(favColor1));

  if (favColor1 >= 0 && favColor1 <= 16777215)
  {
    favoriteColor1 = favColor1;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Favorite Color 1 - Out of range (0x0-0xFFFFFF)!");
    writeColorEEPROM(0, true, 1U);
    favoriteColor1 = 0;
  }

  // Favorite UGLW Color 2
  unsigned int favColor2 = readColorEEPROM(true, 2);

  debugln("[EEPROM] Favorite UGLW Color 2: " + String(favColor2));

  if (favColor2 >= 0 && favColor2 <= 16777215)
  {
    favoriteColor2 = favColor2;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Favorite Color 2 - Out of range (0x0-0xFFFFFF)!");
    writeColorEEPROM(0, true, 1U);
    favoriteColor2 = 0;
  }

  // Favorite UGLW Color 3
  unsigned int favColor3 = readColorEEPROM(true, 3);

  debugln("[EEPROM] Favorite UGLW Color 3: " + String(favColor3));

  if (favColor3 >= 0 && favColor3 <= 16777215)
  {
    favoriteColor3 = favColor3;
  }
  else // if no logic state
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Favorite Color 3 - Out of range (0x0-0xFFFFFF)!");
    writeColorEEPROM(0, true, 1U);
    favoriteColor3 = 0;
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

  debugln("[EEPROM] Favorite UGLW Speed: " + String(favSpeed));

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

  // Favorite UGLW FadeSize
  uint8_t favfs = (uint8_t)EEPROM.read(favoriteFadeSizeAdress);

  debugln("[EEPROM] Favorite UGLW FadeSize: " + String(favfs));

  if (favfs >= 1 && favfs <= 4)
  {
    favoriteFadeSize = favfs;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Favorite FadeSize - Out of range (1-4)!");
    favoriteFadeSize = 2;
    EEPROM.write(favoriteFadeSizeAdress, favoriteFadeSize);
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

  // UGLW FadeSize
  uint8_t fadeSizeContent = EEPROM.read(fadeSizeAdress);

  debugln("[EEPROM] LEDs - FadeSize: " + String(fadeSizeContent));

  if (fadeSizeContent >= 1 && fadeSizeContent <= 4)
  {
    fadeSize = fadeSizeContent;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - FadeSize - Out of range (1-4)!");
    fadeSize = 2;
    EEPROM.write(fadeSizeAdress, fadeSize);
  }

  sliderValues[6] = fadeSize;

  EEPROM.commit();

  debugln("[EEPROM] Extraction completed!");
}

void writeColorEEPROM(unsigned int key, bool favorite, unsigned int colorType)
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

  if (colorType == 1)
  {
    EEPROM.write(favorite ? favoriteColorAdress1_1 : color1_1Adress, lowbit);  // Blue Bit
    EEPROM.write(favorite ? favoriteColorAdress1_2 : color1_2Adress, midbit);  // Green Bit
    EEPROM.write(favorite ? favoriteColorAdress1_3 : color1_3Adress, highbit); // Red Bit
  }
  else if (colorType == 2)
  {
    EEPROM.write(favorite ? favoriteColorAdress2_1 : color2_1Adress, lowbit);  // Blue Bit
    EEPROM.write(favorite ? favoriteColorAdress2_2 : color2_2Adress, midbit);  // Green Bit
    EEPROM.write(favorite ? favoriteColorAdress2_3 : color2_3Adress, highbit); // Red Bit
  }
  else if (colorType == 3)
  {
    EEPROM.write(favorite ? favoriteColorAdress3_1 : color3_1Adress, lowbit);  // Blue Bit
    EEPROM.write(favorite ? favoriteColorAdress3_2 : color3_2Adress, midbit);  // Green Bit
    EEPROM.write(favorite ? favoriteColorAdress3_3 : color3_3Adress, highbit); // Red Bit
  }
  EEPROM.commit();
}

unsigned int readColorEEPROM(bool favorite, unsigned int colorType)
{
  int key;

  if (colorType == 1)
  {
    key = EEPROM.read(favorite ? favoriteColorAdress1_3 : color1_3Adress) << 8;         // Red Bit
    key = (key + EEPROM.read(favorite ? favoriteColorAdress1_2 : color1_2Adress)) << 8; // Green Bit
    key += EEPROM.read(favorite ? favoriteColorAdress1_1 : color1_1Adress);             // Blue Bit
  }
  else if (colorType == 2)
  {
    key = EEPROM.read(favorite ? favoriteColorAdress2_3 : color2_3Adress) << 8;         // Red Bit
    key = (key + EEPROM.read(favorite ? favoriteColorAdress2_2 : color2_2Adress)) << 8; // Green Bit
    key += EEPROM.read(favorite ? favoriteColorAdress2_1 : color2_1Adress);             // Blue Bit
  }
  else if (colorType == 3)
  {
    key = EEPROM.read(favorite ? favoriteColorAdress3_3 : color3_3Adress) << 8;         // Red Bit
    key = (key + EEPROM.read(favorite ? favoriteColorAdress3_2 : color3_2Adress)) << 8; // Green Bit
    key += EEPROM.read(favorite ? favoriteColorAdress3_1 : color3_1Adress);             // Blue Bit
  }

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
void server_init()
{
  assignServerHandlers();
  server.begin();
  debugln("\n[HTTP] Webserver started!");
}

String processor(const String &var)
{
  if (var == "STARSTATETEXT")
  {
    return readStarState();
  }
  else if (var == "TFLSTATETEXT")
  {
    return readTFLState();
  }
  else if (var == "BUTTONPLACEHOLDER0")
  {
    String buttons = "";
    buttons += "<h4>Fade</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"updateButton(this)\" id=\"button1\" " + outputState(1) + "><span class=\"btnslider\"></span></label>";
    return buttons;
  }
  else if (var == "BUTTONPLACEHOLDER1")
  {
    String buttons = "";
    buttons += "<h4>Motor Restriction Star</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"updateButton(this)\" id=\"button2\" " + outputState(2) + "><span class=\"btnslider\"></span></label>";
    return buttons;
  }
  else if (var == "BUTTONPLACEHOLDER2")
  {
    String buttons = "";
    buttons += "<h4>Motor Restriction UGLW</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"updateButton(this)\" id=\"button3\" " + outputState(3) + "><span class=\"btnslider\"></span></label>";
    return buttons;
  }
  else if (var == "BUTTONPLACEHOLDER3")
  {
    String buttons = "";
    buttons += "<h4>TFL Restriction UGLW</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"updateButton(this)\" id=\"button4\" " + outputState(4) + "><span class=\"btnslider\"></span></label>";
    return buttons;
  }
  else if (var == "SLIDERTEXT0")
  {
    switch (sliderValues[0])
    {
    case 1:
      return "Star - Automatic";
      break;
    case 2:
      return "Star - On";
      break;
    case 3:
      return "Star - Off";
      break;

    default:
      break;
    }
  }
  else if (var == "SLIDERVALUE0")
  {
    return String(sliderValues[0]);
  }
  else if (var == "SLIDERTEXT1")
  {
    switch (sliderValues[1])
    {
    case 1:
      return "TFL - Automatic";
      break;
    case 2:
      return "TFL - On";
      break;
    case 3:
      return "TFL - Off";
      break;

    default:
      break;
    }
  }
  else if (var == "SLIDERVALUE1")
  {
    return String(sliderValues[1]);
  }
  else if (var == "SLIDERTEXT2")
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
  else if (var == "SLIDERVALUE2")
  {
    return String(sliderValues[2]);
  }
  else if (var == "SLIDERTEXT3")
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
  else if (var == "SLIDERVALUE3")
  {
    return String(sliderValues[3]);
  }
  else if (var == "SLIDERTEXT4")
  {
    String retval = "Brightness - " + String(sliderValues[4]);
    return retval;
  }
  else if (var == "SLIDERVALUE4")
  {
    return String(sliderValues[4]);
  }
  else if (var == "SLIDERTEXT5")
  {
    String retval = "Speed - " + String(sliderValues[5]);
    return retval;
  }
  else if (var == "SLIDERVALUE5")
  {
    return String(sliderValues[5]);
  }
  else if (var == "INFOTEXT1")
  {
    String retval = "";
    if (connected_clients[2])
      retval += "CC";
    else
      retval += "--";
    retval += " | ";
    if (connected_clients[0])
      retval += "SE";
    else
      retval += "--";
    retval += " | ";
    if (connected_clients[1])
      retval += "SG";
    else
      retval += "--";
    return retval;
  }
  else if (var == "INFOTEXT2")
  {
    return "Version - " + String(VERSION);
  }
  else if (var == "INFOTEXT3")
  {
    return "Battery-Voltage - " + String(batteryVoltage) + " Volt";
  }
  else if (var == "SLIDERTEXT6")
  {
    String retval = "Threshold - " + String(sliderValuesFloat[0]) + " Volt";
    return retval;
  }
  else if (var == "SLIDERVALUE6")
  {
    return String(sliderValuesFloat[0]);
  }
  else if (var == "SLIDERTEXT7")
  {
    String retval = "Calibration Offset - " + String(sliderValuesFloat[1]) + " Volt";
    return retval;
  }
  else if (var == "SLIDERTEXT8")
  {
    String retval = "Favorite Mode - " + String(favoriteMode);
    return retval;
  }
  else if (var == "SLIDERTEXT9")
  {
    String retval = "Motor Offset - " + String(sliderValuesFloat[3]) + " Volt";
    return retval;
  }
  else if (var == "SLIDERTEXT10")
  {
    String retval = "Transition Coefficient - " + String(sliderValuesFloat[2]);
    return retval;
  }
  else if (var == "SLIDERTEXT11")
  {
    switch (sliderValues[6])
    {
    case 1:
      return "Fadesize - Small";
      break;
    case 2:
      return "Fadesize - Medium";
      break;
    case 3:
      return "Fadesize - Large";
      break;
    case 4:
      return "Fadesize - XLarge";
      break;

    default:
      break;
    }
  }
  else if (var == "SLIDERVALUE11")
  {
    return String(sliderValues[6]);
  }
  else if (var == "UGLWSELMODETEXT")
  {
    return "Selected Mode - " + uglwSelectedModeRqst();
  }

  return String();
}

String uglwSelectedModeRqst()
{
  String retval;
  switch (led_mode)
  {
  case 0:
    retval = "Static";
    break;
  case 1:
    retval = "Blink";
    break;
  case 2:
    retval = "Breath";
    break;
  case 3:
    retval = "Color Wipe";
    break;
  case 7:
    retval = "Color Wipe Random";
    break;
  case 8:
    retval = "Random Color";
    break;
  case 9:
    retval = "Single Dynamic";
    break;
  case 10:
    retval = "Multi Dynamic";
    break;
  case 11:
    retval = "Rainbow Even";
    break;
  case 12:
    retval = "Rainbow Cycle";
    break;
  case 13:
    retval = "Scan";
    break;
  case 14:
    retval = "Dual Scan";
    break;
  case 15:
    retval = "Breath Fast";
    break;
  case 16:
    retval = "Theater Chase";
    break;
  case 17:
    retval = "Theater Chase Rainbow";
    break;
  case 18:
    retval = "Running Lights";
    break;
  case 19:
    retval = "Twinkle";
    break;
  case 20:
    retval = "Twinkle Random";
    break;
  case 21:
    retval = "Twinkle Fade";
    break;
  case 22:
    retval = "Twinkle Fade Random";
    break;
  case 23:
    retval = "Sparkle";
    break;
  case 26:
    retval = "Strobe";
    break;
  case 27:
    retval = "Strobe Rainbow";
    break;
  case 29:
    retval = "Blink Rainbow";
    break;
  case 32:
    retval = "Chase Random";
    break;
  case 33:
    retval = "Chase Rainbow";
    break;
  case 38:
    retval = "Chase Blackout Rainbow";
    break;
  case 40:
    retval = "Running Color";
    break;
  case 41:
    retval = "Running Red-Blue";
    break;
  case 42:
    retval = "Running Random";
    break;
  case 43:
    retval = "Larson Scanner";
    break;
  case 44:
    retval = "Comet";
    break;
  case 45:
    retval = "Fireworks";
    break;
  case 46:
    retval = "Fireworks Random";
    break;
  case 47:
    retval = "Merry Christmas";
    break;
  case 49:
    retval = "Fire Flicker";
    break;
  case 51:
    retval = "Circus Combustus";
    break;
  case 52:
    retval = "Halloween";
    break;
  case 55:
    retval = "TwinkleFox";
    break;
  case 56:
    retval = "Rain";
    break;

  default:
    retval = "nan";
    break;
  }
  return retval;
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
                if (request->getParam(PARAM_INPUT_1)->value().length() == 7)
                  sliderID = String(request->getParam(PARAM_INPUT_1)->value().charAt(6)); //extract Slider ID from String
                else if (request->getParam(PARAM_INPUT_1)->value().length() == 8)
                  sliderID = String(request->getParam(PARAM_INPUT_1)->value().charAt(6)) + String(request->getParam(PARAM_INPUT_1)->value().charAt(7)); //extract Slider ID from String

                debugln("\n[HTTP-Hanlder] Extracted Slider-ID '" + sliderID + "'!\n");

                if (String(request->getParam(PARAM_INPUT_1)->value()) == "uglw_speed")
                  sliderID = "5";
                else if (String(request->getParam(PARAM_INPUT_1)->value()) == "batvoltoffset")
                  sliderID = "7";
                else if (String(request->getParam(PARAM_INPUT_1)->value()) == "transcoef")
                  sliderID = "8";
                else if (String(request->getParam(PARAM_INPUT_1)->value()) == "batvoltoffsetmotor")
                  sliderID = "9";
                else if (String(request->getParam(PARAM_INPUT_1)->value()) == "favoriteUGLWMode")
                  sliderID = "19";
                sliderValue = request->getParam(PARAM_INPUT_3)->value();
                if (sliderID.toInt() == 6 || sliderID.toInt() == 7 || sliderID.toInt() == 8 || sliderID.toInt() == 9)
                  sliderValuesFloat[sliderID.toInt() - 6] = sliderValue.toFloat();
                else if (sliderID.toInt() == 19 && sliderValue.toInt() >= 0 && sliderValue.toInt() <= 56)
                {
                  favoriteMode = sliderValue.toInt();
                  favoriteColor1 = led_color1;
                  favoriteColor2 = led_color2;
                  favoriteColor3 = led_color3;
                  favoriteBrtns = led_brtns;
                  favoriteSpeed = led_speed;
                  favoriteFadeSize = fadeSize;
                }
                else if (sliderID == "11")
                  sliderValues[6] = sliderValue.toInt();
                else
                  sliderValues[sliderID.toInt()] = sliderValue.toInt();
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
                String sliderID;
                if (request->getParam(PARAM_INPUT_1)->value().length() == 7)
                  sliderID = String(request->getParam(PARAM_INPUT_1)->value().charAt(6)); //extract Slider ID from String
                else if (request->getParam(PARAM_INPUT_1)->value().length() == 8)
                  sliderID = String(request->getParam(PARAM_INPUT_1)->value().charAt(6)) + String(request->getParam(PARAM_INPUT_1)->value().charAt(7)); //extract Slider ID from String
                request->send_P(200, "text/plain", readWebSldr(sliderID.toInt()).c_str());
              } });

  server.on("/dropdown", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_3))
              {
                unsigned int dropdownID = request->getParam(PARAM_INPUT_1)->value().toInt();
                unsigned int buttonID = request->getParam(PARAM_INPUT_3)->value().toInt();
                if (dropdownID == 0 || dropdownID == 1 || dropdownID == 7 || dropdownID == 8)       //dropdown: 0 - underglow modes | 1 - color1 | 7 - color2 | 8 - color3
                  uglw_sendValue(dropdownID, buttonID, true); //starting at 0
              }
              request->send(200, "text/plain", "OK"); });

  server.on("/infoText1", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "";
              if (connected_clients[2])
                retval += "CC";
              else
                retval += "--";
              retval += " | ";
              if (connected_clients[0])
                retval += "SE";
              else
                retval += "--";
              retval += " | ";
              if (connected_clients[1])
                retval += "SG";
              else
                retval += "--";
              request->send_P(200, "text/plain", retval.c_str()); });

  server.on("/infoText3", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "Battery-Voltage - " + String(batteryVoltage) + " Volt";
              request->send_P(200, "text/plain", retval.c_str()); });

  server.on("/batVoltOffset", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "Calibration Offset - " + String(batVoltOffset) + " Volt";
              request->send_P(200, "text/plain", retval.c_str()); });

  server.on("/favoriteUGLWMode", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "Favorite Mode - " + String(favoriteMode);
              request->send_P(200, "text/plain", retval.c_str()); });

  server.on("/batVoltOffsetMotor", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "Motor Offset - " + String(motorVoltOffset) + " Volt";
              request->send_P(200, "text/plain", retval.c_str()); });

  server.on("/transCoef", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "Transition Coefficient - " + String(transitionCoefficient);
              request->send_P(200, "text/plain", retval.c_str()); });

  server.on("/uglwSelMode", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String retval = "Selected Mode - " + uglwSelectedModeRqst();
              request->send_P(200, "text/plain", retval.c_str()); });
}

String readStarState()
{
  String retval;
  if (apiOverrideOff)
    retval = "Star - BLOCKED";
  else if (batteryEmergency)
    retval = "Star - BAT LOW";
  else if (strobe || strobeShort)
    retval = "Star - Strobing";
  else if (ledcRead(0) > 0 && ledcRead(0) < 1023)
    retval = "Star - Fading";
  else if (ledcRead(0) == 0)
    retval = "Star - OFF";
  else if (ledcRead(0) >= 1023)
    retval = "Star - ON";
  else
    retval = "Star - nan";
  return retval;
}

String readTFLState()
{
  String retval;
  if (apiOverrideOff)
    retval = "TFL - BLOCKED";
  else if (batteryEmergency)
    retval = "TFL - BAT LOW";
  else if (strobe || strobeShort)
    retval = "TFL - Strobing";
  else if ((ledcRead(1) > 0 && ledcRead(1) < 1023 && motorRunning) || (ledcRead(1) > 0 && ledcRead(1) < tflDimTreshold && !motorRunning) || (ledcRead(1) > tflDimTreshold && !motorRunning))
    retval = "TFL - Fading";
  else if (ledcRead(1) == 0)
    retval = "TFL - OFF";
  else if ((ledcRead(1) >= 1023 && motorRunning) || (ledcRead(1) == tflDimTreshold && !motorRunning))
    retval = "TFL - ON";
  else
    retval = "TFL - nan";
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
  else if (id == 11)
  {
    return String(sliderValues[6]); // FadeSize
  }
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
  else if (id == 19) // *** FAVORITE MODE INPUTTEXT ***
  {
    debugln("[HTTP] Favorite UGLW Mode " + String(favoriteMode) + " was selected!");
    EEPROM.write(favoriteModeAdress, favoriteMode);
    debugln("[HTTP] Favorite UGLW Color 1 " + String(favoriteColor1) + " was selected!");
    writeColorEEPROM(favoriteColor1, true, 1U);
    debugln("[HTTP] Favorite UGLW Color 2 " + String(favoriteColor2) + " was selected!");
    writeColorEEPROM(favoriteColor2, true, 2U);
    debugln("[HTTP] Favorite UGLW Color 3 " + String(favoriteColor3) + " was selected!");
    writeColorEEPROM(favoriteColor3, true, 3U);
    debugln("[HTTP] Favorite UGLW Brightness " + String(favoriteBrtns) + " was selected!");
    EEPROM.write(favoriteBrtnsAdress, favoriteBrtns);
    debugln("[HTTP] Favorite UGLW Speed " + String(favoriteSpeed) + " was selected!");
    writeSpeedEEPROM(favoriteSpeed, true);
    debugln("[HTTP] Favorite UGLW FadeSize " + String(favoriteFadeSize) + " was selected!");
    EEPROM.write(favoriteFadeSizeAdress, favoriteFadeSize);
    EEPROM.commit();
  }
  else if (id == 11) // *** UGLW FADESIZE ***
  {
    if (sliderValues[6] >= 1 && sliderValues[6] <= 4)
    {
      uglw_sendValue(9, sliderValues[6], true);
      switch (sliderValues[6])
      {
      case 1:
        debugln("[HTTP] Underglow FadeSize 'Small' was selected!");
        break;
      case 2:
        debugln("[HTTP] Underglow FadeSize 'Medium' was selected!");
        break;
      case 3:
        debugln("[HTTP] Underglow FadeSize 'Large' was selected!");
        break;
      case 4:
        debugln("[HTTP] Underglow FadeSize 'XLarge' was selected!");
        break;
      }
    }
    else
      debugln("[HTTP] ERROR - UGLW FadeSize: Out of boundaries - Value: " + String(sliderValues[6]));
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
    }
    else
    {
      debugln("[HTTP] Fade-Mode OFF");
      fadeMode = false;
      fadeReset();
    }
    EEPROM.writeBool(fademodusAdress, fadeMode);
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
void uglw_sendValue(unsigned int dropdown, float key, bool overwrite = false) // 0 - mode | 1 - color | 2 - brtns | 3 - speed | 4 - Motor-Restriction | 5 - Data-Transmission | 6 - Transition Coefficient | 7 - Color2 | 8 - Color3 | 9 - FadeSize
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
        uglw_modesel2();
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
      retval = "Color 1 Red";
      payload = 16711680;
      break;
    case 1:
      retval = "Color 1 Green";
      payload = 65280;
      break;
    case 2:
      retval = "Color 1 Blue";
      payload = 255;
      break;
    case 3:
      retval = "Color 1 White";
      payload = 16777215;
      break;
    case 4:
      retval = "Color 1 Yellow";
      payload = 16776960;
      break;
    case 5:
      retval = "Color 1 Cyan";
      payload = 65535;
      break;
    case 6:
      retval = "Color 1 Magenta";
      payload = 16711935;
      break;
    case 7:
      retval = "Color 1 Purple";
      payload = 4194432;
      break;
    case 8:
      retval = "Color 1 Orange";
      payload = 16723968;
      break;
    case 9:
      retval = "Color 1 Pink";
      payload = 16716947;
      break;
    case 10:
      retval = "Color 1 Black";
      payload = 0;
      break;

    default:
      retval = "Color 1 " + String(keyI);
      payload = keyI;
      break;
    }
    if (overwrite)
    {
      debugln("\n[LED] Color 1 was changed!");
      led_color1 = payload;
      writeColorEEPROM(payload, false, 1U);
      if (selectedMode == 1 || selectedMode == 2)
        ledSerial.print("color1!" + String(payload) + "$");
    }
    else
      ledSerial.print("color1!" + String(payload) + "$");
  }
  else if (dropdown == 7 && key >= 0 && key <= 16777215) // Color
  {
    unsigned int keyI = (unsigned int)key;
    switch (keyI)
    {
    case 0:
      retval = "Color 2 Red";
      payload = 16711680;
      break;
    case 1:
      retval = "Color 2 Green";
      payload = 65280;
      break;
    case 2:
      retval = "Color 2 Blue";
      payload = 255;
      break;
    case 3:
      retval = "Color 2 White";
      payload = 16777215;
      break;
    case 4:
      retval = "Color 2 Yellow";
      payload = 16776960;
      break;
    case 5:
      retval = "Color 2 Cyan";
      payload = 65535;
      break;
    case 6:
      retval = "Color 2 Magenta";
      payload = 16711935;
      break;
    case 7:
      retval = "Color 2 Purple";
      payload = 4194432;
      break;
    case 8:
      retval = "Color 2 Orange";
      payload = 16723968;
      break;
    case 9:
      retval = "Color 2 Pink";
      payload = 16716947;
      break;
    case 10:
      retval = "Color 2 Black";
      payload = 0;

    default:
      if (keyI == 10)
      {
        retval = "Color 2 Black";
        payload = 0;
      }
      else
      {
        retval = "Color 2 " + String(keyI);
        payload = keyI;
      }
      break;
    }
    if (overwrite)
    {
      debugln("\n[LED] Color 2 was changed!");
      led_color2 = payload;
      writeColorEEPROM(payload, false, 2U);
      if (selectedMode == 1 || selectedMode == 2)
        ledSerial.print("color2!" + String(payload) + "$");
    }
    else
      ledSerial.print("color2!" + String(payload) + "$");
  }
  else if (dropdown == 8 && key >= 0 && key <= 16777215) // Color
  {
    unsigned int keyI = (unsigned int)key;
    switch (keyI)
    {
    case 0:
      retval = "Color 3 Red";
      payload = 16711680;
      break;
    case 1:
      retval = "Color 3 Green";
      payload = 65280;
      break;
    case 2:
      retval = "Color 3 Blue";
      payload = 255;
      break;
    case 3:
      retval = "Color 3 White";
      payload = 16777215;
      break;
    case 4:
      retval = "Color 3 Yellow";
      payload = 16776960;
      break;
    case 5:
      retval = "Color Cyan";
      payload = 65535;
      break;
    case 6:
      retval = "Color 3 Magenta";
      payload = 16711935;
      break;
    case 7:
      retval = "Color 3 Purple";
      payload = 4194432;
      break;
    case 8:
      retval = "Color 3 Orange";
      payload = 16723968;
      break;
    case 9:
      retval = "Color 3 Pink";
      payload = 16716947;
      break;
    case 10:
      retval = "Color 3 Black";
      payload = 0;

    default:
      if (keyI == 10)
      {
        retval = "Color 3 Black";
        payload = 0;
      }
      else
      {
        retval = "Color 3 " + String(keyI);
        payload = keyI;
      }
      break;
    }
    if (overwrite)
    {
      debugln("\n[LED] Color 3 was changed!");
      led_color3 = payload;
      writeColorEEPROM(payload, false, 3U);
      if (selectedMode == 1 || selectedMode == 2)
        ledSerial.print("color3!" + String(payload) + "$");
    }
    else
      ledSerial.print("color3!" + String(payload) + "$");
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
  else if (dropdown == 4 && key >= 0 && key <= 3) // MREST
  {
    unsigned int keyI = (unsigned int)key;
    ledSerial.print("motor!" + String(keyI) + "$");
    retval = "Motor-Restriction " + String(keyI);
  }
  else if (dropdown == 5) // Data-Transmission
  {
    unsigned int keyI = (unsigned int)key;
    ledSerial.print("trans!" + String(keyI) + "$");
    retval = "Data-Transmission " + String(keyI);
  }
  else if (dropdown == 6) // Trans-Coeff
  {
    ledSerial.print(String(transitionCoefficient_topic) + "!" + String(key) + "$");
    retval = "Data-Transmission " + String(key);
  }
  else if (dropdown == 9 && key >= 1 && key <= 4) // FadeSize
  {
    unsigned int keyI = (unsigned int)key;
    retval = "FadeSize " + String(keyI);
    if (overwrite)
    {
      debugln("\n[LED] FadeSize was changed!");
      fadeSize = sliderValues[6];
      EEPROM.write(fadeSizeAdress, fadeSize);
      EEPROM.commit();
      if (selectedMode == 1 || selectedMode == 2)
        ledSerial.print("fadesize!" + String(keyI) + "$");
    }
    else
      ledSerial.print("fadesize!" + String(keyI) + "$");
  }
  debugln("\n[HTTP] Underglow " + retval + " was selected!");
}

// Mode selection functions
void uglw_modesel1()
{
  selectedMode = 1;
  if (selectedModeBef == 4)
    strobeStop(false);
  else if (selectedModeBef == 3)
    tflBool = tflBoolBef;
}

void uglw_modesel2()
{
  selectedMode = 2;
  if (selectedModeBef == 4)
    strobeStop(false);
  else if (selectedModeBef == 3)
    tflBool = tflBoolBef;
  if (strobeActiveMQTT)
  {
    selectedModeBef = 4;
    strobeActiveMQTT = false;
  }
  if (selectedModeBef == 2 || selectedModeBef == 3 || selectedModeBef == 4)
    uglw_sendValue(5, 0); // init data transmission
  uglw_sendValue(3, led_speed);
  if (selectedModeBef != 3 || (selectedModeBef == 3 && tflBool))
    uglw_sendValue(2, led_brtns);
  else if (selectedModeBef == 3 && !tflBool)
    uglw_sendValue(2, 0);
  uglw_sendValue(1, led_color1);
  uglw_sendValue(7, led_color2);
  uglw_sendValue(8, led_color3);
  uglw_sendValue(9, fadeSize);
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
}

void uglw_modesel3()
{
  selectedMode = 3;
  tflBoolBef = tflBool;
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
  uglw_sendValue(1, favoriteColor1);
  uglw_sendValue(7, favoriteColor2);
  uglw_sendValue(8, favoriteColor3);
  uglw_sendValue(9, favoriteFadeSize);
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
}

void uglw_modesel4()
{
  selectedMode = 4;
  if (selectedModeBef == 2 || selectedModeBef == 3)
    uglw_sendValue(5, 0); // init data transmission
  if (selectedModeBef == 3)
    tflBool = tflBoolBef;
  uglw_sendValue(3, LED_SPEED_STROBE);
  uglw_sendValue(2, 255U);
  uglw_sendValue(1, 16777215);
  uglw_sendValue(7, 0);
  uglw_sendValue(8, 0);
  uglw_sendValue(0, LED_MODE_STROBE);
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
}

// CAN Handlers
void CAN_sendMessage(unsigned long txID, byte dlc, byte payload[])
{
  canSerial.print(txID, HEX);
  for (int i = 0; i < (int)dlc; i++)
  {
    canSerial.print(',');
    if (payload[i] < 16)
      canSerial.print("0");
    canSerial.print(payload[i], HEX);
  }
  canSerial.print('!');
}

void CAN_aliveMessage()
{
  if (((millis() > (myLastAveMsg + aveMsgIntervall)) || (myLastAveMsg == 0U)) && connected_clients[2])
  {
    myLastAveMsg = millis();
    byte payload[CAN_DLC] = {CAN_clientID, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0};
    CAN_sendMessage(CAN_txID, CAN_DLC, payload); // Sending ping rqst to clients
    canSerial.print("CC!");
  }

  // Timeout StarEmergency
  if (millis() > (emeg_lastAveMsg + aveMsgTimeout) && (connected_clients[0] || !con_clients_emegBefore))
  {
    con_clients_emegBefore = true;
    connected_clients[0] = false;
    EEPROM.write(apiOverrideOffAdress, 1);
    EEPROM.commit();
    setEmergencyMode();
    debugln("\n[Timeout-WD] StarClient Emergency timed out!");
  }

  // Timeout ShiftGuidance
  if (millis() > (shift_lastAveMsg + aveMsgTimeout) && connected_clients[1])
  {
    connected_clients[1] = false;
    debugln("\n[Timeout-WD] StarClient ShiftGuidance timed out!");
  }

  // Timeout CANChild
  if (millis() > (canChild_lastAveMsg + aveMsgTimeout) && connected_clients[2])
  {
    connected_clients[2] = false;
    debugln("\n[Timeout-WD] CANChild timed out!");
  }
}

uint8_t CAN_checkMessages()
{
  if (!canSerial.available())
    return 1;

  String rxMsg = canSerial.readStringUntil('!');

  // debugln("[UART] RX MSG: " + String(rxMsg));

  // Preparing serial string --> getting length and converting to char ary
  unsigned int msgLength = rxMsg.length();
  char rxMsgChar[msgLength];
  rxMsg.toCharArray(rxMsgChar, msgLength + 1, 0);

  // Checking for can child readiness
  if (string_find(rxMsgChar, CAN_childMsg_alive))
  {
    if (!connected_clients[2])
      debugln("[CAN] CANChild connected!");
    connected_clients[2] = true;
    canChild_lastAveMsg = millis();
    canSerial.print("PA!");
    return 0;
  }
  else if (string_find(rxMsgChar, CAN_childMsg_lives))
  {
    connected_clients[2] = true;
    canChild_lastAveMsg = millis();
    return 0;
  }

  // Extracting can msg id value
  char idChars[4] = {rxMsgChar[0], rxMsgChar[1], rxMsgChar[2], '\0'};
  unsigned long id = (unsigned long)strtol(idChars, 0, 16);

  if (id != 0x321) // Filter unwanted ids
  {
    debug("[CAN] Filtering ID: ");
    debugHEX(id);
    debugln();
    return 1;
  }

  // Converting two chars into one hex value
  byte payload[CAN_DLC];
  for (int i = 0; i < CAN_DLC; i++)
  {
    char hexChars[3] = {rxMsgChar[4 + i * 3], rxMsgChar[5 + i * 3], '\0'};
    payload[i] = (byte)strtol(hexChars, 0, 16);
  }

  // Debug output
  debug("\n[CAN] RX: ID: ");
  debugHEX(id);
  debug(" | DATA: ");
  for (int i = 0; i < CAN_DLC; i++)
  {
    if ((int)payload[i] < 16)
      debug("0");
    debugHEX(payload[i]);
    debug(" ");
  }

  // Message interpretation
  if (payload[2] == CAN_tpc_reset)
  {
    if (payload[0] == 0x1 && payload[1] == 0x1 && payload[3] == 0x1)
    {
      // ACK
      byte payloadACK[CAN_DLC] = {CAN_clientID, 0x2, 0x3, 0x1, 0x0, 0x0, 0x0, 0x0};
      CAN_sendMessage(CAN_txID, CAN_DLC, payloadACK);

      ledSerial.print(String(reset_topic) + "!1$");
      canSerial.print("CR!");
      delay(250);
      debugln("\n*****************************************");
      debugln("\n[RESET] Restarting at your wish master ;)");
      debugln("\n*****************************************");
      ESP.restart();
    }
    else
      debugln("[CAN] ERR Topic Reset: Unknown message!");
  }
  else if (payload[2] == CAN_tpc_apiovr)
  {
    if (payload[0] = 0x1 && payload[1] == 0x1)
    {
      if (payload[3] == 0x1)
      {
        // ACK
        byte payloadACK[CAN_DLC] = {CAN_clientID, 0x2, 0x2, 0x1, 0x0, 0x0, 0x0, 0x0};
        CAN_sendMessage(CAN_txID, CAN_DLC, payloadACK);

        selectedMode = 1;
        apiOverrideOff = true;
        EEPROM.write(apiOverrideOffAdress, 1);
        EEPROM.commit();
        setEmergencyMode();
        emeg_lastAveMsg = millis();
        connected_clients[0] = true;
      }
      else if (payload[3] == 0x0)
      {
        // ACK
        byte payloadACK[CAN_DLC] = {CAN_clientID, 0x2, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0};
        CAN_sendMessage(CAN_txID, CAN_DLC, payloadACK);

        selectedMode = 2;
        apiOverrideOff = false;
        EEPROM.write(apiOverrideOffAdress, 0);
        EEPROM.commit();
        resetEmergencyMode();
        emeg_lastAveMsg = millis();
        connected_clients[0] = true;
      }
      debugln("\n[CAN] Message: APIOVR set to " + String(payload[3]) + "");
    }
    else
      debugln("[CAN] ERR Topic APIOVR: Unknown message!");
  }
  else if (payload[2] == CAN_tpc_ping)
  {
    if (payload[1] == 0x0) // Request
    {
      debugln("\n[CAN] Message: Ping request");
      byte txPL[CAN_DLC] = {CAN_clientID, 0x1, CAN_tpc_ping, 0x1, 0x0, 0x0, 0x0, 0x0};
      CAN_sendMessage(CAN_txID, CAN_DLC, txPL);
    }
    else if (payload[1] == 0x1) // Answer
    {
      if (payload[0] == 0x1 && payload[1] == 0x1 && payload[3] == 0x1)
      {
        debugln("\n[CAN] Message: Ping from StarEmergency");
        if (!connected_clients[0])
        {
          con_clients_emegBefore = true;
          if (!apiOverrideOff)
            resetEmergencyMode();
          debugln("[Client] StarEmergency connected!");
        }
        emeg_lastAveMsg = millis();
        connected_clients[0] = true;
      }
      else if (payload[2] == 0x1 && payload[1] == 0x1 && payload[3] == 0x1)
      {
        debugln("\n[CAN] Message: Ping from StarShiftGuidance");
        if (!connected_clients[1])
          debugln("[Client] ShiftGuidance connected!");
        shift_lastAveMsg = millis();
        connected_clients[1] = true;
      }
      else
        debugln("[CAN] ERR Topic Ping: Unknown client!");
    }
    else
      debugln("[CAN] ERR Topic Ping: Unknown message!");
  }
  else if (payload[2] == CAN_tpc_selMode)
  {
    if (payload[0] == 0x1 && payload[1] == 0x1)
    {
      // ACK
      byte payloadACK[CAN_DLC] = {CAN_clientID, 0x2, 0x1, payload[3], 0x0, 0x0, 0x0, 0x0};
      CAN_sendMessage(CAN_txID, CAN_DLC, payloadACK);

      selectedMode = (unsigned int)payload[3];
      switch (selectedMode)
      {
      case 0x1:
        uglw_modesel1();
        debugln("\n[CAN] Message: FX Mode Blackout");
        break;

      case 0x2:
        uglw_modesel2();
        debugln("\n[CAN] Message: FX Mode HTML controlled");
        break;

      case 0x3:
        uglw_modesel3();
        debugln("\n[CAN] Message: FX Mode Favorite");
        break;

      case 0x4:
        uglw_modesel4();
        debugln("\n[CAN] Message: FX Mode Strobe");
        break;

      default:
        debugln("\n[CAN] ERR Topic Sel. Mode: Unknown mode " + String(selectedMode) + " !");
        break;
      }
      if (!httpStrobe)
        selectedModeBef = selectedMode;
    }
    else
      debugln("[CAN] ERR Topic Sel. Mode: Unknown message!");
  }
  else
  {
    debugln("\n[CAN] ERR Not a subscribed topic!");
  }

  return 0;
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
      if (!tflBoolBef)
        uglw_sendValue(2, led_brtns);
      uglw_sendValue(5, 5);
      uglw_sendValue(4, 0, true);
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
    static unsigned long timer_initConnect = 0;

    // debugln("[UGLW] Message arrived - Topic: '" + topic + "' - Payload: '" + payload + "'\n");

    if (String(topic) == "status")
    {
      if (String(payload) == "cnt-wtg" && ((millis() > timer_initConnect + 3000) || !serialClientInitConnection))
      {
        timer_initConnect = millis();
        ledSerial.print("status!host-wasborn$");
        if (emergency)
          lastSerialMsg = String(apiOvrOff_topic) + "!1$";
        else
          lastSerialMsg = String(apiOvrOff_topic) + "!0$";
        serialClientInitConnection = true;
        debugln("\n[UGLW] Initializing client communication!");
        uglwStarted = true;
        ledSerial.print(lastSerialMsg);
        uglwTFLRestrictionHandler();
        uglw_sendValue(6, transitionCoefficient);
        if (uglwMotorBlock || uglwTFLRestActive)
          uglw_sendValue(4, 1U, true);
        else if (!uglwMotorBlock && !uglwTFLRestActive)
          uglw_sendValue(4, 0U, true);
      }
      else if (String(payload) == "cnctd")
      {
        serialClientConnection = true;
        debugln("\n[UGLW] Client successfully connected!");
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

bool string_find(char *haystack, char *needle)
{
  int compareOffset = 0;
  while (*haystack)
  {
    if (*haystack == *needle)
    {
      compareOffset = 0;
      while (haystack[compareOffset] == needle[compareOffset])
      {
        compareOffset++;
        if (needle[compareOffset] == '\0')
        {
          return true;
        }
      }
    }
    haystack++;
  }
  return false;
}