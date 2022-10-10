// ***---*** Project - Star Client - Underglow ***---***

// ***---*** Versions ***---***
/*--- Changelog ---
  27/11/21 - V1.0
  ~ qC to control ws2812-ledstripes
  ~ Added mqtt client connectivity to listen to messages from host broker and therefore from f.e. emergency-node
  - MQTT callback - apioverrideoff to turn off stripes if message was sent from emergency-node or host
  ~ Added EEPROM system to store variable states also on powerloss
  07/12/21 - V1.1
  ~ Fixed EEPROM bug for color and speed saving
  - EEPROM can store a maximum of 8 bits per "adress", therefore values greater than 255 have to be split in low (, mid) and high bytes
  ~ Added automatic switching to "EMERGENCY MODE", if the wifi / mqtt connection is lost or there is an api call to override the lights
  18/02/22 - V2.0
  ~ Updated whole system (starhost + emergency + underglow) to new generation 2.0
  - New features include the full usability of WS2812FX library (lots of effects, configurable via fadesize), clean transitions between fxmode 2 and 3 by using virtual stripes, ...
  - Transitions can be variated using different durations (code) or by modifying the transition coefficient (web interface)

  --- Bugs ---
  ~ Very rare: After fxmode strobe and a MCU reset strobe stays until another differnt fxmode is selected
  - Origin: not known
*/

//***** INCLUDES *****
#include "Arduino.h"
#include <EEPROM.h>
#include <WS2812FX.h>

//***** DEFINES *****
// Version
#define VERSION 2.0

// Debug
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)

// IOs
#define LED_PIN 5   // D1
#define RELAY_PIN 4 // D2

// EEPROM
#define apiOverrideOffAdress 0
#define modeAdress 1
#define color1_1Adress 2 // 0x0000FF - Blue
#define color1_2Adress 3 // 0x00FF00 - Green
#define color1_3Adress 4 // 0xFF0000 - Red
#define brtnsAdress 5
#define speed1Adress 6 // 0x00FF
#define speed2Adress 7 // 0xFF00 - 0 to 65535
#define motorBlockAdress 8
#define color2_1Adress 9  // 0x0000FF - Blue
#define color2_2Adress 10 // 0x00FF00 - Green
#define color2_3Adress 11 // 0xFF0000 - Red
#define color3_1Adress 12 // 0x0000FF - Blue
#define color3_2Adress 13 // 0x00FF00 - Green
#define color3_3Adress 14 // 0xFF0000 - Red
#define fadeSizeAdress 15 // 0xFF0000 - Red
// continue at 16

// WS2812 LEDs
#define LED_COUNT 595

// Serial Client Topics
#define status_topic "status"
#define apiOvrOff_topic "api/ovroff"
#define mode_topic "mode"
#define color1_topic "color1"
#define color2_topic "color2"
#define color3_topic "color3"
#define brtns_topic "brtns"
#define speed_topic "speed"
#define motor_topic "motor"
#define transmission_topic "trans"
#define transitionCoefficient_topic "transCoef"
#define reset_topic "reset"
#define fadeSize_topic "fadesize"

//***** VARIABLES & OBJECTS *****
bool apiOverrideOff;
bool emergency;

bool serialInitDataReceived = false;
bool serialConnected = false;
bool hostInitConnection = false;

unsigned long lastAliveMsg = 0;
const unsigned int aliveMsgTimeout = 7500;
const String aliveMsg = "host-alive";

int ledBlinkCode = 0;
int ledBlinkCounter = 0;
bool ledBlinkInProgress = false;
unsigned long ledBlinkLastBlink = 0;

// WS2812FX & LED Parameters
WS2812FX leds = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned int led_mode;
unsigned int led_color1;
unsigned int led_color2;
unsigned int led_color3;
uint32_t colors[3] = {BLACK, BLACK, BLACK};
unsigned int led_brtns;
unsigned int led_speed;
uint8_t led_fadeSize;
uint8_t fadeSize;

unsigned int led_mode_bef = -1;
unsigned int led_color1_bef = -1;
unsigned int led_color2_bef = -1;
unsigned int led_color3_bef = -1;
unsigned int led_brtns_bef = -1;
unsigned int led_speed_bef = -1;
uint8_t fadeSize_bef = -1;

// Transition
bool transitionActive = false;
bool transDown = false;
bool transUp = false;
bool activeStrip = true; // true = 1 | false = 2
float transCoef = 1.18;

struct
{
  unsigned int led_mode;
  uint32_t colors[3] = {BLACK, BLACK, BLACK};
  unsigned int led_brtns;
  unsigned int led_speed;
  uint8_t led_fadeSize;
} v1Strip;

struct
{
  unsigned int led_mode;
  uint32_t colors[3] = {BLACK, BLACK, BLACK};
  unsigned int led_brtns;
  unsigned int led_speed;
  uint8_t led_fadeSize;
} v2Strip;

const unsigned int transitionTimeMotorBlock = 2000; // 1s to switch off uglw on starting motor
unsigned int transitionTimeMode = 1000;
unsigned int transitionTimeDelay = 0;
unsigned long transitionLastStep = 0;

// Motor Blockage
bool uglwMotorBlock = false;
bool motorBlockTransition = false;
unsigned int motorBlockageBrtnsBefore;

bool transDataReceived = true;

//***** PROTOTYPES *****
void setup();
void loop();
void handlers();
void saveStripParams(bool);
unsigned int brtnsMathFct(unsigned int);
void transitionLED(unsigned int);
void applySettingsLED();
void ledBlink();
void ledBlinkError();
void ledBlinkSuccess();
void eeprom_initLastState();
void writeColorEEPROM(unsigned int, unsigned int);
unsigned int readColorEEPROM(unsigned int);
void writeSpeedEEPROM(unsigned int);
unsigned int readSpeedEEPROM();
void setEmergencyMode();
void resetEmergencyMode();
bool serialCallback();
void reconnect();
bool checkSerial();
void initSerial();
void serialEmergencyTOWD();

//***** SETUP *****
void setup()
{
  // Begin
  Serial.begin(115200);
  Serial.setTimeout(3);
  EEPROM.begin(16);

  debugln("\n[StarClient-Underglow] Starting programm ~ by spl01t*#7");
  debugln("[StarClient-Underglow] You are running version " + String(VERSION) + "!");

  // IOs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Activate "power" to ws2812 strips

  // Get EEPROM memory
  eeprom_initLastState();

  // LEDs - 2 virtual strips to transition between
  leds.init();
  leds.setSegment(0, 0, LED_COUNT - 1, 0U, 0U, 0U, (uint8_t)0);
  leds.setBrightness(0);
  leds.start();

  // Communication Config
  checkSerial();

  debugln("\n[StarControl-Client] Initialization completed...starting programm loop!\n");
}

//***** LOOP *****
void loop()
{
  handlers();

  transitionLED(0);

  applySettingsLED();

  ledBlink();

  serialCallback();
}

//***** FUNCTIONS *****
// Handler Functions
void handlers()
{
  leds.service();
  serialEmergencyTOWD();
  yield();
}

// WS2812 Handlers
void saveStripParams(bool strip)
{
  // Inverted logic, because active strip is changed not until brtns was 0
  if (strip) // V2
  {
    v2Strip.led_mode = led_mode;
    v2Strip.colors[0] = colors[0];
    v2Strip.colors[1] = colors[1];
    v2Strip.colors[2] = colors[2];
    v2Strip.led_brtns = led_brtns;
    v2Strip.led_speed = led_speed;
    v2Strip.led_fadeSize = led_fadeSize;
    debugln("\n[TRANS] Prepared Virtual 2 - Mode: " + String(v2Strip.led_mode) + " - Color 1: " + String(v2Strip.colors[0]) + " - Color 2: " + String(v2Strip.colors[1]) + " - Color 3: " + String(v2Strip.colors[2]) + " - Brtns: " + String(v2Strip.led_brtns) + " - Speed: " + String(v2Strip.led_speed) + " - FadeSize: " + String(v2Strip.led_fadeSize) + "\n");
  }
  else // V1
  {
    v1Strip.led_mode = led_mode;
    v1Strip.colors[0] = colors[0];
    v1Strip.colors[1] = colors[1];
    v1Strip.colors[2] = colors[2];
    v1Strip.led_brtns = led_brtns;
    v1Strip.led_speed = led_speed;
    v1Strip.led_fadeSize = led_fadeSize;
    debugln("\n[TRANS] Prepared Virtual 1 - Mode: " + String(v1Strip.led_mode) + " - Color 1: " + String(v1Strip.colors[0]) + " - Color 2: " + String(v1Strip.colors[1]) + " - Color 3: " + String(v1Strip.colors[2]) + " - Brtns: " + String(v1Strip.led_brtns) + " - Speed: " + String(v1Strip.led_speed) + " - FadeSize: " + String(v1Strip.led_fadeSize) + "\n");
  }
}

unsigned int brtnsMathFct(unsigned int curBrtns)
{
  float curBrtnsF = (float)curBrtns / 100;
  return (unsigned int)exp(transCoef * curBrtnsF);
}

void transitionLED(unsigned int transitionType) // transType - 0 = tranist | 1 = mode change | 2 = motor start | 3 = motor stop
{
  if (transitionType == 0 && transitionActive && (millis() > transitionLastStep + transitionTimeDelay) && !emergency)
  {
    transitionLastStep = millis();
    if (transDown)
    {
      unsigned int curBrtns = leds.getBrightness();
      if (curBrtns > 0)
      {
        int newBrtns = curBrtns - brtnsMathFct(curBrtns);
        if (newBrtns < 0)
          newBrtns = 0;
        leds.setBrightness(newBrtns);
        leds.service();
      }
      else
      {
        transDown = false;
        transUp = true;
        activeStrip = !activeStrip;
        if (activeStrip)
        {
          leds.setSegment(0, 0, LED_COUNT - 1, v1Strip.led_mode, v1Strip.colors, v1Strip.led_speed, v1Strip.led_fadeSize);
          debugln("\n[TRANS] Reached half transition! - Loading settings V1\n");
        }
        else
        {
          leds.setSegment(0, 0, LED_COUNT - 1, v2Strip.led_mode, v2Strip.colors, v2Strip.led_speed, v2Strip.led_fadeSize);
          debugln("\n[TRANS] Reached half transition! - Loading settings V2\n");
        }
        ledBlinkCode = 1;
      }
    }
    else if (transUp)
    {
      unsigned int curBrtns = leds.getBrightness();
      if (curBrtns < led_brtns)
      {
        unsigned int newBrtns = curBrtns + brtnsMathFct(curBrtns);
        if (newBrtns > led_brtns)
          newBrtns = led_brtns;
        leds.setBrightness(newBrtns);
        leds.service();
      }
      else
      {
        transUp = false;
        transitionActive = false;
        debugln("\n[TRANS] Finished transition!\n");
      }
    }
  }
  else if (transitionType == 1)
  {
    debugln("\n[TRANS] Started transition!");
  }
  else if (transitionType == 2)
  {
    debugln("\n[TRANS] Started transition (down) because of motor blockage!");
  }
  else if (transitionType == 3)
  {
    debugln("\n[TRANS] Started transition (up) because of motor blockage release!");
  }
  if (transitionType == 1 || transitionType == 2 || transitionType == 3)
  {
    saveStripParams(activeStrip);

    if (transitionTimeMode == 0)
    {
      // Static transition
      activeStrip = !activeStrip;
      if (activeStrip)
      {
        leds.setSegment(0, 0, LED_COUNT - 1, v1Strip.led_mode, v1Strip.colors, v1Strip.led_speed, v1Strip.led_fadeSize);
        leds.setBrightness(v1Strip.led_brtns);
      }
      else
      {
        leds.setSegment(0, 0, LED_COUNT - 1, v2Strip.led_mode, v2Strip.colors, v2Strip.led_speed, v2Strip.led_fadeSize);
        leds.setBrightness(v2Strip.led_brtns);
      }
      debugln("\n[TRANS] Finished static transition!\n");
    }
    else
    {
      // Dynamic transition
      transitionActive = true;
      transDown = true;
      ledBlinkCode = 2;

      // Calculate transition step delay and duration
      unsigned int curBrtns = leds.getBrightness();
      unsigned int targetBrtns = activeStrip ? v2Strip.led_brtns : v1Strip.led_brtns;

      unsigned int steps = curBrtns + targetBrtns;

      transitionTimeDelay = transitionTimeMode / steps;
      debugln("\n[TRANS] Delay between steps: " + String(transitionTimeDelay) + "ms");
    }
  }
}

void applySettingsLED()
{
  if (!uglwMotorBlock && !motorBlockTransition && !emergency && transDataReceived)
  {
    bool transitionChanges = false;

    if (led_mode != led_mode_bef)
    {
      transitionChanges = true;
      led_mode_bef = led_mode;
      debugln("\n[LED] MODE was changed to " + String(led_mode) + "!");
    }

    if (led_color1 != led_color1_bef)
    {
      led_color1_bef = led_color1;
      colors[0] = led_color1;
      if (!transitionChanges)
      {
        leds.setColors(0, colors);
      }
      debugln("\n[LED] COLOR 1 was changed to " + String(led_color1) + "!");
    }

    if (led_color2 != led_color2_bef)
    {
      led_color2_bef = led_color2;
      colors[1] = led_color2;
      if (!transitionChanges)
      {
        leds.setColors(0, colors);
      }
      debugln("\n[LED] COLOR 2 was changed to " + String(led_color2) + "!");
    }

    if (led_color3 != led_color3_bef)
    {
      led_color3_bef = led_color3;
      colors[2] = led_color3;
      if (!transitionChanges)
      {
        leds.setColors(0, colors);
      }
      debugln("\n[LED] COLOR 3 was changed to " + String(led_color3) + "!");
    }

    if (led_brtns != led_brtns_bef)
    {
      led_brtns_bef = led_brtns;
      if (!transitionChanges)
      {
        leds.setBrightness(led_brtns);
      }
      debugln("\n[LED] BRIGHTNESS was changed to " + String(led_brtns) + "!");
    }

    if (led_speed != led_speed_bef)
    {
      led_speed_bef = led_speed;
      if (!transitionChanges)
      {
        leds.setSpeed(led_speed);
      }
      debugln("\n[LED] SPEED was changed to " + String(led_speed) + "!");
    }

    if (fadeSize != fadeSize_bef)
    {
      fadeSize_bef = fadeSize;
      switch (fadeSize)
      {
      case 1:
        led_fadeSize = SIZE_SMALL + FADE_XFAST;
        break;
      case 2:
        led_fadeSize = SIZE_MEDIUM + FADE_MEDIUM;
        break;
      case 3:
        led_fadeSize = SIZE_LARGE + FADE_XSLOW;
        break;
      case 4:
        led_fadeSize = SIZE_XLARGE + FADE_GLACIAL;
        break;
      }
      if (!transitionChanges)
      {
        leds.setOptions(0, led_fadeSize);
      }
      debugln("\n[LED] FADESIZE was changed to " + String(led_fadeSize) + "!");
    }

    if (transitionChanges)
    {
      transitionLED(1);
    }
  }
  else if (uglwMotorBlock && motorBlockTransition)
  {
    // Motor-Blockage was received --> Transition to uglw off
    motorBlockTransition = false;
    transitionLED(2);
  }
  else if (!uglwMotorBlock && motorBlockTransition)
  {
    // Motor-Blockage was released --> Transition to uglw on
    motorBlockTransition = false;
    transitionLED(3);
  }
}

// BUILTINLED Blinks
void ledBlink()
{
  if (ledBlinkCode != 0)
  {
    switch (ledBlinkCode)
    {
    case 1:
      ledBlinkError();
      break;

    case 2:
      ledBlinkSuccess();
      break;

    default:
      break;
    }
  }
}

void ledBlinkError()
{
  if (!ledBlinkInProgress)
  {
    ledBlinkInProgress = true;
    ledBlinkCounter = 0;
  }

  if ((ledBlinkLastBlink + 50) < millis())
  {
    ledBlinkLastBlink = millis();

    if (digitalRead(LED_BUILTIN))
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
    else
    {
      digitalWrite(LED_BUILTIN, HIGH);
    }

    ledBlinkCounter++;
  }

  if (ledBlinkCounter >= 20)
  {
    ledBlinkCode = 0;
    ledBlinkInProgress = false;
    digitalWrite(LED_BUILTIN, LOW); // Default on
  }
}

void ledBlinkSuccess()
{
  if (!ledBlinkInProgress)
  {
    ledBlinkInProgress = true;
    ledBlinkCounter = 0;
  }

  if ((ledBlinkLastBlink + 100) < millis())
  {
    ledBlinkLastBlink = millis();

    if (digitalRead(LED_BUILTIN))
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
    else
    {
      digitalWrite(LED_BUILTIN, HIGH);
    }

    ledBlinkCounter++;
  }

  if (ledBlinkCounter >= 10)
  {
    ledBlinkCode = 0;
    ledBlinkInProgress = false;
    digitalWrite(LED_BUILTIN, LOW); // Default on
  }
}

// EEPROM
void eeprom_initLastState()
{
  debugln("\n[EEPROM] Starting data extratction!");

  // API Override Lights Off
  int apiOverrideOffcontent = int(EEPROM.read(apiOverrideOffAdress));

  debug("[EEPROM] API Override Lights Off: " + String(apiOverrideOffcontent));

  if (apiOverrideOffcontent == 1)
  {
    apiOverrideOff = true;
    emergency = true;
    debugln(" - [EMERGENCY] Mode activated!");
  }
  else if (apiOverrideOffcontent == 0)
  {
    apiOverrideOff = false;
    emergency = false;
    debugln(" - [EMERGENCY] Mode deactivated!");
  }
  else
  {
    debug("\n[EEPROM] Reading was no valid option: API Override Lights Off");
    debugln(" - [EMERGENCY] Mode activated!");
    apiOverrideOff = true;
    emergency = true;
    leds.setBrightness(0);
    EEPROM.write(apiOverrideOffAdress, 1); // then write override to turn lights off
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
  unsigned int colorContent1 = readColorEEPROM(1);

  debugln("[EEPROM] LEDs - Color 1: " + String(colorContent1));

  if (colorContent1 >= 0 && colorContent1 <= 16777215)
  {
    led_color1 = colorContent1;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Color 1 - Out of range (0x0-0xFFFFFF)!");
    led_color1 = 0;
    writeColorEEPROM(0, 1);
  }

  colors[0] = led_color1;

  // LEDs Color 2
  unsigned int colorContent2 = readColorEEPROM(2);

  debugln("[EEPROM] LEDs - Color 2: " + String(colorContent2));

  if (colorContent2 >= 0 && colorContent2 <= 16777215)
  {
    led_color2 = colorContent2;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Color 2 - Out of range (0x0-0xFFFFFF)!");
    led_color2 = 0;
    writeColorEEPROM(0, 2);
  }

  colors[1] = led_color2;

  // LEDs Color 3
  unsigned int colorContent3 = readColorEEPROM(3);

  debugln("[EEPROM] LEDs - Color 3: " + String(colorContent3));

  if (colorContent3 >= 0 && colorContent3 <= 16777215)
  {
    led_color3 = colorContent3;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Color 3 - Out of range (0x0-0xFFFFFF)!");
    led_color3 = 0;
    writeColorEEPROM(0, 3);
  }

  colors[2] = led_color3;

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

  // LEDs Speed
  unsigned int speedContent = readSpeedEEPROM();

  debugln("[EEPROM] LEDs - Speed: " + String(speedContent));

  if (speedContent >= 0 && speedContent <= 65535)
  {
    led_speed = speedContent;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - Speed - Out of range (0x0-0xFFFF)!");
    led_speed = 10000;
    writeSpeedEEPROM(led_speed);
  }

  // Motor Block
  unsigned int mbContent = int(EEPROM.read(motorBlockAdress));

  debugln("[EEPROM] Motor Blockage: " + String(mbContent));

  if (mbContent == 1)
  {
    uglwMotorBlock = true;
  }
  else if (mbContent == 0)
  {
    uglwMotorBlock = false;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: Motor Blockage - Setting to true!");
    uglwMotorBlock = true;
  }

  // LEDs FadeSize
  uint8_t fadeSizeContent = uint8_t(EEPROM.read(fadeSizeAdress));

  debugln("[EEPROM] LEDs - FadeSize: " + String(fadeSizeContent));

  if (fadeSizeContent >= 1 && fadeSizeContent <= 4)
  {
    fadeSize = fadeSizeContent;
  }
  else
  {
    debugln("[EEPROM] Reading was no valid option: LEDs - FadeSize - Out of range (1-4)!");
    fadeSize = 1;
    EEPROM.write(fadeSizeAdress, fadeSize);
  }

  EEPROM.commit();

  debugln("[EEPROM] Extraction completed!");
}

void writeColorEEPROM(unsigned int key, unsigned int colorType)
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
    EEPROM.write(color1_1Adress, lowbit);  // Blue Bit
    EEPROM.write(color1_2Adress, midbit);  // Green Bit
    EEPROM.write(color1_3Adress, highbit); // Red Bit
  }
  else if (colorType == 2)
  {
    EEPROM.write(color2_1Adress, lowbit);  // Blue Bit
    EEPROM.write(color2_2Adress, midbit);  // Green Bit
    EEPROM.write(color2_3Adress, highbit); // Red Bit
  }
  else if (colorType == 3)
  {
    EEPROM.write(color3_1Adress, lowbit);  // Blue Bit
    EEPROM.write(color3_2Adress, midbit);  // Green Bit
    EEPROM.write(color3_3Adress, highbit); // Red Bit
  }
  EEPROM.commit();
}

unsigned int readColorEEPROM(unsigned int colorType)
{
  int key;

  if (colorType == 1)
  {
    key = EEPROM.read(color1_3Adress) << 8;         // Red Bit
    key = (key + EEPROM.read(color1_2Adress)) << 8; // Green Bit
    key += EEPROM.read(color1_1Adress);             // Blue Bit
  }
  else if (colorType == 2)
  {
    key = EEPROM.read(color2_3Adress) << 8;         // Red Bit
    key = (key + EEPROM.read(color2_2Adress)) << 8; // Green Bit
    key += EEPROM.read(color2_1Adress);             // Blue Bit
  }
  else if (colorType == 3)
  {
    key = EEPROM.read(color3_3Adress) << 8;         // Red Bit
    key = (key + EEPROM.read(color3_2Adress)) << 8; // Green Bit
    key += EEPROM.read(color3_1Adress);             // Blue Bit
  }

  /*debugln("\nKEY DEC: " + String(key));
    debugln("KEY HEX: " + String(key, HEX));
    debugln("KEY BIN: " + String(key, BIN));*/

  return (unsigned int)key;
}

void writeSpeedEEPROM(unsigned int key)
{
  int lowbit = key & 255;
  int highbit = (key >> 8) & 255;

  /*debugln("\nKEY DEC: " + String(key));
    debugln("KEY HEX: " + String(key, HEX));
    debugln("KEY BIN: " + String(key, BIN));
    debugln("LOWBIT: " + String(lowbit, BIN));
    debugln("HIGHBIT: " + String(highbit, BIN));*/

  EEPROM.write(speed1Adress, lowbit);
  EEPROM.write(speed2Adress, highbit);
  EEPROM.commit();
}

unsigned int readSpeedEEPROM()
{
  int key = EEPROM.read(speed2Adress) << 8;
  key += EEPROM.read(speed1Adress);

  /*debugln("\nKEY DEC: " + String(key));
    debugln("KEY HEX: " + String(key, HEX));
    debugln("KEY BIN: " + String(key, BIN));*/

  return key;
}

// Emergency Handlers
void setEmergencyMode()
{
  emergency = true;
  leds.setBrightness(0);
  leds.service();

  // Stop ongoing transition
  transDown = false;
  transUp = false;
  transitionActive = false;
  debugln("\n[TRANS] Stopped transition!\n");

  debugln("\n[EMERGENCY] Mode activated!\n");
}

void resetEmergencyMode()
{
  emergency = false;
  if (!uglwMotorBlock && serialInitDataReceived)
  {
    leds.setSegment(0, 0, LED_COUNT - 1, led_mode, colors, led_speed, led_fadeSize);
    leds.setBrightness(led_brtns);
    if (activeStrip)
      debugln("\n --- Virtual 1 EMEG - Settings - Mode: " + String(leds.getMode()) + " - Color 1: " + String(colors[0]) + " - Color 2: " + String(colors[1]) + " - Color 3: " + String(colors[2]) + " - Brtns: " + String(leds.getBrightness()) + " - Speed: " + String(leds.getSpeed()) + " ---\n");
    else
      debugln("\n --- Virtual 2 EMEG - Settings - Mode: " + String(leds.getMode()) + " - Color 1: " + String(colors[0]) + " - Color 2: " + String(colors[1]) + " - Color 3: " + String(colors[2]) + " - Brtns: " + String(leds.getBrightness()) + " - Speed: " + String(leds.getSpeed()) + " ---\n");
    led_mode_bef = led_mode;
    led_color1_bef = led_color1;
    led_color2_bef = led_color2;
    led_color3_bef = led_color3;
    led_brtns_bef = led_brtns;
    led_speed_bef = led_speed;
    fadeSize_bef = fadeSize;
  }
  debugln("\n[EMERGENCY] Mode deactivated!\n");
}

// Serial Communication
bool serialCallback()
{
  String topic = "";
  String payload = "";

  if (Serial.available())
  {
    topic = Serial.readStringUntil('!');
    payload = Serial.readStringUntil('$');
    if (0)
      return false;
    else
      debug("[Serial] Message arrived - Topic: '" + topic + "' - Payload: '" + payload + "'\n");
  }
  else
    return false;

  // Check whether Message is subscribed
  if (topic == status_topic)
  {
    debugln("[Serial] Subscribed topic - Status: " + String(payload));
    if (payload == aliveMsg)
      lastAliveMsg = millis();
    else if (payload == "host-wasborn")
      initSerial();
  }
  else if (topic == reset_topic)
  {
    debugln("\n*****************************************");
    debugln("\n[RESET] Restarting at your wish master ;)");
    debugln("\n*****************************************");
    ESP.restart();
  }
  else if (String(topic) == apiOvrOff_topic) // API Override Off Handler
  {
    debugln("[Serial] Subscribed topic - API Override Light Off: " + String(payload));
    if (payload == "1")
    {
      if (!emergency)
      {
        apiOverrideOff = true;
        setEmergencyMode();
        EEPROM.write(apiOverrideOffAdress, 1);
        EEPROM.commit();
      }
    }
    else if (payload == "0")
    {
      if (emergency)
      {
        apiOverrideOff = false;
        resetEmergencyMode();
        EEPROM.write(apiOverrideOffAdress, 0);
        EEPROM.commit();
      }
    }
  }
  else if (String(topic) == mode_topic) // LED Mode Handler
  {
    debugln("[Serial] Subscribed topic - Underglow Mode: " + String(payload));
    unsigned int key = payload.toInt();
    if (key >= 0 && key <= 56 && key != led_mode)
    {
      led_mode = key;
      EEPROM.write(modeAdress, led_mode);
      EEPROM.commit();
      debugln("\n[LED] MODE was saved to " + String(key) + "!\n");
    }
    else if (key == led_mode)
      ;
    else
      debugln("\n[LED] " + String(key) + " is out of range for parameter MODE!");
  }
  else if (String(topic) == color1_topic) // LED Color Handler
  {
    debugln("[Serial] Subscribed topic - Underglow Color 1: " + String(payload));
    unsigned int key = payload.toInt();
    if (key >= 0 && key <= 16777215 && key != led_color1)
    {
      led_color1 = key;
      colors[0] = led_color1;
      writeColorEEPROM(led_color1, 1);
      debugln("\n[LED] COLOR 1 was saved to " + String(key) + "!");
    }
    else if (key == led_color1)
      ;
    else
      debugln("\n[LED] " + String(key) + " is out of range for parameter COLOR 1!");
  }
  else if (String(topic) == color2_topic) // LED Color Handler
  {
    debugln("[Serial] Subscribed topic - Underglow Color 2: " + String(payload));
    unsigned int key = payload.toInt();
    if (key >= 0 && key <= 16777215 && key != led_color2)
    {
      led_color2 = key;
      colors[1] = led_color2;
      writeColorEEPROM(led_color2, 2);
      debugln("\n[LED] COLOR 2 was saved to " + String(key) + "!");
    }
    else if (key == led_color2)
      ;
    else
      debugln("\n[LED] " + String(key) + " is out of range for parameter COLOR 2!");
  }
  else if (String(topic) == color3_topic) // LED Color Handler
  {
    debugln("[Serial] Subscribed topic - Underglow Color 3: " + String(payload));
    unsigned int key = payload.toInt();
    if (key >= 0 && key <= 16777215 && key != led_color3)
    {
      led_color3 = key;
      colors[2] = led_color3;
      writeColorEEPROM(led_color3, 3);
      debugln("\n[LED] COLOR 3 was saved to " + String(key) + "!");
    }
    else if (key == led_color3)
      ;
    else
      debugln("\n[LED] " + String(key) + " is out of range for parameter COLOR 3!");
  }
  else if (String(topic) == brtns_topic) // LED Brightness Handler
  {
    debugln("[Serial] Subscribed topic - Underglow Brightness: " + String(payload));
    unsigned int key = payload.toInt();
    if (key >= 0 && key <= 255 && key != led_brtns)
    {
      led_brtns = key;
      EEPROM.write(brtnsAdress, led_brtns);
      EEPROM.commit();
      debugln("\n[LED] BRIGHTNESS was saved to " + String(key) + "!");
    }
    else if (key == led_brtns)
      ;
    else
      debugln("\n[LED] " + String(key) + " is out of range for parameter BRIGHTNESS!");
  }
  else if (String(topic) == speed_topic) // LED Speed Handler
  {
    debugln("[Serial] Subscribed topic - Underglow Speed: " + String(payload));
    unsigned int key = payload.toInt();
    if (key >= 0 && key <= 65535 && key != led_speed)
    {
      led_speed = key;
      writeSpeedEEPROM(led_speed);
      debugln("\n[LED] SPEED was saved to " + String(key) + "!");
    }
    else if (key == led_speed)
      ;
    else
      debugln("\n[LED] " + String(key) + " is out of range for parameter SPEED!");
  }
  else if (String(topic) == motor_topic)
  {
    debugln("[Serial] Subscribed topic - Underglow Motor-Blockage: " + String(payload));
    if (payload.toInt() == 1)
    {
      if (!uglwMotorBlock)
      {
        uglwMotorBlock = true;
        if (!emergency && serialInitDataReceived)
          motorBlockTransition = true;
        EEPROM.write(motorBlockAdress, 1);
        motorBlockageBrtnsBefore = led_brtns;
        led_brtns = 0;
      }
    }
    else if (payload.toInt() == 0)
    {
      if (uglwMotorBlock)
      {
        uglwMotorBlock = false;
        if (!emergency)
          motorBlockTransition = true;
        EEPROM.write(motorBlockAdress, 0);
        if (led_brtns == 0)
          led_brtns = motorBlockageBrtnsBefore;
      }
    }
    else if (payload.toInt() == 2)
    {
      uglwMotorBlock = false;
      EEPROM.write(motorBlockAdress, 0);
    }
    else if (payload.toInt() == 3)
    {
      uglwMotorBlock = true;
      EEPROM.write(motorBlockAdress, 1);
    }
    EEPROM.commit();
    if (!serialInitDataReceived)
    {
      serialInitDataReceived = true;
      debugln("[Serial] Received init Data!\n");
    }
  }
  else if (String(topic) == transmission_topic)
  {
    debugln("[Serial] Subscribed topic - Data transmission: " + String(payload));
    if (payload.toInt() == 1)
    {
      transDataReceived = true;
      debugln("\n[TRANSMISSION] --- ENDS HERE ---\n");
    }
    else if (payload.toInt() == 0)
    {
      transDataReceived = false;
      debugln("\n[TRANSMISSION] --- STARTS HERE ---\n");
    }
    else if (payload.toInt() == 999)
    {
      transitionTimeMode = 0; // 999 is code for static transition
    }
    else
    {
      transitionTimeMode = payload.toInt() * 100; // f.e. 20 * 100 = 2000ms
    }
  }
  else if (String(topic) == transitionCoefficient_topic)
  {
    debugln("[Serial] Subscribed topic - Transition Coefficient: " + String(payload));
    transCoef = payload.toFloat();
  }
  else if (String(topic) == fadeSize_topic)
  {
    debugln("[Serial] Subscribed topic - FadeSize: " + String(payload));
    unsigned int key = payload.toInt();
    if (key >= 1 && key <= 4 && key != fadeSize)
    {
      fadeSize = key;
      EEPROM.write(fadeSizeAdress, fadeSize);
      EEPROM.commit();
      debugln("\n[LED] FADESIZE was saved to " + String(key) + "!");
    }
    else if (key == fadeSize)
      ;
    else
      debugln("\n[LED] " + String(key) + " is out of range for parameter FADESIZE!");
  }
  else
  {
    debugln("[Serial] Not a subscribed topic!");
    return false;
  }
  return true;
}

bool checkSerial()
{
  if (millis() > (lastAliveMsg + aliveMsgTimeout) || !hostInitConnection)
  {
    setEmergencyMode();
    initSerial();
    digitalWrite(LED_BUILTIN, LOW); // Default on
    debugln("\n[Serial-WD] Host successfully connected!");
  }
  return true;
}

void initSerial()
{
  debug("\n[Serial-WD] Waiting for Serial Connection...");
  serialConnected = false;
  unsigned long timer = 0;
  while (!serialConnected)
  {
    if (millis() > (timer + 250)) // || (!hostInitConnection && millis() > (timer + 50))
    {
      Serial.print("status!cnt-wtg$");
      timer = millis();
    }
    if (Serial.available())
    {
      debug("[Serial-WD] Message arrived - Topic: '");
      String topic = Serial.readStringUntil('!');
      debug(topic + "' - Payload: '");
      String payload = Serial.readStringUntil('$');
      debugln(payload + "'\n");

      if (String(topic) == status_topic)
      {
        if (String(payload) == aliveMsg || String(payload) == "host-wasborn")
        {
          serialConnected = true;
          hostInitConnection = true;
          lastAliveMsg = millis();
          delay(100);
          Serial.print("status!cnctd$");
        }
      }
    }
    yield();
  }
}

// Serial Timeout Watchdog
void serialEmergencyTOWD()
{
  if (millis() > (lastAliveMsg + aliveMsgTimeout) && hostInitConnection)
  {
    checkSerial();
    debugln("\n[Timeout-WD] Emergency Serial Client timed out!");
  }
  else if (millis() == (lastAliveMsg + 6500))
    debugln("\n[Timeout-WD] Emergency Serial Client silent since 6.5 seconds...");
  else if (millis() == (lastAliveMsg + 7000))
    debugln("\n[Timeout-WD] Emergency Serial Client silent since 7 seconds...");
}