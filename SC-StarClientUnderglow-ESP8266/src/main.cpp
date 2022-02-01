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

  --- Bugs ---
  ~ No known bugs
*/

//***** INCLUDES *****
#include "Arduino.h"
#include <EEPROM.h>
#include <WS2812FX.h>

//***** DEFINES *****
// Version
#define VERSION 1.1

// Debug
#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

// IOs
#define LED_PIN 5   // D1
#define RELAY_PIN 4 // D2

// EEPROM
#define apiOverrideOffAdress 0
#define modeAdress 1
#define color1Adress 2 // 0x0000FF - Blue
#define color2Adress 3 // 0x00FF00 - Green
#define color3Adress 4 // 0xFF0000 - Red
#define brtnsAdress 5
#define speed1Adress 6 // 0x00FF
#define speed2Adress 7 // 0xFF00 - 0 to 65535
#define motorBlockAdress 8

// WS2812 LEDs
#define LED_COUNT 600 // INCREASE THIS !!! 300 only for testbench

// Serial Client Topics
#define status_topic "status"
#define apiOvrOff_topic "api/ovroff"
#define mode_topic "mode"
#define color_topic "color"
#define brtns_topic "brtns"
#define speed_topic "speed"
#define motor_topic "motor"
#define transmission_topic "trans"
#define transitionCoefficient_topic "transCoef"
#define reset_topic "reset"

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
unsigned int led_color;
unsigned int led_brtns;
unsigned int led_speed;

unsigned int led_mode_bef = -1;
unsigned int led_color_bef = -1;
unsigned int led_brtns_bef = -1;
unsigned int led_speed_bef = -1;

// Transition
bool transitionActive = false;
bool transDown = false;
bool transUp = false;
bool activeStrip = true; // true = 1 | false = 2
float transCoef = 1.18;

struct
{
    unsigned int led_mode;
    unsigned int led_color;
    unsigned int led_brtns;
    unsigned int led_speed;
} v1Strip;

struct
{
    unsigned int led_mode;
    unsigned int led_color;
    unsigned int led_brtns;
    unsigned int led_speed;
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
void initLastState();
void writeColorEEPROM(unsigned int);
unsigned int readColorEEPROM();
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
    if (DEBUG)
        Serial.begin(115200);
    Serial.setTimeout(3);
    EEPROM.begin(9);

    debugln("\n[StarClient-Underglow] Starting programm ~ by spl01t*#7");
    debugln("[StarClient-Underglow] You are running version " + String(VERSION) + "!");

    // IOs
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH); // Activate "power" to ws2812 strips

    // Get EEPROM memory
    initLastState();

    // LEDs - 2 virtual strips to transition between
    leds.init();
    leds.setSegment(0, 0, LED_COUNT - 1, 0U, 0U, 0U);
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
        v2Strip.led_color = led_color;
        v2Strip.led_brtns = led_brtns;
        v2Strip.led_speed = led_speed;
        debugln("\n[TRANS] Prepared Virtual 2 - Mode: " + String(v2Strip.led_mode) + " - Color: " + String(v2Strip.led_color) + " - Brtns: " + String(v2Strip.led_brtns) + " - Speed: " + String(v2Strip.led_speed) + "\n");
    }
    else // V1
    {
        v1Strip.led_mode = led_mode;
        v1Strip.led_color = led_color;
        v1Strip.led_brtns = led_brtns;
        v1Strip.led_speed = led_speed;
        debugln("\n[TRANS] Prepared Virtual 1 - Mode: " + String(v1Strip.led_mode) + " - Color: " + String(v1Strip.led_color) + " - Brtns: " + String(v1Strip.led_brtns) + " - Speed: " + String(v1Strip.led_speed) + "\n");
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
                    leds.setSegment(0, 0, LED_COUNT - 1, v1Strip.led_mode, v1Strip.led_color, v1Strip.led_speed);
                    debugln("\n[TRANS] Reached half transition! - Loading settings V1\n");
                }
                else
                {
                    leds.setSegment(0, 0, LED_COUNT - 1, v2Strip.led_mode, v2Strip.led_color, v2Strip.led_speed);
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
                leds.setSegment(0, 0, LED_COUNT - 1, v1Strip.led_mode, v1Strip.led_color, v1Strip.led_speed);
                leds.setBrightness(v1Strip.led_brtns);
            }
            else
            {
                leds.setSegment(0, 0, LED_COUNT - 1, v2Strip.led_mode, v2Strip.led_color, v2Strip.led_speed);
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

        if (led_color != led_color_bef)
        {
            led_color_bef = led_color;
            if (!transitionChanges)
            {
                leds.setColor(led_color);
            }
            debugln("\n[LED] COLOR was changed to " + String(led_color) + "!");
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
void initLastState()
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

    // LEDs Color
    unsigned int colorContent = readColorEEPROM();

    debugln("[EEPROM] LEDs - Color: " + String(colorContent));

    if (colorContent >= 0 && colorContent <= 16777215)
    {
        led_color = colorContent;
    }
    else
    {
        debugln("[EEPROM] Reading was no valid option: LEDs - Color - Out of range (0x0-0xFFFFFF)!");
        led_color = 0;
        writeColorEEPROM(0);
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

    EEPROM.commit();

    debugln("[EEPROM] Extraction completed!");
}

void writeColorEEPROM(unsigned int key)
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

    EEPROM.write(color1Adress, lowbit);  // Blue Bit
    EEPROM.write(color2Adress, midbit);  // Green Bit
    EEPROM.write(color3Adress, highbit); // Red Bit
    EEPROM.commit();
}

unsigned int readColorEEPROM()
{
    int key = EEPROM.read(color3Adress) << 8;     // Red Bit
    key = (key + EEPROM.read(color2Adress)) << 8; // Green Bit
    key += EEPROM.read(color1Adress);             // Blue Bit

    /*debugln("\nKEY DEC: " + String(key));
    debugln("KEY HEX: " + String(key, HEX));
    debugln("KEY BIN: " + String(key, BIN));*/

    return key;
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
        leds.setSegment(0, 0, LED_COUNT - 1, led_mode, led_color, led_speed);
        leds.setBrightness(led_brtns);
        if (activeStrip)
            debugln("\n --- Virtual 1 EMEG - Settings - Mode: " + String(leds.getMode()) + " - Color: " + String(leds.getColor()) + " - Brtns: " + String(leds.getBrightness()) + " - Speed: " + String(leds.getSpeed()) + " ---\n");
        else
            debugln("\n --- Virtual 2 EMEG - Settings - Mode: " + String(leds.getMode()) + " - Color: " + String(leds.getColor()) + " - Brtns: " + String(leds.getBrightness()) + " - Speed: " + String(leds.getSpeed()) + " ---\n");
        led_mode_bef = led_mode;
        led_color_bef = led_color;
        led_brtns_bef = led_brtns;
        led_speed_bef = led_speed;
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
        {
            debug("[Serial] Message arrived - Topic: '" + topic + "' - Payload: '" + payload + "'\n");
        }
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
        {
            initSerial();
        }
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
    else if (String(topic) == color_topic) // LED Color Handler
    {
        debugln("[Serial] Subscribed topic - Underglow Color: " + String(payload));
        unsigned int key = payload.toInt();
        if (key >= 0 && key <= 16777215 && key != led_color)
        {
            led_color = key;
            writeColorEEPROM(led_color);
            debugln("\n[LED] COLOR was saved to " + String(key) + "!");
        }
        else if (key == led_color)
            ;
        else
            debugln("\n[LED] " + String(key) + " is out of range for parameter COLOR!");
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
        return true;
    }
    else
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