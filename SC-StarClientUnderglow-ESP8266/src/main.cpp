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
#define LED_COUNT 300 // INCREASE THIS !!! 300 only for testbench
#define LED_PIN 5     // D1

// Serial Client Topics
#define status_topic "status"
#define apiOvrOff_topic "api/ovroff"
#define mode_topic "mode"
#define color_topic "color"
#define brtns_topic "brtns"
#define speed_topic "speed"
#define motor_topic "motor"
#define transmission_topic "trans"

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

WS2812FXT leds = WS2812FXT(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

bool activeStrip = true; // true = 1 | false = 2

unsigned int led_mode;
unsigned int led_color;
unsigned int led_brtns;
unsigned int led_speed;

unsigned int led_mode_bef = -1;
unsigned int led_color_bef = -1;
unsigned int led_brtns_bef = -1;
unsigned int led_speed_bef = -1;

bool uglwMotorBlock = false;
bool motorBlockTransition = false;

const unsigned int transitionTimeMotorBlock = 2000; // 1s to switch off uglw on starting motor
unsigned int transitionTimeMode = 1000;

bool transDataReceived = true;

//***** PROTOTYPES *****
void setup();
void loop();
void handlers();
void applyLEDSettings();
void ledBlink();
void ledBlinkError();
void ledBlinkSuccess();
bool checkSerial();
void initLastState();
void writeColorEEPROM(unsigned int);
unsigned int readColorEEPROM();
void writeSpeedEEPROM(unsigned int);
unsigned int readSpeedEEPROM();
void setEmergencyMode();
void resetEmergencyMode();
bool serialCallback();
void reconnect();
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

    // Get EEPROM memory
    initLastState();

    // LEDs - 2 virtual strips to transition between
    leds.init();
    leds.v1->setSegment(0, 0, LED_COUNT - 1, 0U, 0U, 0U);
    leds.v1->setBrightness(0);
    leds.v2->setSegment(0, 0, LED_COUNT - 1, 0U, 0U, 0U);
    leds.v2->setBrightness(0);
    leds.start();

    // Communication Config
    checkSerial();

    debugln("\n[StarControl-Client] Initialization completed...starting programm loop!\n");
}

//***** LOOP *****
void loop()
{
    handlers();

    applyLEDSettings();

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
void applyLEDSettings()
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
                if (activeStrip)
                    leds.v1->setColor(led_color);
                else
                    leds.v2->setColor(led_color);
            }
            debugln("\n[LED] COLOR was changed to " + String(led_color) + "!");
        }

        if (led_brtns != led_brtns_bef)
        {
            led_brtns_bef = led_brtns;
            if (!transitionChanges)
            {
                if (activeStrip)
                    leds.v1->setBrightness(led_brtns);
                else
                    leds.v2->setBrightness(led_brtns);
            }
            debugln("\n[LED] BRIGHTNESS was changed to " + String(led_brtns) + "!");
        }

        if (led_speed != led_speed_bef)
        {
            led_speed_bef = led_speed;
            if (!transitionChanges)
            {
                if (activeStrip)
                    leds.v1->setSpeed(led_speed);
                else
                    leds.v2->setSpeed(led_speed);
            }
            debugln("\n[LED] SPEED was changed to " + String(led_speed) + "!");
        }

        if (transitionChanges)
        {
            if (activeStrip)
            {
                leds.v2->setSegment(0, 0, LED_COUNT - 1, led_mode, led_color, led_speed);
                leds.v2->setBrightness(led_brtns);
            }
            else
            {
                leds.v1->setSegment(0, 0, LED_COUNT - 1, led_mode, led_color, led_speed);
                leds.v1->setBrightness(led_brtns);
            }
            leds.startTransition(transitionTimeMode, activeStrip);
            activeStrip = !activeStrip;
            debugln("\n[LED] Started transition - mode changed!");
        }
    }
    else if (uglwMotorBlock && motorBlockTransition)
    {
        // Motor-Blockage was received --> Transition to uglw off
        motorBlockTransition = false;
        if (activeStrip)
        {
            leds.v2->setSegment(0, 0, LED_COUNT - 1, leds.v1->getMode(), leds.v1->getColor(), leds.v1->getSpeed());
            leds.v2->setBrightness(0);
        }
        else
        {
            leds.v1->setSegment(0, 0, LED_COUNT - 1, leds.v2->getMode(), leds.v2->getColor(), leds.v2->getSpeed());
            leds.v1->setBrightness(0);
        }
        leds.startTransition(transitionTimeMotorBlock, activeStrip);
        activeStrip = !activeStrip;
        debugln("\n[LED] Started transition (down) because of motor blockage!");
    }
    else if (!uglwMotorBlock && motorBlockTransition)
    {
        // Motor-Blockage was released --> Transition to uglw on
        motorBlockTransition = false;
        led_mode_bef = led_mode;
        led_color_bef = led_color;
        led_brtns_bef = led_brtns;
        led_speed_bef = led_speed;
        if (activeStrip)
        {
            leds.v2->setSegment(0, 0, LED_COUNT - 1, led_mode, led_color, led_speed);
            leds.v2->setBrightness(led_brtns);
        }
        else
        {
            leds.v1->setSegment(0, 0, LED_COUNT - 1, led_mode, led_color, led_speed);
            leds.v1->setBrightness(led_brtns);
        }
        leds.startTransition(transitionTimeMotorBlock, activeStrip);
        activeStrip = !activeStrip;
        debugln("\n[LED] Started transition (up) because of motor blockage release!");
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
        leds.v1->setBrightness(0);
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
    if (activeStrip)
        leds.v1->setBrightness(0);
    else
        leds.v2->setBrightness(0);
    leds.service();
    debugln("\n[EMERGENCY] Mode activated!");
}

void resetEmergencyMode()
{
    emergency = false;
    if (!uglwMotorBlock)
    {
        if (activeStrip)
        {
            leds.v1->setSegment(0, 0, LED_COUNT - 1, led_mode, led_color, led_speed);
            leds.v1->setBrightness(led_brtns);
        }
        else
        {
            leds.v2->setSegment(0, 0, LED_COUNT - 1, led_mode, led_color, led_speed);
            leds.v2->setBrightness(led_brtns);
        }
        led_mode_bef = led_mode;
        led_color_bef = led_color;
        led_brtns_bef = led_brtns;
        led_speed_bef = led_speed;
    }
    debugln("\n[EMERGENCY] Mode deactivated!");
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
    }
    else if (String(topic) == apiOvrOff_topic) // API Override Off Handler
    {
        debugln("[Serial] Subscribed topic - API Override Light Off: " + String(payload));
        if (!serialInitDataReceived)
        {
            serialInitDataReceived = true;
            debugln("\n[EMERGENCY] Received init Data!");
        }
        if (payload == "1")
        {
            apiOverrideOff = true;
            setEmergencyMode();
            EEPROM.write(apiOverrideOffAdress, 1);
            EEPROM.commit();
        }
        else if (payload == "0")
        {
            apiOverrideOff = false;
            resetEmergencyMode();
            EEPROM.write(apiOverrideOffAdress, 0);
            EEPROM.commit();
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
            debugln("\n[LED] MODE was saved to " + String(key) + "!");
        }
        else if (key == led_mode);
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
        else if (key == led_color);
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
        else if (key == led_brtns);
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
        else if (key == led_speed);
        else
            debugln("\n[LED] " + String(key) + " is out of range for parameter SPEED!");
    }
    else if (String(topic) == motor_topic)
    {
        debugln("[Serial] Subscribed topic - Underglow Motor-Blockage: " + String(payload));
        if (payload.toInt() == 1)
        {
            uglwMotorBlock = true;
            if (!emergency)
                motorBlockTransition = true;
            EEPROM.write(motorBlockAdress, 1);
        }
        else if (payload.toInt() == 0)
        {
            uglwMotorBlock = false;
            if (!emergency)
                motorBlockTransition = true;
            EEPROM.write(motorBlockAdress, 0);
        }
        EEPROM.commit();
    }
    else if (String(topic) == transmission_topic)
    {
        debugln("[Serial] Subscribed topic - Data transmission: " + String(payload));
        if (payload.toInt() == 1)
        {
            transDataReceived = true;
        }
        else if (payload.toInt() == 0)
        {
            transDataReceived = false;
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
    else
    {
        debugln("[Serial] Not a subscribed topic!");
        return false;
    }
    return true;
}

bool checkSerial()
{
    if (millis() > lastAliveMsg + aliveMsgTimeout || !hostInitConnection)
    {
        serialConnected = false;
        setEmergencyMode();
        debug("\n[Serial-WD] Waiting for Serial Connection...");
        unsigned long timer = 0;
        while (!serialConnected)
        {
            if (millis() > timer + 250 || (!hostInitConnection && millis() > (timer + 50)))
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
                    if (String(payload) == aliveMsg)
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
        digitalWrite(LED_BUILTIN, LOW); // Default on
        debugln("\n[Serial-WD] Host successfully connected!");
        return true;
    }
    else
        return true;
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