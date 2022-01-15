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

// WS2812 LEDs
#define LED_COUNT 600 // Little Strip 114
#define LED_PIN 5     // D1 - D4 = 2

// MQTT Client
#define status_topic "status"
#define apiOvrOff_topic "api/ovroff"
#define mode_topic "mode"
#define color_topic "color"
#define brtns_topic "brtns"
#define speed_topic "speed"
#define motor_topic "motor"

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

WS2812FX leds = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned int led_mode = 0;
unsigned int led_color = 0;
unsigned int led_brtns = 0;
unsigned int led_speed = 10000;

bool uglwMotorBlock;

//***** PROTOTYPES *****
void setup();
void loop();
void handlers();
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
    EEPROM.begin(8);

    debugln("\n[StarClient-Underglow] Starting programm ~ by spl01t*#7");
    debugln("[StarClient-Underglow] You are running version " + String(VERSION) + "!");

    // IOs
    pinMode(LED_BUILTIN, OUTPUT);

    // LEDs
    leds.init();
    leds.setBrightness(led_brtns);
    leds.setSpeed(led_speed);
    leds.setMode(led_mode);
    leds.setColor(led_color);
    leds.start();

    // Get EEPROM memory
    initLastState();

    // Communication Config
    checkSerial();

    led_brtns = 255;
    led_speed = 100;
    leds.setBrightness(led_brtns);
    leds.setSpeed(led_speed);

    debugln("\n[StarControl-Client] Initialization completed...starting programm loop!\n");
}

//***** LOOP *****
void loop()
{
    handlers();

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
        leds.setBrightness(0);
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
        leds.setMode(led_mode);
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
        if (!emergency)
            leds.setColor(led_color);
        writeColorEEPROM(led_color);
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
        if (emergency)
        {
            debugln("[EEPROM] LEDs - Brightness was not set, because emergency is active!");
        }
        else
        {
            leds.setBrightness(led_brtns);
        }
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
        leds.setSpeed(led_speed);
        writeSpeedEEPROM(led_speed);
    }
    else
    {
        debugln("[EEPROM] Reading was no valid option: LEDs - Speed - Out of range (0x0-0xFFFF)!");
        led_speed = 10000;
        EEPROM.write(speed1Adress, led_color);
        EEPROM.write(speed2Adress, led_color);
    }

    EEPROM.commit();
    leds.service();

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
    debugln("\n[EMERGENCY] Mode activated!");
}

void resetEmergencyMode()
{
    emergency = false;
    if (!uglwMotorBlock)
        leds.setBrightness(led_brtns);
    else
        leds.setBrightness(0);
    leds.service();
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
        debug("[Serial] Subscribed topic - Status: ");
        debugln(String(payload));
        if (payload == aliveMsg)
            lastAliveMsg = millis();
    }
    else if (String(topic) == apiOvrOff_topic) // API Override Off Handler
    {
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
        debug("[Serial] Subscribed topic - API Override Light Off: ");
        debugln(String(payload));
    }
    else if (String(topic) == mode_topic) // LED Mode Handler
    {
        debug("[Serial] Subscribed topic - Underglow Mode: ");
        debugln(String(payload));
        int key = payload.toInt();
        if (key >= 0 && key <= 56)
        {
            led_mode = key;
            leds.setMode(led_mode);
            leds.service();
            EEPROM.write(modeAdress, led_mode);
            EEPROM.commit();
            debugln("\n[LED] MODE was set to " + String(key) + "!");
        }
        else
            debugln("\n[LED] " + String(key) + " is out of range for parameter MODE!");
    }
    else if (String(topic) == color_topic) // LED Color Handler
    {
        debug("[Serial] Subscribed topic - Underglow Color: ");
        debugln(String(payload));
        int key = payload.toInt();
        if (key >= 0 && key <= 16777215)
        {
            led_color = key;
            leds.setColor(led_color);
            leds.service();
            writeColorEEPROM(led_color);
            debugln("\n[LED] COLOR was set to " + String(key) + "!");
        }
        else
            debugln("\n[LED] " + String(key) + " is out of range for parameter COLOR!");
    }
    else if (String(topic) == brtns_topic) // LED Brightness Handler
    {
        debug("[Serial] Subscribed topic - Underglow Brightness: ");
        debugln(String(payload));
        int key = payload.toInt();
        if (key >= 0 && key <= 255)
        {
            led_brtns = key;
            if (serialInitDataReceived && !apiOverrideOff && !uglwMotorBlock)
                leds.setBrightness(led_brtns);
            leds.service();
            EEPROM.write(brtnsAdress, led_brtns);
            EEPROM.commit();
            debugln("\n[LED] BRIGHTNESS was set to " + String(key) + "!");
        }
        else
            debugln("\n[LED] " + String(key) + " is out of range for parameter BRIGHTNESS!");
    }
    else if (String(topic) == speed_topic) // LED Speed Handler
    {
        debug("[Serial] Subscribed topic - Underglow Speed: ");
        debugln(String(payload));
        int key = payload.toInt();
        if (key >= 0 && key <= 65535)
        {
            led_speed = key;
            leds.setSpeed(led_speed);
            leds.service();
            writeSpeedEEPROM(led_speed);
            debugln("\n[LED] SPEED was set to " + String(key) + "!");
        }
        else
            debugln("\n[LED] " + String(key) + " is out of range for parameter SPEED!");
    }
    else if (String(topic) == motor_topic)
    {
        if (payload.toInt() == 1)
        {
            uglwMotorBlock = true;
            leds.setBrightness(0);
            leds.service();
        }
        else if (payload.toInt() == 0)
        {
            uglwMotorBlock = false;
            if (!emergency)
            {
                leds.setBrightness(led_brtns);
                leds.service();
            }
        }
        debug("[Serial] Subscribed topic - Underglow Motor-Restriction: " + String(payload));
    }
    else
    {
        debugln("[Serial] Not a subscribed topic (0x1)");
        return false;
    }
    debugln();
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