// ***---*** Project - Star Client - Emergency ***---***

// ***---*** Versions ***---***
/*--- Changelog ---
  18/11/21 - V1.0
  ~ Added physical button to execute api override command (button 1 - D1 - touch)
  ~ Added checkWiFi to automatically reconnect to host qC
  ~ Added EEPROM system to store variable states also on powerloss
  ~ Added builtin LED blink pattern to make visible wheter get request was successful
  27/11/21 - V1.1
  ~ Added mqtt client connectivity to listen to messages from host broker
  - API calls are therefore now accessed via mqtt publishes and callbacks
  18/02/22 - V2.0
  ~ Updated whole system (starhost + emergency + underglow) to new generation 2.0
  ~ Added physical button slider with 4 states (fxmodes: emergency, default, favorite, strobe)
  ~ Added physical status pixel (blue = mcu on, green = starhost connected, red = emergency active)
  ~ Added secret restart pattern: slide button states like so  1, 4, 1, 4, 1 and whole system will execute a restart
  23/02/22 - V2.01
  ~ Modified StarEmergency to not restart itself in favorite mode (selMode = 3) if Starhost lost its mqtt connection
  - This fixes the issue of not being able to do an OTA update to StarHost (via StarEmergency - its the "router")

  --- Bugs ---
  ~ No known bugs
*/

//***** INCLUDES *****
#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include "Adafruit_NeoPixel.h"

//***** DEFINES *****
// Version
#define VERSION 2.01

// Debug
#define DEBUG 0

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugHEX(x) Serial.print(x, HEX)
#define debugDEC(x) Serial.print(x, DEC)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugHEX(x)
#define debugDEC(x)
#define debugln(x)
#endif

// WiFi
#define APSSID "StarEmergency"
#define APPSK "starnetwork"

// Inputs^^
#define mode1Pin 5   // D1
#define mode2Pin 4   // D2
#define mode3Pin 14  // D5
#define mode4Pin 12  // D6

// MQTT Client
#define emergency_avemsg "emeg_ave"
#define starhost_avemsg "host_ave"

// WS2812FX
#define LED_PIN 13  // D7
#define LED_COUNT 1
#define LED_BRTNS 48  // Maximum brtns of status pixel

// CAN topics
#define CAN_txID 0x321
#define CAN_DLC 8
#define CAN_clientID 0x1

#define CAN_tpc_ping 0x0
#define CAN_tpc_selMode 0x1
#define CAN_tpc_apiovr 0x2
#define CAN_tpc_reset 0x3

//***** VARIABLES & OBJECTS *****
// Modes & Restrictions
unsigned int selectedMode = -1;
unsigned int selectedModeBefore = -1;
unsigned long modeChangeTime = 0;
const unsigned long modeChangeTimeThres = 350;

bool apiOverrideOff;

int ledBlinkCode = 0;
int ledBlinkCounter = 0;
bool ledBlinkInProgress = false;
unsigned long ledBlinkLastBlink = 0;

// Reset detection
unsigned int lastModes[4] = { 0, 0, 0, 0 };

// Client timeout
bool connected_clients[3] = { false, false, false };  // 0 - Host | 1 - Shift | 2 - CANChild
unsigned const int aveMsgTimeout = 10000;
const unsigned int aveMsgIntervall = 2500;  // interval in which host sends can rqst to ping alive
unsigned long myLastAveMsg = 0;             // timer to send can ping rqst

// Star Emergency - API Emergency Mode
unsigned long host_lastAveMsg = 0;  // stores when last alive msg was received

// Star ShiftGuidance
unsigned long shift_lastAveMsg = 0;  // stores when last alive msg was received

// CAN Child
unsigned long canChild_lastAveMsg = 0;  // stores when last alive msg was received

// WS2812FX Status Pixel
Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//***** PROTOTYPES *****
void setup();
void loop();
void handlers();
void statusPixel();
void readInputs();
void interpretInputs();
void ledBlink();
void ledBlinkError();
void ledBlinkSuccess();
String CAN_publisher(byte, byte);
void CAN_sendMessage(unsigned long, byte, byte[]);
void CAN_aliveMessage();
uint8_t CAN_checkMessages();
void esp_restart();
bool string_find(char *, char *);

//***** SETUP *****
void setup() {
  Serial.begin(38400);

  debugln("\n[StarClient-Emergency] Starting programm ~ by spl01t*#7");
  debugln("[StarClient-Emergency] You are running version " + String(VERSION) + "!");

  // IOs
  pinMode(mode1Pin, INPUT_PULLUP);
  pinMode(mode2Pin, INPUT_PULLUP);
  pinMode(mode3Pin, INPUT_PULLUP);
  pinMode(mode4Pin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // WiFi
  WiFi.setOutputPower(20.5);
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(APSSID, APPSK)) {
    debug("\n[WiFi] AP Host-IP: ");
    debugln(WiFi.softAPIP());
  }

  // OTA
  ArduinoOTA.setHostname("starEmergency");
  ArduinoOTA.begin();
  debugln("\n[OTA] Service started!");

  // LEDs
  leds.begin();
  leds.clear();

  for (int i = 0; i < 65535; i++) {
    yield();
    leds.setPixelColor(0, leds.ColorHSV(i));
    leds.show();
    i += (65535 / 1250);
    delay(1);
  }

  debugln("\n[StarControl-Client] Initialization completed...starting programm loop!\n");
}

//***** LOOP *****
void loop() {
  handlers();

  statusPixel();

  ledBlink();

  readInputs();

  interpretInputs();

  CAN_aliveMessage();

  CAN_checkMessages();
}

//***** FUNCTIONS *****
// Handler Functions
void handlers() {
  ArduinoOTA.handle();
  yield();
}

// Status Pixel
void statusPixel() {
  static unsigned long timer = 0;

  if (selectedMode == 1 || selectedMode == 2) {
    bool red = false;
    bool green = false;
    bool blue = true;

    if (selectedMode == 1)
      red = true;
    else
      red = false;

    if (connected_clients[0])
      green = true;

    uint32_t color = 0;
    if (red)
      color += 0xFF0000;
    if (green)
      color += 0xFF00;
    if (blue && !red && !green)
      color += 0xFF;

    if (leds.getPixelColor(0) != color && connected_clients[2]) {
      leds.setPixelColor(0, color);
      leds.setBrightness(LED_BRTNS);
      leds.show();
    } else if (!connected_clients[2]) {
      if (millis() > timer + 500) {
        timer = millis();
        if (leds.getPixelColor(0) > 0)
          leds.setPixelColor(0, 0);
        else
          leds.setPixelColor(0, color);
        leds.show();
      }
    }
  } else if (selectedMode == 3 && millis() > timer + 50) {
    timer = millis();
    static uint16_t colorWheel = 0;
    leds.setPixelColor(0, leds.ColorHSV(colorWheel));
    leds.show();
    colorWheel += 750;
    if (colorWheel > 65535)
      colorWheel = 0;
  } else if (selectedMode == 4 && millis() > timer + 50) {
    timer = millis();
    static bool strobeState = false;
    if (strobeState) {
      leds.setPixelColor(0, 0xFFFFFF);
      leds.show();
    } else {
      leds.setPixelColor(0, 0);
      leds.show();
    }
    strobeState = !strobeState;
  }
}

// Read Inputs
void readInputs() {
  if (!digitalRead(mode1Pin) && digitalRead(mode2Pin) && digitalRead(mode3Pin) && digitalRead(mode4Pin) && selectedMode != 1) {
    // Mode 1 - All off
    selectedMode = 1;
    modeChangeTime = millis();
  } else if (digitalRead(mode1Pin) && !digitalRead(mode2Pin) && digitalRead(mode3Pin) && digitalRead(mode4Pin) && selectedMode != 2) {
    // Mode 2 - Star on but listens to motor restriction - UGLW on with resting motor
    selectedMode = 2;
    modeChangeTime = millis();
  } else if (digitalRead(mode1Pin) && digitalRead(mode2Pin) && !digitalRead(mode3Pin) && digitalRead(mode4Pin) && selectedMode != 3) {
    // Mode 3 - Star strict on - UGLW fav fx mode strict on
    selectedMode = 3;
    modeChangeTime = millis();
  } else if (digitalRead(mode1Pin) && digitalRead(mode2Pin) && digitalRead(mode3Pin) && !digitalRead(mode4Pin) && selectedMode != 4) {
    selectedMode = 4;
    modeChangeTime = millis();
  }
}

void interpretInputs() {
  // Added some delay between deciding and transmitting which mode is selected to prevent quick and dirty switches from one mode to another -> can be conflicting with WS2812FX and also does not look good
  if (selectedModeBefore != selectedMode && millis() > (modeChangeTime + modeChangeTimeThres)) {
    selectedModeBefore = selectedMode;
    // debugln("\n" + CAN_publisher(CAN_tpc_selMode, (byte)selectedMode));
    CAN_publisher(CAN_tpc_selMode, (byte)selectedMode);
    switch (selectedMode) {
      case 1:
        // Mode 1 - All off
        debug("[INPUT] Mode 1 was selected!");
        break;

      case 2:
        // Mode 2 - Star on but listens to motor restriction - UGLW on with resting motor
        debug("[INPUT] Mode 2 was selected!");
        break;

      case 3:
        debug("[INPUT] Mode 3 was selected!");
        break;

      case 4:
        // Mode 4 - Strobe both strict on
        debug("[INPUT] Mode 4 was selected!");
        break;

      default:
        break;
    }

    // Decide to set or reset APIOvrOff
    if (selectedMode == 1) {
      debugln(" - Emergency on!");
      // debugln("\n" + CAN_publisher(CAN_tpc_apiovr, 1));
      CAN_publisher(CAN_tpc_apiovr, 1);
      apiOverrideOff = true;
    } else {
      debugln(" - Emergency off!");
      // debugln("\n" + CAN_publisher(CAN_tpc_apiovr, 0));
      CAN_publisher(CAN_tpc_apiovr, 0);
      apiOverrideOff = false;
    }

    // Reset array
    for (int i = 3; i > 0; i--) {
      lastModes[i] = lastModes[i - 1];
    }
    lastModes[0] = selectedMode;

    if (lastModes[0] == 1 && lastModes[1] == 4 && lastModes[2] == 1 && lastModes[3] == 4) {
      debugln("\n[RESET] Detected reset request!");
      // debugln("\n" + CAN_publisher(CAN_tpc_reset, 1));
      CAN_publisher(CAN_tpc_reset, 1);
      Serial.print("CR!");
      for (int i = 0; i < 65535; i++) {
        yield();
        leds.setPixelColor(0, leds.ColorHSV(i));
        leds.show();
        i += (65535 / 1250);
        delay(1);
      }
      WiFi.mode(WIFI_OFF);
      delay(100);
      debugln("\n*****************************************");
      debugln("\n[RESET] Restarting at your wish master ;)");
      debugln("\n*****************************************");
      esp_restart();
    }
  }
}

// BUILTINLED Blinks
void ledBlink() {
  if (ledBlinkCode != 0) {
    switch (ledBlinkCode) {
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

void ledBlinkError() {
  if (!ledBlinkInProgress) {
    ledBlinkInProgress = true;
    ledBlinkCounter = 0;
  }

  if ((ledBlinkLastBlink + 50) < millis()) {
    ledBlinkLastBlink = millis();

    if (digitalRead(LED_BUILTIN)) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }

    ledBlinkCounter++;
  }

  if (ledBlinkCounter >= 20) {
    ledBlinkCode = 0;
    ledBlinkInProgress = false;
    digitalWrite(LED_BUILTIN, LOW);  // Default on
  }
}

void ledBlinkSuccess() {
  if (!ledBlinkInProgress) {
    ledBlinkInProgress = true;
    ledBlinkCounter = 0;
  }

  if ((ledBlinkLastBlink + 100) < millis()) {
    ledBlinkLastBlink = millis();

    if (digitalRead(LED_BUILTIN)) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }

    ledBlinkCounter++;
  }

  if (ledBlinkCounter >= 10) {
    ledBlinkCode = 0;
    ledBlinkInProgress = false;
    digitalWrite(LED_BUILTIN, LOW);  // Default on
  }
}

// CAN Publisher
String CAN_publisher(byte topic, byte payload) {
  byte txData[CAN_DLC] = { CAN_clientID, 0x1, topic, payload, 0x0, 0x0, 0x0, 0x0 };
  CAN_sendMessage(CAN_txID, CAN_DLC, txData);

  ledBlinkCode = 2;

  return "[CAN] Publish on '" + String(topic) + "' with '" + String(payload) + "' successful!\n";
}

// CAN Handlers
void CAN_sendMessage(unsigned long txID, byte dlc, byte payload[]) {
  if (connected_clients[2]) {
    Serial.print(txID, HEX);
    for (int i = 0; i < (int)dlc; i++) {
      Serial.print(',');
      if (payload[i] < 16)
        Serial.print("0");
      Serial.print(payload[i], HEX);
    }
    Serial.print('!');
  }
}

void CAN_aliveMessage() {
  if (((millis() > (myLastAveMsg + aveMsgIntervall)) || (myLastAveMsg == 0U)) && connected_clients[2]) {
    myLastAveMsg = millis();
    byte payload[CAN_DLC] = { CAN_clientID, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0 };
    CAN_sendMessage(CAN_txID, CAN_DLC, payload);  // Sending ping rqst to clients
    Serial.print("CC!");
  }

  // Timeout StarEmergency
  if (millis() > (host_lastAveMsg + aveMsgTimeout) && selectedMode != 3 && connected_clients[0] && connected_clients[2]) {
    debugln("\n[Timeout-WD] StarHost timed out!");
    esp_restart();
  }

  // Timeout ShiftGuidance
  if (millis() > (shift_lastAveMsg + aveMsgTimeout) && connected_clients[1]) {
    connected_clients[1] = false;
    debugln("\n[Timeout-WD] StarClient ShiftGuidance timed out!");
  }

  // Timeout CANChild
  if (millis() > (canChild_lastAveMsg + aveMsgTimeout) && connected_clients[2]) {
    connected_clients[2] = false;
    debugln("\n[Timeout-WD] CANChild timed out!");
  }
}

uint8_t CAN_checkMessages() {
  if (!Serial.available())
    return 1;

  String rxMsg = Serial.readStringUntil('!');

  // debugln("[UART] RX MSG: " + String(rxMsg));

  // Preparing serial string --> getting length and converting to char ary
  unsigned int msgLength = rxMsg.length();
  char rxMsgChar[msgLength];
  rxMsg.toCharArray(rxMsgChar, msgLength + 1, 0);

  // Checking for can child readiness
  if (string_find(rxMsgChar, "CA")) {
    if (!connected_clients[2])
      debugln("[CAN] CANChild connected!");
    connected_clients[2] = true;
    canChild_lastAveMsg = millis();
    Serial.print("PA!");
    return 0;
  } else if (string_find(rxMsgChar, "CL")) {
    if (!connected_clients[2])
      debugln("[CAN] CANChild connected!");
    connected_clients[2] = true;
    canChild_lastAveMsg = millis();
    return 0;
  }

  // Extracting can msg id value
  char idChars[4] = { rxMsgChar[0], rxMsgChar[1], rxMsgChar[2], '\0' };
  unsigned long id = (unsigned long)strtol(idChars, 0, 16);

  if (id != 0x321)  // Filter unwanted ids
  {
    debugln("[CAN] Filtering ID: " + String(id));
    return 1;
  }

  // Converting two chars into one hex value
  byte payload[CAN_DLC];
  for (int i = 0; i < CAN_DLC; i++) {
    char hexChars[3] = { rxMsgChar[4 + i * 3], rxMsgChar[5 + i * 3], '\0' };
    payload[i] = (byte)strtol(hexChars, 0, 16);
  }

  // Debug output
  debug("\n[CAN] RX: ID: ");
  debugHEX(id);
  debug(" | DATA: ");
  for (int i = 0; i < CAN_DLC; i++) {
    if ((int)payload[i] < 16)
      debug("0");
    debugHEX(payload[i]);
    debug(" ");
  }

  // Message interpretation
  if (payload[2] == CAN_tpc_reset) {
    if (payload[0] == 0x0 && payload[1] == 0x1 && payload[3] == 0x1) {
      debugln("\n*****************************************");
      debugln("\n[RESET] Restarting at your wish master ;)");
      debugln("\n*****************************************");
      esp_restart();
    } else
      debugln("[CAN] ERR Topic Reset: Unknown message!");
  } else if (payload[2] == CAN_tpc_ping) {
    if (payload[1] == 0x0)  // Request
    {
      debugln("\n[CAN] Message: Ping request");
      byte txPL[CAN_DLC] = { CAN_clientID, 0x1, CAN_tpc_ping, 0x1, 0x0, 0x0, 0x0, 0x0 };
      CAN_sendMessage(CAN_txID, CAN_DLC, txPL);
    } else if (payload[1] == 0x1)  // Answer
    {
      if (payload[0] == 0x0 && payload[1] == 0x1 && payload[3] == 0x1) {
        debugln("\n[CAN] Message: Ping from StarHost");
        if (!connected_clients[0]) {
          debugln("[Client] StarHost connected! Sending current data...");
          if (selectedMode == 1)
            // debugln("\n" + CAN_publisher(CAN_tpc_apiovr, 1));
            CAN_publisher(CAN_tpc_apiovr, 1);
          else
            // debugln("\n" + CAN_publisher(CAN_tpc_apiovr, 0));
            CAN_publisher(CAN_tpc_apiovr, 0);
          // debugln("\n" + CAN_publisher(CAN_tpc_selMode, (byte)selectedMode));
          CAN_publisher(CAN_tpc_selMode, (byte)selectedMode);
        }
        host_lastAveMsg = millis();
        connected_clients[0] = true;
      } else if (payload[2] == 0x1 && payload[1] == 0x1 && payload[3] == 0x1) {
        debugln("\n[CAN] Message: Ping from StarShiftGuidance");
        if (!connected_clients[1])
          debugln("[Client] ShiftGuidance connected!");
        shift_lastAveMsg = millis();
        connected_clients[1] = true;
      } else
        debugln("[CAN] ERR Topic Ping: Unknown client!");
    } else
      debugln("[CAN] ERR Topic Ping: Unknown message!");
  } else {
    debugln("\n[CAN] ERR Not a subscribed topic!");
  }

  return 0;
}

void esp_restart() {
  if (selectedMode != 3)
    ESP.restart();
}

bool string_find(char *haystack, char *needle) {
  int compareOffset = 0;
  while (*haystack) {
    if (*haystack == *needle) {
      compareOffset = 0;
      while (haystack[compareOffset] == needle[compareOffset]) {
        compareOffset++;
        if (needle[compareOffset] == '\0') {
          return true;
        }
      }
    }
    haystack++;
  }
  return false;
}