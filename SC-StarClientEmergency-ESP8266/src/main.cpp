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
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include "uMQTTBroker.h"
#include <ArduinoOTA.h>
#include "Adafruit_NeoPixel.h"

//***** DEFINES *****
// Version
#define VERSION 2.01

// Debug
#define DEBUG 0

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

// WiFi
#define APSSID "Mercedes Star Control"
#define APPSK "sploithd"

// EEPROM
#define apiOverrideOffAdress 0

// Inputs
#define mode1Pin 5  // D1
#define mode2Pin 4  // D2
#define mode3Pin 14 // D5
#define mode4Pin 12 // D6

// MQTT Client
#define mqtt_server "192.168.0.1"
#define status_topic "status"
#define apiOvrOff_topic "api/ovroff"
#define alive_topic "alive"
#define mode_topic "mode"
#define color_topic "color"
#define brtns_topic "brtns"
#define speed_topic "speed"
#define fxmode_topic "fxmode"
#define reset_topic "reset"

#define emergency_avemsg "emeg_ave"
#define starhost_avemsg "host_ave"

// WS2812FX
#define LED_PIN 13 // D7
#define LED_COUNT 1
#define LED_BRTNS 48 // Maximum brtns of status pixel

//***** VARIABLES & OBJECTS *****
// WiFi Variables
IPAddress local_IP = IPAddress(192, 168, 4, 10);
IPAddress gateway = IPAddress(192, 168, 4, 1);
IPAddress subnet = IPAddress(255, 255, 255, 0);

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
unsigned int lastModes[4] = {0, 0, 0, 0};

// MQTT Broker
// Prototypes
String mqttPublisher(String, String);
bool hostConnected = false;
bool hostWasConnected = false;
unsigned long myLastAveMsg = 0;
const unsigned int aveMsgIntervall = 2500;
unsigned long yourlastAveMsg = 0; // stores when last alive msg was received
unsigned const int aveMsgTimeout = 10000;

class MQTTBroker : public uMQTTBroker
{
public:
  virtual bool onConnect(IPAddress addr, uint16_t client_count)
  {
    debugln("[MQTT] Client '" + addr.toString() + "' connected!");
    return true;
  }

  virtual void onDisconnect(IPAddress addr, String client_id)
  {
    Serial.println("[MQTT] Client '" + client_id + "' disconnected!");
    hostConnected = false;
    if (client_id == "starhost1" && selectedMode != 3) // In favorite mode emeg will not restart so an ota update to starhost can successfully occur!
      ESP.restart();
  }

  virtual bool onAuth(String username, String password, String client_id)
  {
    return true;
  }

  virtual void onData(String topic, const char *data, uint32_t length)
  {
    char data_str[length + 1];
    os_memcpy(data_str, data, length);
    data_str[length] = '\0';
    debugln("\n[Broker] Received message - topic: " + topic + " | payload: " + (String)data_str);

    if (topic == status_topic)
    {
      if (String(data_str) == "starhost1 active!")
      {
        debugln("\n[Broker] StarHost is online, sending current data!");
        hostConnected = true;
        hostWasConnected = true;
        if (selectedMode == 1)
          debugln("\n" + mqttPublisher(apiOvrOff_topic, "1"));
        else
          debugln("\n" + mqttPublisher(apiOvrOff_topic, "0"));
        debugln("\n" + mqttPublisher(fxmode_topic, String(selectedMode)));
      }
    }
    else if (topic == alive_topic)
    {
      if (String(data_str) == starhost_avemsg)
      {
        yourlastAveMsg = millis();
        debugln("\n[MQTT] Subscribed topic - Alive Message!");
      }
    }
  }
};

MQTTBroker broker;

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
void mqttAliveMessage();

//***** SETUP *****
void setup()
{
  // Begin
  if (DEBUG)
    Serial.begin(115200);

  EEPROM.begin(1);

  debugln("\n[StarClient-Emergency] Starting programm ~ by spl01t*#7");
  debugln("[StarClient-Emergency] You are running version " + String(VERSION) + "!");

  // IOs
  pinMode(mode1Pin, INPUT_PULLUP);
  pinMode(mode2Pin, INPUT_PULLUP);
  pinMode(mode3Pin, INPUT_PULLUP);
  pinMode(mode4Pin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // WiFi
  WiFi.disconnect();
  delay(25);
  if (WiFi.softAP(APSSID, APPSK))
  {
    debug("\n[WiFi] AP started! IP address: ");
    debugln(WiFi.softAPIP());
  }
  WiFi.setOutputPower(20.5);

  // OTA
  ArduinoOTA.setHostname("starclient-emergency");
  ArduinoOTA.begin();
  debugln("\n[OTA] Service started!");

  // MQTT
  broker.init();
  broker.subscribe("#");
  debug("[MQTT] Succesfully started MQTT-Broker (IP: ");
  debug(WiFi.softAPIP());
  debugln(")");

  // LEDs
  leds.begin();
  leds.clear();

  for (int i = 0; i < 65535; i++)
  {
    yield();
    leds.setPixelColor(0, leds.ColorHSV(i));
    leds.show();
    i += (65535 / 1250);
    delay(1);
  }

  debugln("\n[StarControl-Client] Initialization completed...starting programm loop!\n");
}

//***** LOOP *****
void loop()
{
  handlers();

  statusPixel();

  ledBlink();

  readInputs();

  interpretInputs();

  mqttAliveMessage();
}

//***** FUNCTIONS *****
// Handler Functions
void handlers()
{
  ArduinoOTA.handle();
  yield();
}

// Status Pixel
void statusPixel()
{
  static unsigned long timer = 0;

  if (selectedMode == 1 || selectedMode == 2)
  {
    bool red = false;
    bool green = false;
    bool blue = true;

    if (selectedMode == 1)
      red = true;
    else
      red = false;

    if (hostWasConnected)
      green = true;

    uint32_t color = 0;
    if (red)
      color += 0xFF0000;
    if (green)
      color += 0xFF00;
    if (blue && !red && !green)
      color += 0xFF;

    if (leds.getPixelColor(0) != color)
    {
      leds.setPixelColor(0, color);
      leds.setBrightness(LED_BRTNS);
      leds.show();
    }
  }
  else if (selectedMode == 3 && millis() > timer + 50)
  {
    timer = millis();
    static uint16_t colorWheel = 0;
    leds.setPixelColor(0, leds.ColorHSV(colorWheel));
    leds.show();
    colorWheel += 750;
    if (colorWheel > 65535)
      colorWheel = 0;
  }
  else if (selectedMode == 4 && millis() > timer + 50)
  {
    timer = millis();
    static bool strobeState = false;
    if (strobeState)
    {
      leds.setPixelColor(0, 0xFFFFFF);
      leds.show();
    }
    else
    {
      leds.setPixelColor(0, 0);
      leds.show();
    }
    strobeState = !strobeState;
  }
}

// Read Inputs
void readInputs()
{
  if (!digitalRead(mode1Pin) && digitalRead(mode2Pin) && digitalRead(mode3Pin) && digitalRead(mode4Pin) && selectedMode != 1)
  {
    // Mode 1 - All off
    selectedMode = 1;
    modeChangeTime = millis();
  }
  else if (digitalRead(mode1Pin) && !digitalRead(mode2Pin) && digitalRead(mode3Pin) && digitalRead(mode4Pin) && selectedMode != 2)
  {
    // Mode 2 - Star on but listens to motor restriction - UGLW on with resting motor
    selectedMode = 2;
    modeChangeTime = millis();
  }
  else if (digitalRead(mode1Pin) && digitalRead(mode2Pin) && !digitalRead(mode3Pin) && digitalRead(mode4Pin) && selectedMode != 3)
  {
    // Mode 3 - Star strict on - UGLW rainbow strict on
    selectedMode = 3;
    modeChangeTime = millis();
  }
  else if (digitalRead(mode1Pin) && digitalRead(mode2Pin) && digitalRead(mode3Pin) && !digitalRead(mode4Pin) && selectedMode != 4)
  {
    selectedMode = 4;
    modeChangeTime = millis();
  }
}

void interpretInputs()
{
  // Added some delay between deciding and transmitting which mode is selected to prevent quick and dirty switches from one mode to another -> can be conflicting with WS2812FX and also does not look good
  if (selectedModeBefore != selectedMode && millis() > (modeChangeTime + modeChangeTimeThres))
  {
    selectedModeBefore = selectedMode;
    debugln("\n" + mqttPublisher(fxmode_topic, String(selectedMode)));
    switch (selectedMode)
    {
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
    if (selectedMode == 1)
    {
      debugln(" - Emergency on!");
      debugln("\n" + mqttPublisher(apiOvrOff_topic, "1"));
      apiOverrideOff = true;
    }
    else
    {
      debugln(" - Emergency off!");
      debugln("\n" + mqttPublisher(apiOvrOff_topic, "0"));
      apiOverrideOff = false;
    }

    // Reset array
    for (int i = 3; i > 0; i--)
    {
      lastModes[i] = lastModes[i - 1];
    }
    lastModes[0] = selectedMode;

    if (lastModes[0] == 1 && lastModes[1] == 4 && lastModes[2] == 1 && lastModes[3] == 4)
    {
      debugln("\n[RESET] Detected reset request!");
      debugln("\n" + mqttPublisher(reset_topic, "1"));
      for (int i = 0; i < 65535; i++)
      {
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
      ESP.restart();
    }
  }
}

// MQTT Publisher
String mqttPublisher(String topic, String payload)
{
  String retval;

  delay(50);
  bool transmSuccess = broker.publish(topic, payload);
  delay(50);

  if (transmSuccess)
  {
    retval = "[MQTT] Publish on '" + topic + "' with '" + payload + "' successful!\n";
    ledBlinkCode = 2;
  }
  else
  {
    retval = "[MQTT] Publish on '" + topic + "' with '" + payload + "' failed!\n";
    ledBlinkCode = 1;
  }

  return retval;
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

// MQTT
void mqttAliveMessage()
{
  if (((millis() > (myLastAveMsg + aveMsgIntervall)) || (myLastAveMsg == 0U)) && hostConnected) // TX
  {
    myLastAveMsg = millis();
    delay(50);
    broker.publish(alive_topic, emergency_avemsg);
    delay(50);
  }

  if (millis() > (yourlastAveMsg + aveMsgTimeout) && hostWasConnected) // RX
  {
    yourlastAveMsg = millis();
    debugln("\n[Timeout-WD] Host MQTT Client timed out!");
    debugln("\n[ESP] Restarting for new connection with host!\n");
    if (selectedMode != 3)
    ESP.restart();
  }
  else if (millis() == (yourlastAveMsg + aveMsgTimeout - 2000) && hostWasConnected)
    debugln("\n[Timeout-WD] Host MQTT Client silent -2 seconds...");
  else if (millis() == (yourlastAveMsg + aveMsgTimeout - 1000) && hostWasConnected)
    debugln("\n[Timeout-WD] Host MQTT Client silent -1 second...");
}