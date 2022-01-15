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

  --- Bugs ---
  ~ No known bugs
*/

//***** INCLUDES *****
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include "uMQTTBroker.h"

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
#define strobe_topic "fxmode/strobe"
#define rainbow_topic "fxmode/rainbow"

#define emergency_avemsg "emeg_ave"
#define starhost_avemsg "host_ave"

//***** VARIABLES & OBJECTS *****
// WiFi Variables
IPAddress local_IP = IPAddress(192, 168, 4, 10);
IPAddress gateway = IPAddress(192, 168, 4, 1);
IPAddress subnet = IPAddress(255, 255, 255, 0);

// Modes & Restrictions
unsigned int selectedMode = -1;

bool apiOverrideOff;

int ledBlinkCode = 0;
int ledBlinkCounter = 0;
bool ledBlinkInProgress = false;
unsigned long ledBlinkLastBlink = 0;

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
    if (client_id == "starhost1")
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
        {
          debugln("\n" + mqttPublisher(apiOvrOff_topic, "1"));
        }
        else if (selectedMode == 2)
        {
          debugln("\n" + mqttPublisher(apiOvrOff_topic, "0"));
          debugln("\n" + mqttPublisher(rainbow_topic, "0"));
        }
        else if (selectedMode == 3)
        {
          debugln("\n" + mqttPublisher(apiOvrOff_topic, "0"));
          debugln("\n" + mqttPublisher(strobe_topic, "0"));
          debugln("\n" + mqttPublisher(rainbow_topic, "1"));
        }
        else if (selectedMode == 4)
        {
          debugln("\n" + mqttPublisher(apiOvrOff_topic, "0"));
          debugln("\n" + mqttPublisher(strobe_topic, "1"));
        }
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

//***** PROTOTYPES *****
void setup();
void loop();
void handlers();
void readInputs();
void ledBlink();
void ledBlinkError();
void ledBlinkSuccess();
void initLastState();
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

  // Get EEPROM memory
  initLastState();

  // WiFi
  WiFi.disconnect();
  delay(100);
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

  debugln("\n[StarControl-Client] Initialization completed...starting programm loop!\n");
}

//***** LOOP *****
void loop()
{
  handlers();

  ledBlink();

  readInputs();

  mqttAliveMessage();
}

//***** FUNCTIONS *****
// Handler Functions
void handlers()
{
  ArduinoOTA.handle();
  yield();
}

// Read Inputs
void readInputs()
{
  if (!digitalRead(mode1Pin) && digitalRead(mode2Pin) && digitalRead(mode3Pin) && digitalRead(mode4Pin) && selectedMode != 1)
  {
    // Mode 1 - All off
    debugln("[INPUT] Mode 1 was selected!");
    apiOverrideOff = true;
    debugln("\n" + mqttPublisher(apiOvrOff_topic, "1"));
    EEPROM.write(apiOverrideOffAdress, 1);
    EEPROM.commit();
    selectedMode = 1;
  }
  else if (digitalRead(mode1Pin) && !digitalRead(mode2Pin) && digitalRead(mode3Pin) && digitalRead(mode4Pin) && selectedMode != 2)
  {
    // Mode 2 - Star on but listens to motor restriction - UGLW on with resting motor
    debugln("[INPUT] Mode 2 was selected!");
    apiOverrideOff = false;
    if (selectedMode == 1)
      debugln("\n" + mqttPublisher(apiOvrOff_topic, "0"));
    else if (selectedMode == 3)
      debugln("\n" + mqttPublisher(rainbow_topic, "0"));
    EEPROM.write(apiOverrideOffAdress, 0);
    EEPROM.commit();
    selectedMode = 2;
  }
  else if (digitalRead(mode1Pin) && digitalRead(mode2Pin) && !digitalRead(mode3Pin) && digitalRead(mode4Pin) && selectedMode != 3)
  {
    // Mode 3 - Star strict on - UGLW rainbow strict on
    debugln("[INPUT] Mode 3 was selected!");
    if (selectedMode == 4)
      debugln("\n" + mqttPublisher(strobe_topic, "0"));
    else if (selectedMode == 2)
      debugln("\n" + mqttPublisher(rainbow_topic, "1"));
    selectedMode = 3;
  }
  else if (digitalRead(mode1Pin) && digitalRead(mode2Pin) && digitalRead(mode3Pin) && !digitalRead(mode4Pin) && selectedMode != 4)
  {
    // Mode 4 - Strobe both strict on
    debugln("[INPUT] Mode 4 was selected!");
    debugln("\n" + mqttPublisher(strobe_topic, "1"));
    selectedMode = 4;
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

// EEPROM
void initLastState()
{
  debugln("\n[EEPROM] Starting data extratction!");

  // API Override Lights Off
  int apiOverrideOffcontent = int(EEPROM.read(apiOverrideOffAdress));

  debugln("[EEPROM] API Override Lights Off: " + String(apiOverrideOffcontent));

  if (apiOverrideOffcontent == 1)
  {
    apiOverrideOff = true;
  }
  else if (apiOverrideOffcontent == 0)
  {
    apiOverrideOff = false;
  }
  else // if no logic state
  {
    debugln("[EEPROM Reading] EEPROM Reading no valid option - Setting: API Override Lights Off");
    EEPROM.write(apiOverrideOffAdress, 1); // then write override to turn lights off
    apiOverrideOff = true;
  }

  EEPROM.commit();

  debugln("[EEPROM] Extraction completed!");
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

  if (millis() > (yourlastAveMsg + aveMsgTimeout) && hostWasConnected) //RX
  {
    yourlastAveMsg = millis();
    debugln("\n[Timeout-WD] Host MQTT Client timed out!");
    debugln("\n[ESP] Restarting for new connection with host!\n");
    ESP.restart();
  }
  else if (millis() == (yourlastAveMsg + aveMsgTimeout - 2000) && hostWasConnected)
    debugln("\n[Timeout-WD] Host MQTT Client silent -2 seconds...");
  else if (millis() == (yourlastAveMsg + aveMsgTimeout - 1000) && hostWasConnected)
    debugln("\n[Timeout-WD] Host MQTT Client silent -1 second...");
}