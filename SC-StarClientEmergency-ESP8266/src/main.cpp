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
#include <inputs.h>
#include <ledbi.h>
#include <statuspixel.h>

//***** DEFINES *****

//***** VARIABLES & OBJECTS *****
// Maindata struct
sct_md maindata_struct;
sct_md *md = &maindata_struct;

//***** PROTOTYPES *****
void setup();
void loop();
void handlers();

//***** SETUP *****
void setup()
{
  Serial.begin(38400);
  while (!Serial)
    ;

  debugln("\n[StarClient-Emergency] Starting programm ~ by spl01t*#7");
  debugln("[StarClient-Emergency] You are running version " + String(VERSION) + "!");

  // IOs
  pinMode(PIN_mode1, INPUT_PULLUP);
  pinMode(PIN_mode2, INPUT_PULLUP);
  pinMode(PIN_mode3, INPUT_PULLUP);
  pinMode(PIN_mode4, INPUT_PULLUP);
  pinMode(PIN_LBI, OUTPUT);

  // WiFi
  WiFi.setOutputPower(20.5);
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(APSSID, APPSK))
  {
    debug("\n[WiFi] AP Host-IP: ");
    debugln(WiFi.softAPIP());
  }

  // OTA
  ArduinoOTA.setHostname("starEmergency");
  ArduinoOTA.begin();
  debugln("\n[OTA] Service started!");

  // Status pixel
  stapi_init();
  stapi_startanimation();

  debugln("\n[StarControl-Client] Initialization completed...starting programm loop!\n");
}

//***** LOOP *****
void loop()
{
  handlers();

  stapi_update(md->selectedMode, md->connected_clients[0], md->connected_clients[2]);

  ledbi_update(md->ledBlinkCode);

  readInputs(md);

  interpretInputs(md);

  CAN_aliveMessage(md);

  CAN_checkMessages(md);
}

//***** FUNCTIONS *****
// Handler Functions
void handlers()
{
  ArduinoOTA.handle();
  yield();
}
