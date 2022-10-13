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
#include <can.h>
#include <ledbi.h>
#include <statuspixel.h>

//***** DEFINES *****

//***** VARIABLES & OBJECTS *****
// Maindata struct
sct_md maindata_struct;
sct_md *md = &maindata_struct;

// Modes & Restrictions
unsigned int selectedModeBefore = -1;
unsigned long modeChangeTime = 0;
const unsigned long modeChangeTimeThres = 350;

bool apiOverrideOff;

// Reset detection
unsigned int lastModes[4] = {0, 0, 0, 0};

//***** PROTOTYPES *****
void setup();
void loop();
void handlers();
void readInputs(sct_md *);
void interpretInputs(sct_md *);

//***** SETUP *****
void setup()
{
  Serial.begin(38400);
  while (!Serial)
    ;

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

// Read Inputs
void readInputs(sct_md * md)
{
  if (!digitalRead(mode1Pin) && digitalRead(mode2Pin) && digitalRead(mode3Pin) && digitalRead(mode4Pin) && md->selectedMode != 1)
  {
    // Mode 1 - All off
    md->selectedMode = 1;
    modeChangeTime = millis();
  }
  else if (digitalRead(mode1Pin) && !digitalRead(mode2Pin) && digitalRead(mode3Pin) && digitalRead(mode4Pin) && md->selectedMode != 2)
  {
    // Mode 2 - Star on but listens to motor restriction - UGLW on with resting motor
    md->selectedMode = 2;
    modeChangeTime = millis();
  }
  else if (digitalRead(mode1Pin) && digitalRead(mode2Pin) && !digitalRead(mode3Pin) && digitalRead(mode4Pin) && md->selectedMode != 3)
  {
    // Mode 3 - Star strict on - UGLW fav fx mode strict on
    md->selectedMode = 3;
    modeChangeTime = millis();
  }
  else if (digitalRead(mode1Pin) && digitalRead(mode2Pin) && digitalRead(mode3Pin) && !digitalRead(mode4Pin) && md->selectedMode != 4)
  {
    md->selectedMode = 4;
    modeChangeTime = millis();
  }
}

void interpretInputs(sct_md * md)
{
  // Added some delay between deciding and transmitting which mode is selected to prevent quick and dirty switches from one mode to another -> can be conflicting with WS2812FX and also does not look good
  if (selectedModeBefore != md->selectedMode && millis() > (modeChangeTime + modeChangeTimeThres))
  {
    selectedModeBefore = md->selectedMode;
    // debugln("\n" + CAN_publisher(CAN_tpc_selMode, (byte)selectedMode));
    CAN_publisher(md, CAN_tpc_selMode, (byte)md->selectedMode);
    switch (md->selectedMode)
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
    if (md->selectedMode == 1)
    {
      debugln(" - Emergency on!");
      // debugln("\n" + CAN_publisher(CAN_tpc_apiovr, 1));
      CAN_publisher(md, CAN_tpc_apiovr, 1);
      apiOverrideOff = true;
    }
    else
    {
      debugln(" - Emergency off!");
      // debugln("\n" + CAN_publisher(CAN_tpc_apiovr, 0));
      CAN_publisher(md, CAN_tpc_apiovr, 0);
      apiOverrideOff = false;
    }

    // Reset array
    for (int i = 3; i > 0; i--)
    {
      lastModes[i] = lastModes[i - 1];
    }
    lastModes[0] = md->selectedMode;

    if (lastModes[0] == 1 && lastModes[1] == 4 && lastModes[2] == 1 && lastModes[3] == 4)
    {
      debugln("\n[RESET] Detected reset request!");
      // debugln("\n" + CAN_publisher(CAN_tpc_reset, 1));
      CAN_publisher(md, CAN_tpc_reset, 1);
      Serial.print("CR!");
      delay(100);
      WiFi.mode(WIFI_OFF);
      delay(100);
      debugln("\n*****************************************");
      debugln("\n[RESET] Restarting at your wish master ;)");
      debugln("\n*****************************************");
      esp_restart(md->selectedMode);
    }
  }
}


