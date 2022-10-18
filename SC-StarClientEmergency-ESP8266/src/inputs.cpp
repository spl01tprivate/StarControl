/*
 * inputs.cpp
 *
 *  Created on: 18-10-2022
 *      Author: Sploit
 */

// *** INCLUDES ***
#include <inputs.h>

// *** Defines ***

// *** Variables & Objects ***
// Modes & Restrictions
unsigned int selectedModeBefore = -1;
unsigned long modeChangeTime = 0;
const unsigned long modeChangeTimeThres = 350;

bool apiOverrideOff;

// Reset detection
unsigned int lastModes[4] = {0, 0, 0, 0};

// *** Prototypes ***
void readInputs(sct_md *md);
void interpretInputs(sct_md *md);

// *** Functions ***
/*!
   @brief   Function to check the input pins and select the current mode.
   @return
   @note
*/
void readInputs(sct_md *md)
{
  if (!digitalRead(PIN_mode1) && digitalRead(PIN_mode2) && digitalRead(PIN_mode3) && digitalRead(PIN_mode4) && md->selectedMode != 1)
  {
    // Mode 1 - All off
    md->selectedMode = 1;
    modeChangeTime = millis();
  }
  else if (digitalRead(PIN_mode1) && !digitalRead(PIN_mode2) && digitalRead(PIN_mode3) && digitalRead(PIN_mode4) && md->selectedMode != 2)
  {
    // Mode 2 - Star on but listens to motor restriction - UGLW on with resting motor
    md->selectedMode = 2;
    modeChangeTime = millis();
  }
  else if (digitalRead(PIN_mode1) && digitalRead(PIN_mode2) && !digitalRead(PIN_mode3) && digitalRead(PIN_mode4) && md->selectedMode != 3)
  {
    // Mode 3 - Star strict on - UGLW fav fx mode strict on
    md->selectedMode = 3;
    modeChangeTime = millis();
  }
  else if (digitalRead(PIN_mode1) && digitalRead(PIN_mode2) && digitalRead(PIN_mode3) && !digitalRead(PIN_mode4) && md->selectedMode != 4)
  {
    md->selectedMode = 4;
    modeChangeTime = millis();
  }
}

/*!
   @brief   Function to check the current mode and handle corresponding. Also checking for restart sequence.
   @return
   @note
*/
void interpretInputs(sct_md *md)
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