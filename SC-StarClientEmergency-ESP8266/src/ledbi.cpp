/*
 * ledbi.cpp
 *
 *  Created on: 10-10-2022
 *      Author: Sploit
 */

// *** INCLUDES ***
#include <ledbi.h>

// *** Defines ***

// *** Variables & Objects ***
uint8_t ledBlinkCounter = 0;
bool ledBlinkInProgress = false;
unsigned long ledBlinkLastBlink = 0;

// *** Prototypes ***
void ledbi_update(uint8_t);
void ledbi_blinkerror(uint8_t);
void ledbi_blinksuccess(uint8_t);

// *** Functions ***
/*!
   @brief   Updates the blink effects on the builtin led.
   @return
   @note
*/
void ledbi_update(uint8_t ledBlinkCode)
{
   if (ledBlinkCode != 0)
   {
      switch (ledBlinkCode)
      {
      case 1:
         ledbi_blinkerror(ledBlinkCode);
         break;

      case 2:
         ledbi_blinksuccess(ledBlinkCode);
         break;

      default:
         break;
      }
   }
}

/*!
   @brief   Fast LBI blinking to indicate error.
   @return
   @note
*/
void ledbi_blinkerror(uint8_t ledBlinkCode)
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

/*!
   @brief   Slow LBI blinking to indicate success.
   @return
   @note
*/
void ledbi_blinksuccess(uint8_t ledBlinkCode)
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
