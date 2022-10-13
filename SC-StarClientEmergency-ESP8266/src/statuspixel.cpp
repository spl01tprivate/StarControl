/*
 * statuspixel.cpp
 *
 *  Created on: 10-10-2022
 *      Author: Sploit
 */

// *** INCLUDES ***
#include <statuspixel.h>

// *** Defines ***

// *** Variables & Objects ***
Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// *** Prototypes ***
void stapi_init();
void stapi_startanimation();
void stapi_update(uint8_t, bool, bool);

// *** Functions ***
/*!
   @brief   Initializes the status pixel library.
   @return
   @note
*/
void stapi_init()
{
    leds.begin();
    leds.clear();
}

// *** Functions ***
/*!
   @brief   Start animation at boot of mcu.
   @return
   @note
*/
void stapi_startanimation()
{
    for (int i = 0; i < 65535; i++)
    {
        yield();
        leds.setPixelColor(0, leds.ColorHSV(i));
        leds.show();
        i += (65535 / 1250);
        delay(1);
    }
}

void stapi_update(uint8_t selMode, bool hostConnected, bool canConnected)
{
    static unsigned long timer = 0;

    if (selMode == 1 || selMode == 2)
    {
        bool red = false;
        bool green = false;
        bool blue = true;

        if (selMode == 1)
            red = true;
        else
            red = false;

        if (hostConnected)
            green = true;

        uint32_t color = 0;
        if (red)
            color += 0xFF0000;
        if (green)
            color += 0xFF00;
        if (blue && !red && !green)
            color += 0xFF;

        if (leds.getPixelColor(0) != color && canConnected)
        {
            leds.setPixelColor(0, color);
            leds.setBrightness(LED_BRTNS);
            leds.show();
        }
        else if (!canConnected)
        {
            if (millis() > timer + 500)
            {
                timer = millis();
                if (leds.getPixelColor(0) > 0)
                    leds.setPixelColor(0, 0);
                else
                    leds.setPixelColor(0, color);
                leds.show();
            }
        }
    }
    else if (selMode == 3 && millis() > timer + 50)
    {
        timer = millis();
        static uint16_t colorWheel = 0;
        leds.setPixelColor(0, leds.ColorHSV(colorWheel));
        leds.show();
        colorWheel += 750;
        if (colorWheel > 65535)
            colorWheel = 0;
    }
    else if (selMode == 4 && millis() > timer + 50)
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