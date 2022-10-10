/*
 * led.cpp
 *
 *  Created on: 09-10-2022
 *      Author: Sploit
 */

// *** INCLUDES ***
#include <led.h>

// *** Defines ***

// *** Variables & Objects ***

// *** Prototypes ***
void led_init();
void led_update();
void led_test();

// *** Functions ***
/*!
   @brief   Initialises the ws2812fx system.
            Links ESP32 RMT peripheral to pixel library.
   @return
   @note
*/
void led_init()
{
    debugln("[LED] RMT hardware initialised!");
}

/*!
   @brief   Updates the ws2812 leds.
   @return
   @note
*/
void led_update()
{

}

/*!
   @brief   Test
   @return
   @note
*/
void led_test()
{
    
}