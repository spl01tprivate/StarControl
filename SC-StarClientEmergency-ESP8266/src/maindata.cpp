/*
 * maindata.cpp
 *
 *  Created on: 10-10-2022
 *      Author: Sploit
 */

// *** INCLUDES ***
#include <maindata.h>

// *** Defines ***

// *** Variables & Objects ***

// *** Prototypes ***
bool string_find(char *, char *);

// *** Functions ***
/*!
   @brief   Search function to find string in string.
   @return  True if substring was found in string.
   @note
*/
bool string_find(char *haystack, char *needle)
{
    int compareOffset = 0;
    while (*haystack)
    {
        if (*haystack == *needle)
        {
            compareOffset = 0;
            while (haystack[compareOffset] == needle[compareOffset])
            {
                compareOffset++;
                if (needle[compareOffset] == '\0')
                {
                    return true;
                }
            }
        }
        haystack++;
    }
    return false;
}

/*!
   @brief   Restart function. Checks whether mode 3 is not selected.
            Otherwise ESP wont restart. Use mode 3 for OTA updates.
   @return
   @note
*/
void esp_restart(uint8_t selMode)
{
    if (selMode != 3)
        ESP.restart();
}