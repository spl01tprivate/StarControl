#ifndef STATUSPIXEL_h
#define STATUSPIXEL_h

#include <maindata.h>
#include <Adafruit_NeoPixel.h>

void stapi_init();
void stapi_startanimation();
void stapi_update(uint8_t, bool, bool);

#endif