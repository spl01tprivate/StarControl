#ifndef MAINDATA_h
#define MAINDATA_h

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>

//***** DEFINES *****
// Version
#define VERSION 2.01

// Debug
#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugHEX(x) Serial.print(x, HEX)
#define debugDEC(x) Serial.print(x, DEC)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugHEX(x)
#define debugDEC(x)
#define debugln(x)
#endif

// WiFi
#define APSSID "StarEmergency"
#define APPSK "starnetwork"

// Inputs
#define mode1Pin 5  // D1
#define mode2Pin 4  // D2
#define mode3Pin 14 // D5
#define mode4Pin 12 // D6

// WS2812FX
#define LED_PIN 13 // D7
#define LED_COUNT 1
#define LED_BRTNS 48 // Maximum brtns of status pixel

// CAN topics
#define CAN_txID 0x321
#define CAN_DLC 8
#define CAN_clientID 0x1

#define CAN_tpc_ping 0x0
#define CAN_tpc_selMode 0x1
#define CAN_tpc_apiovr 0x2
#define CAN_tpc_reset 0x3

// User Data
struct sct_md
{
  // Modes & Restrictions
  unsigned int selectedMode = -1;
  unsigned int selectedModeBefore = -1;

  bool connected_clients[3] = {false, false, false}; // 0 - Host | 1 - Shift | 2 - CANChild
  
  uint8_t ledBlinkCode = 0;
};

// ***** Prototypes *****
bool string_find(char *, char *);
void esp_restart(uint8_t);

#endif