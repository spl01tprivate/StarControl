#ifndef MAINDATA_h
#define MAINDATA_h

#include <Arduino.h>

//***** DEFINES *****
// Version
#define VERSION 2.01

// Debug
#define DEBUG 1
#define ISRDEBUG 0

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

#if ISRDEBUG == 1
#define ISRln(x) Serial.println(x)
#else
#define ISRln(x)
#endif

// WiFi
#define APSSID "StarHost"
#define APPSK "starnetwork"

// EEPROM
#define starAdress 0
#define betriebsmodusAdress 1
#define fademodusAdress 2
#define tflAdress 3
#define tflModeAdress 4
#define starFadePauseAdress 5
#define tflFadePauseAdress 6
#define strobeAdress 7
#define starMotorRestrictionAdress 8
#define apiOverrideOffAdress 9
#define modeAdress 10
#define color1_1Adress 11 // 0x0000FF - Blue
#define color1_2Adress 12 // 0x00FF00 - Green
#define color1_3Adress 13 // 0xFF0000 - Red
#define brtnsAdress 14
#define speed1Adress 15 // 0x00FF
#define speed2Adress 16 // 0xFF00 - 0 to 65535
#define uglwMotorRestrictionAdress 17
#define batVoltThresAdress 18
#define batVoltOffsetAdress 22
#define favoriteModeAdress 26
#define favoriteColorAdress1_1 27
#define favoriteColorAdress1_2 28
#define favoriteColorAdress1_3 29
#define favoriteBrtnsAdress 30
#define favoriteSpeedAdress1 31
#define favoriteSpeedAdress2 32
#define uglwTFLRestrictionAdress 33
#define transitionCoefficientAdress 34
#define batVoltOffsetMotorAdress 38
#define color2_1Adress 42
#define color2_2Adress 43
#define color2_3Adress 44
#define color3_1Adress 45
#define color3_2Adress 46
#define color3_3Adress 47
#define favoriteColorAdress2_1 48
#define favoriteColorAdress2_2 49
#define favoriteColorAdress2_3 50
#define favoriteColorAdress3_1 51
#define favoriteColorAdress3_2 52
#define favoriteColorAdress3_3 53
#define fadeSizeAdress 54
#define favoriteFadeSizeAdress 55
// continue at adress 56

// Inputs
#define pin_tfl 14     // D5 - Tagfahrlicht             - Type: PWM
#define pin_kl15 35    // D6 - ZST 2                    - Type: Digital
#define pin_kl50 36    // D7 - Starter mit Impuls       - Type: Digital
#define pin_batVolt 34 // xx - Battery Voltage          - Type: ADC Input

// Outputs
#define pin_tflLeft 25       // D1 - TFL Links MOSFET (Weiß)  - Type: PWM
#define pin_tflRight 26       // D2 - TFL Rechts MOSFET (Blau) - Type: PWM
#define pin_star 32       // D8 - Star MOSFET (Lila)       - Type: PWM
#define pin_tflLeftRelais 18 // D3 - TFL Links Relais         - Type: Digital
#define pin_tflRightRelais 19 // D4 - TFL Rechts Relais        - Type: Digital
#define pin_starRelais 33 // D0 - Star Relais              - Type: Digital
#define pin_led 17

// Fade-Pause Timings
#define fadePauseFast 1
#define fadePauseNormal 4
#define fadePauseSlow 6

// Strobe Timings & Cycles
#define strobePause 50
#define strobeShortCycles 20
#define LED_MODE_STROBE 1
#define LED_SPEED_STROBE 120

// TFL dimmed treshold
#define tflDimTreshold 512

// UART topics
#define status_topic "status"
#define apiOvrOff_topic "api/ovroff"
#define alive_topic "alive"
#define mode_topic "mode"
#define color_topic "color"
#define brtns_topic "brtns"
#define speed_topic "speed"
#define fxmode_topic "fxmode"
#define motor_topic "motor"
#define transitionCoefficient_topic "transCoef"
#define reset_topic "reset"
#define fadeSize_topic "fadesize"

#define emergency_avemsg "emeg_ave"
#define starhost_avemsg "host_ave"

// CAN topics
#define CAN_txID 0x321
#define CAN_DLC 8
#define CAN_clientID 0x0

#define CAN_tpc_ping 0x0
#define CAN_tpc_selMode 0x1
#define CAN_tpc_apiovr 0x2
#define CAN_tpc_reset 0x3

// WS2812FX
#define LED_COUNT 595

// User Data
struct userDataStruct
{
  bool lol = false;
};

#endif