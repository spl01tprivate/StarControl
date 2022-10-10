/*
 * wifihandler.cpp
 *
 *  Created on: 10-10-2022
 *      Author: Sploit
 */

// *** INCLUDES ***
#include <wifihandler.h>

// *** Defines ***

// *** Variables & Objects ***

// *** Prototypes ***

// *** Functions ***
/*!
   @brief   Initialises the ws2812fx system.
            Links ESP32 RMT peripheral to pixel library.
   @return
   @note
*/
void wifi_startAP()
{
  // WiFi config & AP start
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.mode(WIFI_MODE_AP);
  String hostname = "starhost-" + String(random(100));
  WiFi.setHostname(hostname.c_str());
  WiFi.softAP(APSSID, APPSK);

  // IP
  debug("\n[WiFi] AP Host-IP: ");
  debugln(WiFi.softAPIP());

  // mDNS
  if (MDNS.begin("starhost"))
    debugln("[mDNS] Service started!");
  else
    debugln("[mDNS] Starting service failed!");

  // OTA
  ArduinoOTA.setHostname("starHost");
  ArduinoOTA.begin();
  debugln("\n[OTA] Service started!");
}

void wifi_eventHandler(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_WIFI_READY:
    debugln("\n[WiFi] Interface ready!");
    break;

  case SYSTEM_EVENT_AP_START:
    debugln("\n[WiFi] AP started!");
    break;

  case SYSTEM_EVENT_AP_STOP:
    debugln("\n[WiFi] AP stopped!");
    WiFi.softAPdisconnect();
    wifi_startAP();
    break;

  case SYSTEM_EVENT_AP_STACONNECTED:
    debugln("\n[WiFi] Station connected to AP!");
    break;

  case SYSTEM_EVENT_AP_STADISCONNECTED:
    debugln("\n[WiFi] Station disconnected from AP!");
    break;

  default:
    break;
  }
}