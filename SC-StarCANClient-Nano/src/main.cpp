// ***---*** Project - StarControl CanBus Client - by Sploit ***---***

// *** FILE - MAIN ***

// ***---*** Versions ***---***
/*--- Changelog ---
  02/10/22 - V1.0
  ~ Started can communication extension

  --- Bugs ---
  ~ No known bugs
*/

// ***---*** Description ***---***
/*
    This code module is written to run on the Arduino Nanos which are connected with a MCP2515 canbus module.
    The Nanos receive and transmit incoming canbus data via their UART interface to the parent ESPs.
*/

// ***** INCLUDES *****
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

// ***** DEFINES *****
// Version
#define VERSION 1.0

// Debug
#define DEBUG 0

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

// Pins
#define PIN_LBI LED_BUILTIN

// CAN communication
#define CAN_DLC 8

// ***** VARIABLES & OBJECTS *****
// CAN Object
MCP_CAN CAN0(10); // Set CAN0 CS to pin 10

// Parent timeout
const uint16_t aveMsgTimeout = 10000;
unsigned long parent_lastAveMsg = 0;

// ***** PROTOTYPES *****
void (*resetFunc)(void) = 0; // Reset Function
void parentComInit();
void writeCAN(unsigned long, byte, byte[]);
uint8_t readCAN();
void writeUART(unsigned long, byte, byte[]);
uint8_t readUART();
uint8_t blinkStatusLED(uint8_t, uint16_t);
uint8_t timeoutHandler();

// ***** SETUP *****
void setup()
{
  Serial.begin(38400);
  while (!Serial)
    ;

  pinMode(PIN_LBI, OUTPUT);
  digitalWrite(PIN_LBI, LOW);

  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
  {
    debugln("MCP2515 Initialized Successfully!");
    blinkStatusLED(3, 75);
    digitalWrite(PIN_LBI, LOW);
  }
  else
  {
    debugln("Error Initializing MCP2515!");
    blinkStatusLED(5, 150);
    resetFunc();
  }

  CAN0.init_Mask(0,0,0x03210000);
  CAN0.init_Mask(1,0,0x03210000);
  CAN0.init_Filt(0,0,0x03210000);
  CAN0.setMode(MCP_NORMAL);

  debugln("\n[StarCANClient] Initialization completed...\n");

  parentComInit();
}

// ***** LOOP *****
void loop()
{
  readUART();

  readCAN();

  timeoutHandler();
}

// ***** FUNCTIONS *****
void parentComInit()
{
  debugln("[COMINIT] Waiting for parent to come alive...!");

  bool parent_alive = false;
  while (!parent_alive)
  {
    Serial.print("CA!");
    if (Serial.available())
    {
      String payload = Serial.readStringUntil('!');
      if (payload == "PA")
        parent_alive = true;
    }
    delay(100);
  }

  parent_lastAveMsg = millis();

  debugln("[COMINIT] Parent alive...starting loop!");
}

uint8_t readUART()
{
  if (!Serial.available())
    return 1;

  String rxMsg = Serial.readStringUntil('!');

  // debugln("[UART] RX MSG: " + String(rxMsg));

  // Checking for parent readiness
  if (rxMsg == "CC")
  {
    Serial.print("CL!");
    parent_lastAveMsg = millis();
    return 0;
  }
  else if (rxMsg == "CR")
    resetFunc();

  // Preparing serial string --> getting length and converting to char ary
  unsigned int msgLength = rxMsg.length();
  char rxMsgChar[msgLength];
  rxMsg.toCharArray(rxMsgChar, msgLength + 1, 0);

  // Extracting can msg id value
  char idChars[4] = {rxMsgChar[0], rxMsgChar[1], rxMsgChar[2], '\0'};
  unsigned long id = (unsigned long)strtol(idChars, 0, 16);

  if (id != 0x321)
  {
    debugln("[UART] CAN ID other than 321 - sending no can message!");
    return 1;
  }

  // Converting two chars into one hex value
  byte txBuf[CAN_DLC];
  for (int i = 0; i < CAN_DLC; i++)
  {
    char hexChars[3] = {rxMsgChar[4 + i * 3], rxMsgChar[5 + i * 3], '\0'};
    txBuf[i] = (byte)strtol(hexChars, 0, 16);
  }

  // Debug output
  // debug("\n[UART] RX: ID: ");
  // debugHEX(id);
  // debug(" | DATA: ");
  // for (int i = 0; i < CAN_DLC; i++)
  // {
  //   if ((int)txBuf[i] < 16)
  //     debug("0");
  //   debugHEX(txBuf[i]);
  //   debug(" ");
  // }

  writeCAN(id, CAN_DLC, txBuf);

  return 0;
}

uint8_t readCAN()
{
  if (CAN0.checkReceive() == CAN_NOMSG)
    return 1;

  unsigned long rxID = 0;
  byte dlc = 0;
  byte rxBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  if (CAN0.readMsgBuf(&rxID, &dlc, rxBuf) == CAN_OK)
  {
    digitalWrite(PIN_LBI, HIGH);

    debug("[CAN] RX: ID: ");
    debugHEX(rxID);
    debug(" | DLC: ");
    debugDEC(dlc);
    debug(" | DATA: ");
    for (byte i = 0; i < dlc; i++) // print the data
    {
      debugHEX(rxBuf[i]);
      debug(" ");
    }
    debugln("!");

    writeUART(rxID, dlc, rxBuf);
  }
  else
    return 1;

  return 0;
}

void writeUART(unsigned long txID, byte dlc, byte payload[])
{
  Serial.print(txID, HEX);
  for (int i = 0; i < (int)dlc; i++)
  {
    Serial.print(',');
    if (payload[i] < 16)
      Serial.print("0");
    Serial.print(payload[i], HEX);
  }
  Serial.print('!');

  debugln("[UART] Message sent successfully!");
  digitalWrite(PIN_LBI, HIGH);
}

void writeCAN(unsigned long txID, byte dlc, byte payload[])
{
  debugln("[CAN] TX: " + String(txID) + " | DLC: " + String(dlc) + " | DATA: " + String(payload[0]) + " " + String(payload[1]) + " " + String(payload[2]) + " " + String(payload[3]) + " " + String(payload[4]) + " " + String(payload[5]) + " " + String(payload[6]) + " " + String(payload[7]));
  uint8_t resCode = CAN0.sendMsgBuf(txID, dlc, payload);
  if (resCode == CAN_OK)
  {
    debugln("[CAN] Message sent successfully!");
    digitalWrite(PIN_LBI, HIGH);
  }
  else
  {
    debugln("[CAN] Error sending message: " + String(resCode) + "!");
    digitalWrite(PIN_LBI, LOW);
  }
}

uint8_t blinkStatusLED(uint8_t repeats, uint16_t delayTime)
{
  for (uint8_t i = 0; i < repeats; i++)
  {
    digitalWrite(PIN_LBI, HIGH);
    delay(delayTime);
    digitalWrite(PIN_LBI, LOW);
    delay(delayTime);
  }

  return 0;
}

uint8_t timeoutHandler()
{
  if (millis() > parent_lastAveMsg + aveMsgTimeout)
  {
    debugln("[UART] Parent timed out!");
    parentComInit();
  }
}