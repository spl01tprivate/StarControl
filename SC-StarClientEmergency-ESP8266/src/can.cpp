/*
 * can.cpp
 *
 *  Created on: 13-10-2022
 *      Author: Sploit
 */

// *** INCLUDES ***
#include <can.h>

// *** Defines ***

// *** Variables & Objects ***
// Client timeout
unsigned const int aveMsgTimeout = 10000;
const unsigned int aveMsgIntervall = 2500; // interval in which host sends can rqst to ping alive
unsigned long myLastAveMsg = 0;            // timer to send can ping rqst

// Star Emergency - API Emergency Mode
unsigned long host_lastAveMsg = 0; // stores when last alive msg was received

// Star ShiftGuidance
unsigned long shift_lastAveMsg = 0; // stores when last alive msg was received

// CAN Child
unsigned long canChild_lastAveMsg = 0; // stores when last alive msg was received

// CAN
char CAN_childMsg_alive[3] = {'C', 'A', '\0'};
char CAN_childMsg_lives[3] = {'C', 'L', '\0'};

// *** Prototypes ***
String CAN_publisher(sct_md *, byte, byte);
void CAN_sendMessage(sct_md *, unsigned long, byte, byte[]);
void CAN_aliveMessage(sct_md *);
uint8_t CAN_checkMessages(sct_md *);

// *** Functions ***
/*!
   @brief   Function to send CAN messages only giving topic and payload.
   @return  String message of operation result.
   @note
*/
String CAN_publisher(sct_md *md, byte topic, byte payload)
{
    byte txData[CAN_DLC] = {CAN_clientID, 0x1, topic, payload, 0x0, 0x0, 0x0, 0x0};
    CAN_sendMessage(md, CAN_txID, CAN_DLC, txData);

    md->ledBlinkCode = 2;

    return "[CAN] Publish on '" + String(topic) + "' with '" + String(payload) + "' successful!\n";
}

/*!
   @brief   Function to send CAN messages to the UART interface.
   @return
   @note
*/
void CAN_sendMessage(sct_md *md, unsigned long txID, byte dlc, byte payload[])
{
    if (md->connected_clients[2])
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
    }
}

/*!
   @brief   Function to send CAN messages only giving topic and payload.
   @return
   @note
*/
void CAN_aliveMessage(sct_md *md)
{
    if (((millis() > (myLastAveMsg + aveMsgIntervall)) || (myLastAveMsg == 0U)) && md->connected_clients[2])
    {
        myLastAveMsg = millis();
        byte payload[CAN_DLC] = {CAN_clientID, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0};
        CAN_sendMessage(md, CAN_txID, CAN_DLC, payload); // Sending ping rqst to clients
        Serial.print("CC!");
    }

    // Timeout StarEmergency
    if (millis() > (host_lastAveMsg + aveMsgTimeout) && md->selectedMode != 3 && md->connected_clients[0] && md->connected_clients[2])
    {
        debugln("\n[Timeout-WD] StarHost timed out!");
        esp_restart(md->selectedMode);
    }

    // Timeout ShiftGuidance
    if (millis() > (shift_lastAveMsg + aveMsgTimeout) && md->connected_clients[1])
    {
        md->connected_clients[1] = false;
        debugln("\n[Timeout-WD] StarClient ShiftGuidance timed out!");
    }

    // Timeout CANChild
    if (millis() > (canChild_lastAveMsg + aveMsgTimeout) && md->connected_clients[2])
    {
        md->connected_clients[2] = false;
        debugln("\n[Timeout-WD] CANChild timed out!");
    }
}

/*!
   @brief   Function to check for incoming CAN msgs via UART. Msgs are interpreted and further actions called.
   @return  1 - no message | 0 - message received
   @note
*/
uint8_t CAN_checkMessages(sct_md *md)
{
    if (!Serial.available())
        return 1;

    String rxMsg = Serial.readStringUntil('!');

    // debugln("[UART] RX MSG: " + String(rxMsg));

    // Preparing serial string --> getting length and converting to char ary
    unsigned int msgLength = rxMsg.length();
    char rxMsgChar[msgLength];
    rxMsg.toCharArray(rxMsgChar, msgLength + 1, 0);

    // Checking for can child readiness
    if (string_find(rxMsgChar, "CA"))
    {
        if (!md->connected_clients[2])
            debugln("[CAN] CANChild connected!");
        md->connected_clients[2] = true;
        canChild_lastAveMsg = millis();
        Serial.print("PA!");
        return 0;
    }
    else if (string_find(rxMsgChar, "CL"))
    {
        if (!md->connected_clients[2])
            debugln("[CAN] CANChild connected!");
        md->connected_clients[2] = true;
        canChild_lastAveMsg = millis();
        return 0;
    }

    // Extracting can msg id value
    char idChars[4] = {rxMsgChar[0], rxMsgChar[1], rxMsgChar[2], '\0'};
    unsigned long id = (unsigned long)strtol(idChars, 0, 16);

    if (id != 0x321) // Filter unwanted ids
    {
        debugln("[CAN] Filtering ID: " + String(id));
        return 1;
    }

    // Converting two chars into one hex value
    byte payload[CAN_DLC];
    for (int i = 0; i < CAN_DLC; i++)
    {
        char hexChars[3] = {rxMsgChar[4 + i * 3], rxMsgChar[5 + i * 3], '\0'};
        payload[i] = (byte)strtol(hexChars, 0, 16);
    }

    // Debug output
    debug("\n[CAN] RX: ID: ");
    debugHEX(id);
    debug(" | DATA: ");
    for (int i = 0; i < CAN_DLC; i++)
    {
        if ((int)payload[i] < 16)
            debug("0");
        debugHEX(payload[i]);
        debug(" ");
    }

    // Message interpretation
    if (payload[2] == CAN_tpc_reset)
    {
        if (payload[0] == 0x0 && payload[1] == 0x1 && payload[3] == 0x1)
        {
            debugln("\n*****************************************");
            debugln("\n[RESET] Restarting at your wish master ;)");
            debugln("\n*****************************************");
            Serial.print("CR!");
            esp_restart(md->selectedMode);
        }
        else
            debugln("[CAN] ERR Topic Reset: Unknown message!");
    }
    else if (payload[2] == CAN_tpc_ping)
    {
        if (payload[1] == 0x0) // Request
        {
            debugln("\n[CAN] Message: Ping request");
            byte txPL[CAN_DLC] = {CAN_clientID, 0x1, CAN_tpc_ping, 0x1, 0x0, 0x0, 0x0, 0x0};
            CAN_sendMessage(md, CAN_txID, CAN_DLC, txPL);
        }
        else if (payload[1] == 0x1) // Answer
        {
            if (payload[0] == 0x0 && payload[1] == 0x1 && payload[3] == 0x1)
            {
                debugln("\n[CAN] Message: Ping from StarHost");
                if (!md->connected_clients[0])
                {
                    debugln("[Client] StarHost connected! Sending current data...");
                    if (md->selectedMode == 1)
                        // debugln("\n" + CAN_publisher(CAN_tpc_apiovr, 1));
                        CAN_publisher(md, CAN_tpc_apiovr, 1);
                    else
                        // debugln("\n" + CAN_publisher(CAN_tpc_apiovr, 0));
                        CAN_publisher(md, CAN_tpc_apiovr, 0);
                    // debugln("\n" + CAN_publisher(CAN_tpc_selMode, (byte)selectedMode));
                    CAN_publisher(md, CAN_tpc_selMode, (byte)md->selectedMode);
                }
                host_lastAveMsg = millis();
                md->connected_clients[0] = true;
            }
            else if (payload[2] == 0x1 && payload[1] == 0x1 && payload[3] == 0x1)
            {
                debugln("\n[CAN] Message: Ping from StarShiftGuidance");
                if (!md->connected_clients[1])
                    debugln("[Client] ShiftGuidance connected!");
                shift_lastAveMsg = millis();
                md->connected_clients[1] = true;
            }
            else
                debugln("[CAN] ERR Topic Ping: Unknown client!");
        }
        else
            debugln("[CAN] ERR Topic Ping: Unknown message!");
    }
    else
    {
        debugln("\n[CAN] ERR Not a subscribed topic!");
    }

    return 0;
}