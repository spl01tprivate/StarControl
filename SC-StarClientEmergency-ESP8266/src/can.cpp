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
const unsigned int CAN_pingCC_interval = 2500; // interval in which host sends can rqst to ping alive
unsigned long CAN_pingCC_lastMsg = 0;          // timer to send can ping rqst

// Star Emergency - API Emergency Mode
unsigned long host_lastAveMsg = 0; // stores when last alive msg was received

// Star ShiftGuidance
unsigned long shift_lastAveMsg = 0; // stores when last alive msg was received

// CAN Child
unsigned long canChild_lastAveMsg = 0; // stores when last alive msg was received

// CAN
char CAN_childMsg_alive[3] = {'C', 'A', '\0'};
char CAN_childMsg_lives[3] = {'C', 'L', '\0'};

// CAN Ping
unsigned long CAN_ping_lastMsg = 0;
const uint16_t CAN_ping_interval = 5000;

// ACK buffer
const uint8_t CAN_ackBuf_size = 5; // Size of ack buffer to store can messages
int CAN_ackBuf_txID[CAN_ackBuf_size];
byte CAN_ackBuf_dlc[CAN_ackBuf_size];
byte CAN_ackBuf_payload[CAN_ackBuf_size][CAN_DLC];
uint8_t CAN_ackBuf_index = 0;

// ACK timings
const uint16_t CAN_ack_msgTimeout = 1000;
unsigned long CAN_ack_sentTime[CAN_ackBuf_size];

// *** Prototypes ***
String CAN_publisher(sct_md *, byte, byte);
void CAN_ackBuf_remElement(int8_t);
bool CAN_ackTimeout(sct_md *);
bool CAN_ackMessage(sct_md *, int, byte, byte[]);
bool CAN_queueAckMessage(unsigned long, byte, byte[]);
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
   @brief   Removes buffer element.
   @return
   @note
*/
void CAN_ackBuf_remElement(int8_t data_match)
{
    // Resort buffer data if match was in between start and end of buffer elements
    if (CAN_ackBuf_index == 2 && data_match == 0) // first of two elements get removed
    {
        CAN_ackBuf_txID[0] = CAN_ackBuf_txID[1];
        CAN_ackBuf_dlc[0] = CAN_ackBuf_dlc[1];
        for (int i = 0; i < CAN_ackBuf_dlc[1]; i++)
        {
            CAN_ackBuf_payload[0][i] = CAN_ackBuf_payload[1][i];
        }
        CAN_ack_sentTime[0] = CAN_ack_sentTime[1];
        debugln("[CAN] Buffer: Removed first element of two!");
    }
    else if (CAN_ackBuf_index > 2 && data_match < (CAN_ackBuf_index - 1)) // element of deletion is not last element in buffer
    {
        CAN_ackBuf_txID[data_match] = CAN_ackBuf_txID[CAN_ackBuf_index - 1];
        CAN_ackBuf_dlc[data_match] = CAN_ackBuf_dlc[CAN_ackBuf_index - 1];
        for (int i = 0; i < CAN_ackBuf_dlc[1]; i++)
        {
            CAN_ackBuf_payload[data_match][i] = CAN_ackBuf_payload[CAN_ackBuf_index - 1][i];
        }
        CAN_ack_sentTime[data_match] = CAN_ack_sentTime[CAN_ackBuf_index - 1];
        debugln("[CAN] Buffer: Removed element " + String(data_match) + " of " + String(CAN_ackBuf_index - 1) + " !");
    }
    else
        debugln("[CAN] Buffer: Removed last element " + String(data_match) + " !");

    // Decrement buffer index
    CAN_ackBuf_index--;
}

/*!
   @brief   Function to check whether ACK for sent messages timedout, to init another transmission.
   @return  false: buffer is empty | true: buffer is not empty and / or message ack timed out
   @note
*/
bool CAN_ackTimeout(sct_md *md)
{
    if (CAN_ackBuf_index == 0)
        return false;

    for (int i = 0; i < CAN_ackBuf_index; i++)
    {
        if (millis() > (CAN_ack_sentTime[i] + CAN_ack_msgTimeout))
        {
            debugln("[CAN] ACK timeout of message " + String(i) + ", resending message");
            byte payload[CAN_ackBuf_dlc[i]];
            for (int j = 0; j < CAN_ackBuf_dlc[i]; j++)
            {
                payload[j] = CAN_ackBuf_payload[i][j];
            }
            payload[0] = 0x1;
            payload[1] = 0x1;
            CAN_ackBuf_remElement(i);
            CAN_sendMessage(md, CAN_ackBuf_txID[i], CAN_ackBuf_dlc[i], payload);
            i--;
        }
    }

    return true;
}

/*!
   @brief   Function to check whether sent out messages were acknowledged.
   @return  false: buffer is empty or message is not matching.
   @note
*/
bool CAN_ackMessage(sct_md *md, int txID, byte dlc, byte payload[])
{
    if (CAN_ackBuf_index == 0)
        return false;

    int8_t data_match = -1;
    for (int i = 0; i < CAN_ackBuf_index; i++)
    {
        bool data_match_txID = false;
        bool data_match_dlc = false;
        bool data_match_payload = false;

        if (txID == CAN_ackBuf_txID[i])
            data_match_txID = true;
        else
            continue;

        if (dlc == CAN_ackBuf_dlc[i])
            data_match_dlc = true;
        else
            continue;

        for (int j = 0; j < dlc; j++)
        {
            if (payload[j] == CAN_ackBuf_payload[i][j])
                data_match_payload = true;
            else
            {
                data_match_payload = false;
                break;
            }
        }

        if (data_match_txID && data_match_dlc && data_match_payload)
        {
            data_match = i;
            debugln("[CAN] ACK data match in buffer position " + String(data_match));
            break;
        }
    }

    if (data_match > -1)
    {
        CAN_ackBuf_remElement(data_match);
    }
    else
    {
        debugln("[CAN] No ACK data match was found!");
        return false;
    }

    return true;
}

/*!
   @brief   Function to queue CAN messages into buffer
   @return  false: message not meant to be queued
   @note
*/
bool CAN_queueAckMessage(unsigned long txID, byte dlc, byte payload[])
{
    // Filters
    if (payload[2] == CAN_tpc_ping) // Ping requests don't require an acknowledgement
        return false;

    CAN_ackBuf_txID[CAN_ackBuf_index] = txID;
    CAN_ackBuf_dlc[CAN_ackBuf_index] = dlc;

    for (int i = 0; i < dlc; i++)
    {
        CAN_ackBuf_payload[CAN_ackBuf_index][i] = payload[i];
    }

    CAN_ackBuf_payload[CAN_ackBuf_index][0] = 0x0; // MCU: StarHost
    CAN_ackBuf_payload[CAN_ackBuf_index][1] = 0x2; // Msg type: ACK

    if (payload[2] == CAN_tpc_reset) // Reset CAN message creates two to get acknowledged messages (Host + ShiftGuidance)
    {
        CAN_ackBuf_index++;
        CAN_ackBuf_txID[CAN_ackBuf_index] = txID;
        CAN_ackBuf_dlc[CAN_ackBuf_index] = dlc;
        for (int i = 0; i < dlc; i++)
        {
            CAN_ackBuf_payload[CAN_ackBuf_index][i] = payload[i];
        }
        CAN_ackBuf_payload[CAN_ackBuf_index][0] = 0x2; // MCU: StarShiftGuidance
        CAN_ackBuf_payload[CAN_ackBuf_index][1] = 0x2; // Msg type: ACK
    }

    CAN_ack_sentTime[CAN_ackBuf_index] = millis(); // Store message transmission time

    CAN_ackBuf_index++;

    // Debug output buffer content
    debug("[CAN] Buffer content:");
    for (int i = 0; i < CAN_ackBuf_index; i++)
    {
        debugln("\n--- Pos | " + String(i) + " ---");
        debugln("txID: " + String(CAN_ackBuf_txID[i]));
        debugln("dlc : " + String(CAN_ackBuf_dlc[i]));
        debug("data: ");
        for (int j = 0; j < CAN_ackBuf_dlc[i]; j++)
        {
            if ((int)CAN_ackBuf_payload[i][j] < 16)
                debug("0");
            debugHEX(CAN_ackBuf_payload[i][j]);
            debug(" ");
        }
    }

    return true;
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
        CAN_queueAckMessage(txID, dlc, payload);

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
    // Ping CC
    if (((millis() > (CAN_pingCC_lastMsg + CAN_pingCC_interval)) || (CAN_pingCC_lastMsg == 0U)) && md->connected_clients[2])
    {
        CAN_pingCC_lastMsg = millis();
        Serial.print("CC!");
    }

    // Ping CanClients
    if ((millis() > (CAN_ping_lastMsg + CAN_ping_interval) || (CAN_ping_lastMsg == 0U)) && md->connected_clients[2])
    {
        debugln("[PING] No ping rqst since " + String(CAN_ping_interval) + "ms. Sending own ping request!");
        CAN_ping_lastMsg = millis();
        byte payload[CAN_DLC] = {CAN_clientID, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0};
        CAN_sendMessage(md, CAN_txID, CAN_DLC, payload); // Sending ping rqst to clients
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
    CAN_ackTimeout(md);

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

    // CAN message acknowledgment function
    CAN_ackMessage(md, id, CAN_DLC, payload);

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
            debug("\n[CAN] Message: Ping request from ");
            if (payload[0] == 0x0)
            {
                debug("StarHost!");
                if (!md->connected_clients[0])
                    debugln("[Client] StarHost connected!");
                host_lastAveMsg = millis();
                md->connected_clients[0] = true;
            }
            else if (payload[0] == 0x2)
            {
                debug("ShiftGuidance!");
                if (!md->connected_clients[1])
                    debugln("[Client] ShiftGuidance connected!");
                shift_lastAveMsg = millis();
                md->connected_clients[1] = true;
            }
            byte txPL[CAN_DLC] = {CAN_clientID, 0x1, CAN_tpc_ping, 0x1, 0x0, 0x0, 0x0, 0x0};
            CAN_sendMessage(md, CAN_txID, CAN_DLC, txPL);
            CAN_ping_lastMsg = millis();
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
        debugln("\n[CAN] ERR Not a subscribed topic!");

    return 0;
}