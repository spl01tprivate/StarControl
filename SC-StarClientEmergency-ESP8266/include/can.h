#ifndef CAN_h
#define CAN_h

#include <maindata.h>

String CAN_publisher(sct_md *, byte, byte);
void CAN_ackBuf_remElement(int8_t);
bool CAN_ackTimeout(sct_md *);
bool CAN_ackMessage(sct_md *, int, byte, byte[]);
bool CAN_queueAckMessage(unsigned long, byte, byte[]);
void CAN_sendMessage(sct_md *, unsigned long, byte, byte[]);
void CAN_aliveMessage(sct_md *);
uint8_t CAN_checkMessages(sct_md *);

#endif