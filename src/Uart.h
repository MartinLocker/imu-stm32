#ifndef _UART_H_
#define _UART_H_

#include "Config.h"
#include "CRC.h"

#define TRANSMIT_PROGESS	0
#define	RECEIVE_PROGRESS	1
#define PACKET_COMPLETE		2
#define PACKET_READY			3
#define CRC_ERROR					4

#define SYNC							0x55
#define ESC               0x56

#define RXD								PD0
#define TXD								PD1

#define	MASTER_ADDR				0x00
#define	BROADCAST_ADDR		0xFF
#define	SELF_ADDR					0x02
#define	ANSWER_ADDR				0x80

#define ptr(x)						(uint8_t*)&(x)
#define TPacketData(x,i)	(*((uint8_t*)(&x.Sync)+i))

#define PACKET_LENGTH 128

typedef struct __attribute__((packed)) {
  uint8_t Index;
  uint8_t Sync;
  uint8_t Addr;
  uint8_t Length;
  uint8_t Data[PACKET_LENGTH+1];
} TBuffer;

extern volatile uint8_t FlagsUART;
extern TBuffer RxBuffer, TxBuffer;

extern void    InitUART(void);
extern uint8_t SendAnswer(uint8_t* data, uint8_t length);
extern uint8_t DecodePacket(void);
extern uint8_t UartReceive();

#endif
