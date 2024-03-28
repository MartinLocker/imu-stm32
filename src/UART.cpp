#include "UART.h"
#include <Arduino.h>

volatile uint8_t FlagsUART;
TBuffer RxBuffer, TxBuffer;

void InitUART(void) {
  ComPort.begin(UART_BAUD_RATE);
  FlagsUART = 0;
  TxBuffer.Sync = SYNC;
  RxBuffer.Sync = SYNC;
}

uint8_t SendAnswer(uint8_t* data, uint8_t length) {
	TxBuffer.Addr = SELF_ADDR | ANSWER_ADDR;
  TxBuffer.Length = length + 3; // + ADDR, LEN, CRC
  uint8_t crc = SYNC;
  crc ^= TxBuffer.Addr;
  crc ^= length + 3;

  uint8_t index = 0;
  for (uint8_t i = 0; i < length; i++) {
    uint8_t a = data[i];
    crc ^= a;
    if (a == SYNC || a == ESC) {
    	TxBuffer.Data[index++] = ESC;
      a = ~a;
    }
  	TxBuffer.Data[index++] = a;
  }
  if (crc == SYNC || crc == ESC) {
      TxBuffer.Data[index++] = ESC;
      crc = ~crc;    
  }
  TxBuffer.Data[index++] = crc;

  ComPort.write((uint8_t*)&TxBuffer.Sync, index + 3); // + SYNC, ADDR, LEN, CRC
  return(1);
}

uint8_t DecodePacket(void)
{
  FlagsUART = 0;

//  if (RxBuffer.Addr == SELF_ADDR || RxBuffer.Addr == BROADCAST_ADDR)
  {
  	if (!CRC8(ptr(RxBuffer.Sync), RxBuffer.Length + 1))	{
  	  sbi(FlagsUART, PACKET_READY);
  	  return(1);					// Packet ready
    }
    else
    {
  	  sbi(FlagsUART, CRC_ERROR);	// CRC error
    }
  }
  return(0);						// Packet isn't for me
}

uint8_t UartReceive() {
  uint8_t static esc = 0;

  if (ComPort.available()) {
    uint8_t a = ComPort.read();
  
    if (a == SYNC) {
      RxBuffer.Index = 1;
      sbi(FlagsUART, RECEIVE_PROGRESS);
    } else {
      if (bit_is_set(FlagsUART, RECEIVE_PROGRESS)) {
        if (a == ESC) {
          esc = 1;
          return 0;
        } else if (esc) {
          a = ~a;
          esc = 0;
        }
        TPacketData(RxBuffer, RxBuffer.Index++) = a;
        if (RxBuffer.Index > 3) {
          if (RxBuffer.Index == RxBuffer.Length + 1) { 
            FlagsUART = (1 << PACKET_COMPLETE);
          }
        } else if (RxBuffer.Index == 3) {
          if (a > PACKET_LENGTH) { // Podezrele dlouhy packet
            cbi(FlagsUART, RECEIVE_PROGRESS);
          }
        } else if (RxBuffer.Index == 2) {
          if (a != SELF_ADDR && a != BROADCAST_ADDR) { // Packet neni urceny pro me
            cbi(FlagsUART, RECEIVE_PROGRESS);
          }
        }
      }
    }
  }
  return bit_is_set(FlagsUART, PACKET_COMPLETE);
}

