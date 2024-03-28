#ifndef _CONFIG_H_
#define _CONFIG_H_

// ========================================================================
//    UART
// ========================================================================

#define Wire ImuWire

//#define DEBUG
#ifdef SERIAL_USB
  #define DebugPort   Serial1
  #define SerialGPS   Serial2 
  #define ComPort     Serial3
#else
  #define DebugPort   Serial
  #define SerialGPS   Serial1 
  #define ComPort     Serial2
#endif

#define UART_BAUD_RATE    250000         /* baud rate*/

#ifndef sbi
  #define sbi(port, bit)  (port) |= (1<<(bit))
#endif

#ifndef cbi
  #define cbi(port, bit)  (port) &= ~(1<<(bit))
#endif

#ifndef bit_is_set
  #define bit_is_set(port, bit)  (((port) & (1<<(bit))) != 0)
#endif

#endif
