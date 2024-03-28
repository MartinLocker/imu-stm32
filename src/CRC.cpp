#include "CRC.h"

uint8_t CRC8(uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  uint8_t i;
  for (i = 0; i < len; i++) {
    crc ^= data[i];
  } 
  return crc; 
}
