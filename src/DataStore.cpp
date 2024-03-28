#include "DataStore.h"

TParameters params;

void InitEeprom() {
  EEPROM.init(0x801F000, 0x801F800, 0x400);  
}

#define WordArray(x)(((uint16_t*)&params + i))

void SaveParameters() {
  for (int i = 0; i < sizeof(params) / 2; i++) {
    EEPROM.update(i, *WordArray(i));
  }
}

void LoadParameters() {
  for (int i = 0; i < sizeof(params) / 2; i++) {
    EEPROM.read(i, WordArray(i));
  }
}
