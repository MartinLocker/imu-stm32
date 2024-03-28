#ifndef _DATASTORE_H_
#define _DATASTORE_H_

#include <EEPROM.h> 

typedef struct {
  long 
    gyroBias[3],
    accelBias[3],
    magBias[3],
    magScale[3];
} TParameters;

extern TParameters params;

void SaveParameters();
void LoadParameters();

#endif

