#include <Wire.h>

#include "quaternionFilters.h"
#include "MPU9250.h"

#include "Config.h"
#include "UART.h"
#include "DataStore.h"
#include "GPS.h"

#define LED   PB7

#ifdef SerialUSB
  HardwareSerial Serial2(USART2);
  HardwareSerial Serial3(USART3);
#endif  

MPU9250 myIMU;

struct __attribute__((__packed__)) ImuData {
  int16_t roll;   // v 0.01°
  int16_t pitch;
  int16_t yaw;
  int32_t gpsX;   // v mm od base position
  int32_t gpsY;  
  uint8_t fix;
  uint8_t hdop;
};

struct __attribute__((__packed__)) ImuRaw {
  float q[4];
  uint32_t latitude;  // get latitude [ddmm.mmmmm] ==> convert to pure degrees [ssssssss*10e3] format
	uint32_t longitude; 
  uint8_t  fix;
  uint8_t  hdop;
};

ImuData imuData;
ImuRaw  imuRaw;

void calibrateGyro() {
  myIMU.calibrateMPU9250(myIMU.gyroBias);  
}

void calibrateAccelerometer() {
  myIMU.accelBias[0] = 0.0035;
  myIMU.accelBias[1] = 0.005;
  myIMU.accelBias[2] = -0.035;
}

void setAccelerometer() {
  myIMU.accelBias[0] = params.accelBias[0] / 65536.0f;
  myIMU.accelBias[1] = params.accelBias[1] / 65536.0f;
  myIMU.accelBias[2] = params.accelBias[2] / 65536.0f;
}
  
void zeroMagnetometer() {
  myIMU.magBias[0] = 215.84;
  myIMU.magBias[1] = 141.04;
  myIMU.magBias[2] = -405.94;
  myIMU.magScale[0] = 1.06;
  myIMU.magScale[1] = 1.01;
  myIMU.magScale[2] = 0.93;
#ifdef DEBUG  
  DebugPort.println("AK8963 mag biases (mG)");
  DebugPort.println(myIMU.magBias[0]);
  DebugPort.println(myIMU.magBias[1]);
  DebugPort.println(myIMU.magBias[2]);
  DebugPort.println("AK8963 mag scale (mG)");
  DebugPort.println(myIMU.magScale[0]);
  DebugPort.println(myIMU.magScale[1]);
  DebugPort.println(myIMU.magScale[2]); 
#endif    
}

void calibrateMagnetometer() {
  myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
  DebugPort.println("AK8963 mag biases (mG)");
  DebugPort.println(myIMU.magBias[0]);
  DebugPort.println(myIMU.magBias[1]);
  DebugPort.println(myIMU.magBias[2]);
  DebugPort.println("AK8963 mag scale (mG)");
  DebugPort.println(myIMU.magScale[0]);
  DebugPort.println(myIMU.magScale[1]);
  DebugPort.println(myIMU.magScale[2]); 
}

void setMagnetometer() {
  myIMU.magBias[0] = params.magBias[0] / 65536.0f;
  myIMU.magBias[1] = params.magBias[1] / 65536.0f;
  myIMU.magBias[2] = params.magBias[2] / 65536.0f;
  myIMU.magScale[0] = params.magScale[0] / 65536.0f;
  myIMU.magScale[1] = params.magScale[1] / 65536.0f;
  myIMU.magScale[2] = params.magScale[2] / 65536.0f;
}

void resetImu() {
  calibrateGyro();
  calibrateAccelerometer();
  
  myIMU.initMPU9250();
  myIMU.initAK8963(myIMU.factoryMagCalibration);

  // Get sensor resolutions, only need to do this once
  myIMU.getAres();
  myIMU.getGres();
  myIMU.getMres();

  zeroMagnetometer();
}

void PrepareData() {
  myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                * *(getQ()+3));
  myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                * *(getQ()+2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                * *(getQ()+3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw   *= RAD_TO_DEG;
  myIMU.roll  *= RAD_TO_DEG;

  memcpy(imuRaw.q, getQ(), sizeof(imuRaw.q));
  imuRaw.latitude = GPSInfo.Latitude;
  imuRaw.longitude = GPSInfo.Longitude;
  imuRaw.fix   = (GPSInfo.PositionFixStatus << 6) | (GPSInfo.NumberOfSatellites & 0x3F);
  imuRaw.hdop  = (uint8_t)(GPSInfo.HDOP >> 14);

  imuData.pitch = myIMU.pitch * 100;
  imuData.yaw   = myIMU.yaw   * 100;
  imuData.roll  = myIMU.roll  * 100;
//  imuData.gpsX  = GPSInfo.PositionX;
//  imuData.gpsY  = GPSInfo.PositionY;
  imuData.gpsX  = GPSInfo.Latitude;
  imuData.gpsY  = GPSInfo.Longitude;
  imuData.fix   = imuRaw.fix;
  imuData.hdop  = imuRaw.hdop;

  GPSInfo.Latitude = 0;
  GPSInfo.Longitude = 0;
  GPSInfo.PositionX = 0;
  GPSInfo.PositionY = 0;
  GPSInfo.PositionFixStatus = 0;
}

void setup()
{
  pinMode(LED, OUTPUT); 
  digitalWrite(LED, 0);

  SerialGPS.begin(57600);
  InitGPS(&SerialGPS); 
  
//#ifdef DEBUG
  DebugPort.begin(UART_BAUD_RATE);
//#endif

  Wire.begin();
  resetImu();

  InitUART();
}

void Debug() {
  myIMU.delt_t = millis() - myIMU.count;
  if (myIMU.delt_t > 100)  {
      PrepareData();
      DebugPort.print("Yaw: ");
      DebugPort.print(myIMU.yaw, 2);
      DebugPort.print("\tPitch: ");
      DebugPort.print(myIMU.pitch, 2);
      DebugPort.print("\tRoll: ");
      DebugPort.print(myIMU.roll, 2);   
      DebugPort.print("\tx: ");
      DebugPort.print(imuData.gpsX);   
      DebugPort.print("\ty: ");
      DebugPort.println(imuData.gpsX);   
      myIMU.count = millis(); 
  }
}

void loop() {
  if (myIMU.scanSensors()) { 
    myIMU.updateTime(); 
    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                           myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.mx,
                           myIMU.my, myIMU.mz, myIMU.deltat);
  }

  if (SerialGPS.available()) {
    if (ReceiveGPS(SerialGPS.read()) == NMEA_GPGGA) {
#ifdef DEBUG
      DebugPort.println(PacketNMEA); 
#endif      
      ComputePositionGPS(&GPSInfo.PositionX, &GPSInfo.PositionY);      
    }
  } 
 
  if (UartReceive()) {
    if (DecodePacket()) {
      if (RxBuffer.Data[0] == 'G') {
        digitalWrite(LED, 1);
        SendAnswer((uint8_t*)&imuData, sizeof(ImuData));
      }
      if (RxBuffer.Data[0] == 'g') {
        digitalWrite(LED, 1);
        SendAnswer((uint8_t*)&imuRaw, sizeof(ImuRaw));
      }
      if (RxBuffer.Data[0] == 'S') {
        digitalWrite(LED, !digitalRead(LED));
        PrepareData();
      }
      if (RxBuffer.Data[0] == 'C') {  // kalibrace gyra
        resetImu();
      }    
      if (RxBuffer.Data[0] == 'U') {  // Ulozeni novych parametru
        if (RxBuffer.Length == sizeof(params)+1) {
          memcpy((void*)&params, (const void*)&RxBuffer.Data[1], sizeof(params));
          SaveParameters();
        }
      }
      if (RxBuffer.Data[0] == 'D') {  // Odeslani hodnot parametru
        SendAnswer((uint8_t*)&params, sizeof(params));
      }    
      if (RxBuffer.Data[0] == 'A') {  // Kalibracni hodnoty akcelerometru
        memcpy((void*)&params.accelBias, (const void*)&RxBuffer.Data[1], 3*sizeof(long));
        setAccelerometer();
      }
      if (RxBuffer.Data[0] == 'M') {  // Kalibracni hodnoty akcelerometru
        memcpy((void*)&params.magBias, (const void*)&RxBuffer.Data[1], 6*sizeof(long));
        setMagnetometer();
      }
      if (RxBuffer.Data[0] == 'B') {  // Nastaveni base pozice GPS
        memcpy((void*)&GPSInfo.Latitude, (const void*)&RxBuffer.Data[1], 2*sizeof(long));
        SetBase(GPSInfo.Latitude, GPSInfo.Longitude);
      }
    } 
  } 

  if (DebugPort.available()) {
    char a = DebugPort.read();
    if (a == 'm') {
      calibrateMagnetometer();
    }
  }
#ifdef DEBUG
  Debug();
#endif  

}
