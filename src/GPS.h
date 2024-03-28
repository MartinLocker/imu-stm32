#ifndef _GPS_H_
#define _GPS_H_

#include "Arduino.h"

#define NMEA_BUFFERSIZE		80

// Message Codes
#define NMEA_NODATA		0	// No data. Packet not available, bad, or not decoded
#define NMEA_GPGGA		1	// Global Positioning System Fix Data
#define NMEA_GPVTG		2	// Course over ground and ground speed
#define NMEA_GPGLL		3	// Geographic position - latitude/longitude
#define NMEA_GPGSV		4	// GPS satellites in view
#define NMEA_GPGSA		5	// GPS DOP and active satellites
#define NMEA_GPRMC		6	// Recommended minimum specific GPS data
#define NMEA_UNKNOWN	0xFF// Packet received but not known

typedef struct {
	long PositionX;
	long PositionY;
  long TimeFix; 
	long Latitude;
	long Longitude;
	long Altitude;
	long HDOP;
	uint8_t NumberOfSatellites;
	uint8_t PositionFixStatus;
	long BaseLatitude;
	long BaseLongitude;
} TGPSInfo;
	
extern TGPSInfo GPSInfo;
extern char PacketNMEA[NMEA_BUFFERSIZE];
		
void InitGPS(Stream*);
uint8_t ReceiveGPS(char);
uint8_t ProcessPacketNMEA(char*);
void ComputePositionGPS(long* x, long* y);
void SetBase(long Latitude, long Longitude);

#endif
