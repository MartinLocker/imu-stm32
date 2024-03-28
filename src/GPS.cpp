#include "GPS.h"

#include <string.h>

TGPSInfo GPSInfo;

uint8_t StartFlag;
uint8_t IndexNMEA;
char PacketNMEA[NMEA_BUFFERSIZE];

// meritko ulozeni pozice (ssssss * 10e6)
#define Scale  (3600*1000l)
double fracX, fracY;

#define atol atoi

int32_t atoi(char* p) {
  int32_t x = 0;
  int8_t sign = 0;
  if (*p == '-') {
    sign = 1;
    p++;
  }
  while (*p >= '0' && *p <= '9') {
    x = x * 10 + (*p - '0');
    p++;
  }
  if (sign) x = -x;
  return x;
}
double frac(char* p) {
  double x = 0;
  int32_t e = 1;
  while (*p >= '0' && *p <= '9') {
    x = x * 10 + (*p - '0');
    p++;
    e *= 10;
  }    
  return (double)x/e;
}
double strtod(char* p) {
  double x = 0, y = 0;
  
  while (*p >= '0' && *p <= '9') {
    x = x * 10 + (*p - '0');
    p++;
  }
  if (*p++ == '.') y = frac(p);
  return (x + y);
}

void InitGPS(Stream* SerialGPS) {
  StartFlag   = 0;
  IndexNMEA = 0;
  SerialGPS->println("$PMTK220,100*2F");
  SerialGPS->println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  // Provizorni reseni
  ProcessPacketNMEA("GPGGA,110647.000,5009.9376,N,01616.7827,E,1,05,2.8,318.2,M,43.9,M,,0000*59");
  SetBase(GPSInfo.Latitude, GPSInfo.Longitude);  
}
   
uint8_t ProcessPacketNMEA(char* Packet) {
	uint8_t i;
	char* endptr;
	long  Degrees;
	long  MinutesFrac = 0;

	if(!strncmp(Packet, "GPGGA", 5))
	{
		// start parsing just after "GPGGA,"
		i = 6;
		// attempt to reject empty packets right away
		if(Packet[i]==',' && Packet[i+1]==',')
			return 0;
	
		// get UTC time [hhmmss]
		GPSInfo.TimeFix = atol(&Packet[i]);
		while(Packet[i++] != ',');				// next field: latitude

		// get latitude [ddmm.mmmmm]
		Degrees = atol(&Packet[i]);
		if (Degrees != 0)
		{
			while(Packet[i++] != '.');				// next field: frac minutes
			MinutesFrac = atol(&Packet[i]);			
		}
		// convert to pure degrees [ssssssss*10e3] format
		GPSInfo.Latitude = ((Degrees/100)*600000 + ((Degrees%100)*10000 + MinutesFrac))*6;
		while(Packet[i++] != ',');				// next field: N/S indicator

		// correct latitute for N/S
		if(Packet[i] == 'S') GPSInfo.Latitude = -GPSInfo.Latitude;
		while(Packet[i++] != ',');				// next field: longitude
		
		// get longitude [ddmm.mmmmm]
		Degrees = atol(&Packet[i]);
		if (Degrees != 0)
		{
			while(Packet[i++] != '.');				// next field: frac minutes
			MinutesFrac = atol(&Packet[i]);			
		}
		// convert to pure degrees [ssssssss*10e3] format
		GPSInfo.Longitude = ((Degrees/100)*600000 + ((Degrees%100)*10000 + MinutesFrac))*6;
		while(Packet[i++] != ',');				// next field: E/W indicator
	
		// correct latitute for E/W
		if(Packet[i] == 'W') GPSInfo.Longitude = -GPSInfo.Longitude;
		while(Packet[i++] != ',');				// next field: position fix status
	
		// position fix status
		// 0 = Invalid, 1 = Valid SPS, 2 = Valid DGPS, 3 = Valid PPS
		// check for good position fix
		GPSInfo.PositionFixStatus = atoi(&Packet[i]);
			
		while(Packet[i++] != ',');				// next field: satellites used
		
		// get number of satellites used in GPS solution
		GPSInfo.NumberOfSatellites = atoi(&Packet[i]);
		while(Packet[i++] != ',');				// next field: HDOP (horizontal dilution of precision)

		// get HDOP (horizontal dilution of precisiom) (in meters mm.mm)
//		GPSInfo.HDOP = (long) (65536 * strtod(&Packet[i], &endptr));
    GPSInfo.HDOP = (long) (65536 * strtod(&Packet[i]));
		while(Packet[i++] != ',');				// next field: altitude
		
		// get altitude (in meters mm.mm)
//		GPSInfo.Altitude = (long) (65536 * strtod(&Packet[i], &endptr));
    GPSInfo.Altitude = (long) (65536 * strtod(&Packet[i]));
			
//		while(Packet[i++] != ',');				// next field: altitude units, always 'M'
//		while(Packet[i++] != ',');				// next field: geoid seperation
//		while(Packet[i++] != ',');				// next field: seperation units
//		while(Packet[i++] != ',');				// next field: DGPS age
//		while(Packet[i++] != ',');				// next field: DGPS station ID
//		while(Packet[i++] != '*');				// next field: checksum

		return NMEA_GPGGA;
	}
	else
  	return NMEA_UNKNOWN;
}

uint8_t ReceiveGPS(char x) {
	if (x == '$')
	{
		StartFlag = 1;
		IndexNMEA = 0;
		return 0;
	}	
	
	if (StartFlag)
	{
	  if (x == '\r')
	  {
			StartFlag = 0;
			return ProcessPacketNMEA(PacketNMEA);
  	}
  	else
  	{
  	  if (IndexNMEA < NMEA_BUFFERSIZE)
  	  	PacketNMEA[IndexNMEA++] = x;
  	  return 0;	
  	}
	}
	return 0;
}

void ComputePositionGPS(long* x, long* y) {
  if (GPSInfo.PositionFixStatus == 1) {
    *x = (double)(GPSInfo.Longitude - GPSInfo.BaseLongitude) * fracX;
    *y = (double)(GPSInfo.Latitude - GPSInfo.BaseLatitude) * fracY;  
  } else {
    *x = -10;
    *y = -10;
  }
  
}

void SetBase(long Latitude, long Longitude) {
  GPSInfo.BaseLatitude = Latitude;
  GPSInfo.BaseLongitude = Longitude;
  fracY = (40000000000.0/360.0)/Scale;
  fracX = (40000000000.0/360.0)/Scale;
  fracX *= cos((double) GPSInfo.BaseLatitude*M_PI/(Scale*180));
}


