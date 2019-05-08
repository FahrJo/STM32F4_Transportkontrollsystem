#ifndef GPS_MODULH
#define GPS_MODULH

#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "hilfsfunktionen.h"

#define GPS_RINGBUFFER_SIZE 700
#define GPGGA_ANZAHL_KOMMA 14
#define GPRMC_ANZAHL_KOMMA 11


typedef struct {
	char NMEA_GPRMC[80]; // $GPRMC,blabli
	char NMEA_GPGGA[80]; // $GPGGA,blabla
	uint16_t actualPos[2]; //fraglich ob diese mit in die Funktion sollen)
	uint16_t lastPos[2];
	char savedToSD;
}s_gpsSetOfData;


int GPS_activateReceiver(void);
int GPS_deactivateReceiver(void);
int GPS_sortInNewData(s_gpsSetOfData* gpsActualDataset, char* pNewNmeaString);
int GPS_getVelocity(s_gpsSetOfData* gpsActualDataset); // als INT in ZentiMeter pro Sekunde ( GG.G -> int
int GPS_getMovedDistance(s_gpsSetOfData* gpsActualDataset); // in Meter


#endif
