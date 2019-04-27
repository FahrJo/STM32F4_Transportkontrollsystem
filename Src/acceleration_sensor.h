#ifndef ACCELERATION_SENSORH
#define ACCELERATION_SENSORH

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Reg 0x0B System mode. Default value: 00.
00: Standby mode
01: Wake mode
10: Sleep mode
READ ONLY !!!! */

// Register des Accelerometers ACC_...
#define ACC_OUT_X_MSB 0x01
#define ACC_OUT_X_LSB 0x02
#define ACC_OUT_Y_MSB 0x03
#define ACC_OUT_Y_LSB 0x04
#define ACC_OUT_Z_MSB 0x05
#define ACC_OUT_Z_LSB 0x06

#define ACC_CTRL_REG1 0x2A
#define ACC_CTRL_REG2 0x2B
// following used for interupt generation
#define ACC_CTRL_REG3 0x2C
#define ACC_CTRL_REG4 0x2D
#define ACC_CTRL_REG5 0x2E
// important register bits to set/reset
#define ACC_CTRL_REG1_ACTIVE 0x01
#define ACC_CTRL_REG1_F_READ 0x02
#define ACC_CTRL_REG1_LNOISE 0x04
// for DataRate CTRL_REG1 = xx100xxx (die drei legen fest vonn 000=800Hz bis 111=1,56Hz.  Nicht einfach über einmalig Bitmaske umschaltbar 

// 001110(0/1)+R/W   0011 100x  /bzw/ 0011 101x   0x1C, 0x1D wären die Beiden ohne r/w bit  38 / 3A
#define ACCELEROMETER_I2C_ADRESS 0x3A

#define GPS_RINGBUFFER_SIZE 500

typedef struct {
	int16_t x_Value;
	int16_t y_Value;
	int16_t z_Value;
}s_accelerometerValues;

typedef struct {
	float x_Value;
	float y_Value;
	float z_Value;
}s_accelerometerValuesFloat;

typedef struct {
	char NMEA_GPRMC[80]; // $GPRMC,blabli
	char NMEA_GPGGA[80]; // $GPGGA,blabla
	uint16_t actualPos[2]; //fraglich ob diese mit in die Funktion sollen)
	uint16_t lastPos[2];
	char savedToSD;
}s_gpsSetOfData;


/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef i2c_write_register(I2C_HandleTypeDef *hi2c3, uint8_t device_slave_adress, uint8_t register_pointer, uint16_t register_data_to_write, uint16_t number_bytes_to_write);
HAL_StatusTypeDef i2c_read_register(I2C_HandleTypeDef *hi2c3, uint8_t device_slave_adress, uint8_t register_pointer, uint8_t *register_data_read_buffer, uint8_t number_bytes_to_read);
HAL_StatusTypeDef ACC_activate(I2C_HandleTypeDef *i2cHandler);
HAL_StatusTypeDef ACC_deactivate(I2C_HandleTypeDef *i2cHandler);
HAL_StatusTypeDef ACC_getAllValues(I2C_HandleTypeDef *hi2c3, s_accelerometerValues *accValues);

float ACC_convertAccelToFloat(int16_t rohDaten, uint8_t breiteInBit, uint8_t messbereich);

void ACC_incrementAdress(void);
void ACC_setAdress(uint8_t);

int GPS_activateReceiver(void);
int GPS_deactivateReceiver(void);
int GPS_sortInNewData(s_gpsSetOfData* gpsActualDataset, char* pNewNmeaString);
int GPS_getVelocity(s_gpsSetOfData* gpsActualDataset); // als INT in ZentiMeter pro Sekunde ( GG.G -> int
int GPS_getMovedDistance(s_gpsSetOfData* gpsActualDataset); // in Meter

#endif
