#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef i2c_write_register(I2C_HandleTypeDef hi2c3, uint8_t device_slave_adress, uint8_t register_pointer, uint16_t register_data_to_write, uint16_t number_bytes_to_write);
HAL_StatusTypeDef i2c_read_register(I2C_HandleTypeDef hi2c3, uint8_t device_slave_adress, uint8_t register_pointer, uint8_t *register_data_read_buffer, uint8_t number_bytes_to_read);
HAL_StatusTypeDef getAllAccelerometerValues(I2C_HandleTypeDef hi2c3, s_accelerometerValues *accValues);
