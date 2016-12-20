#ifndef BMP180_H
#define BMP180_H

#define BMP180_DEBUG_MODE			0

#include <stdint.h>
#include "hw_config.h"	// delay_ms, DebugOutput, I2C_read/I2C_write

#define ERROR						1
#define NO_ERROR					0

#define BMP180_ULTRALOWPOWER		0
#define BMP180_STANDARD				1
#define BMP180_HIGHRES				2
#define BMP180_ULTRAHIGHRES			3


uint8_t BMP180_Init				(uint8_t mode);
float 	BMP180_ReadTemperature	(void);
int32_t BMP180_ReadPressure		(void);
float 	BMP180_ReadAltitude		(void);

uint16_t BMP180_ReadRawTemperature	(void);
uint32_t BMP180_ReadRawPressure		(void);

#endif //  BMP180_H
