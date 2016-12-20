#include "BMP180.h"
#include <math.h>

#define BMP180_I2CADDR				0xEF  // Крайние 7 бит

// Адреса с калибровочными коэффициентами
#define BMP180_CAL_AC1				0xAA  // R   Calibration data (16 bits)
#define BMP180_CAL_AC2				0xAC
#define BMP180_CAL_AC3				0xAE
#define BMP180_CAL_AC4				0xB0
#define BMP180_CAL_AC5				0xB2
#define BMP180_CAL_AC6				0xB4
#define BMP180_CAL_B1				0xB6
#define BMP180_CAL_B2				0xB8
#define BMP180_CAL_MB				0xBA
#define BMP180_CAL_MC				0xBC
#define BMP180_CAL_MD				0xBE
// Регистры датчика
#define BMP180_CONTROL				0xF4
#define BMP180_TEMPDATA				0xF6
#define BMP180_PRESSUREDATA			0xF6
// Команды
#define BMP180_READTEMPCMD			0x2E
#define BMP180_READPRESSURECMD		0x34

const float SeaLevelPressure = 101325.0; // Pa

int16_t	AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;
uint8_t oversampling;


uint8_t BMP180_Init(uint8_t mode)
{
	I2C_Initialization(BMP180_I2CADDR);
	delay_init();

	if (mode > BMP180_ULTRAHIGHRES) 		mode = BMP180_ULTRAHIGHRES;

	oversampling = mode;

	if (I2C_read(0xD0) != 0x55) return ERROR; // Не BMP180!

	/* read calibration data */
	AC1 = I2C_16read(BMP180_CAL_AC1);
	AC2 = I2C_16read(BMP180_CAL_AC2);
	AC3 = I2C_16read(BMP180_CAL_AC3);
	AC4 = I2C_16read(BMP180_CAL_AC4);
	AC5 = I2C_16read(BMP180_CAL_AC5);
	AC6 = I2C_16read(BMP180_CAL_AC6);

	B1 = I2C_16read(BMP180_CAL_B1);
	B2 = I2C_16read(BMP180_CAL_B2);

	MB = I2C_16read(BMP180_CAL_MB);
	MC = I2C_16read(BMP180_CAL_MC);
	MD = I2C_16read(BMP180_CAL_MD);

#if (BMP180_DEBUG_MODE == 1)
	/*
	Serial.print("ac1 = "); Serial.println(ac1, DEC);
	Serial.print("ac2 = "); Serial.println(ac2, DEC);
	Serial.print("ac3 = "); Serial.println(ac3, DEC);
	Serial.print("ac4 = "); Serial.println(ac4, DEC);
	Serial.print("ac5 = "); Serial.println(ac5, DEC);
	Serial.print("ac6 = "); Serial.println(ac6, DEC);

	Serial.print("b1 = "); Serial.println(b1, DEC);
	Serial.print("b2 = "); Serial.println(b2, DEC);

	Serial.print("mb = "); Serial.println(mb, DEC);
	Serial.print("mc = "); Serial.println(mc, DEC);
	Serial.print("md = "); Serial.println(md, DEC);
	 */
#endif

	return NO_ERROR;
}

uint16_t BMP180_ReadRawTemperature(void)
{
	I2C_write(BMP180_CONTROL, BMP180_READTEMPCMD);
	delay_ms(5);

	return I2C_16read(BMP180_TEMPDATA);
}

uint32_t BMP180_ReadRawPressure(void)
{
	uint32_t raw;

	I2C_write(BMP180_CONTROL, BMP180_READPRESSURECMD + (oversampling << 6));

	switch (oversampling)
	{
	case BMP180_ULTRALOWPOWER:
		delay_ms(5);
		break;
	case BMP180_STANDARD:
		delay_ms(8);
		break;
	case BMP180_HIGHRES:
		delay_ms(14);
		break;
	case BMP180_ULTRAHIGHRES:
		delay_ms(26);
		break;
	default:
		delay_ms(26);
		break;
	}

	raw = I2C_16read(BMP180_PRESSUREDATA);

	raw <<= 8;
	raw |= I2C_read(BMP180_PRESSUREDATA + 2);
	raw >>= (8 - oversampling);

	/* this pull broke stuff, look at it later?
  if (oversampling==0) {
    raw <<= 8;
    raw |= I2C_read(BMP180_PRESSUREDATA+2);
    raw >>= (8 - oversampling);
  }
	 */

#if BMP180_DEBUG_MODE == 1
	//Serial.print("Raw pressure: "); Serial.println(raw);
#endif

	return raw;
}


int32_t BMP180_ReadPressure(void) 
{
	int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
	uint32_t B4, B7;

	UT = BMP180_ReadRawTemperature();
	UP = BMP180_ReadRawPressure();

#if BMP180_DEBUG_MODE == 1
	// use datasheet numbers!
	UT = 27898;
	UP = 23843;
	AC6 = 23153;
	AC5 = 32757;
	MC = -8711;
	MD = 2868;
	B1 = 6190;
	B2 = 4;
	AC3 = -14383;
	AC2 = -72;
	AC1 = 408;
	AC4 = 32741;

	oversampling = 0;
#endif

	// do temperature calculations
	X1 = (UT-(int32_t)(AC6))*((int32_t)(AC5))/pow(2,15);
	X2 = ((int32_t)MC*pow(2,11))/(X1+(int32_t)MD);
	B5 = X1 + X2;

#if BMP180_DEBUG_MODE == 1
	/*
	Serial.print("X1 = "); Serial.println(X1);
	Serial.print("X2 = "); Serial.println(X2);
	Serial.print("B5 = "); Serial.println(B5);
	 */
#endif

	// do pressure calcs
	B6 = B5 - 4000;
	X1 = ((int32_t)B2 * ( (B6 * B6)>>12 )) >> 11;
	X2 = ((int32_t)AC2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = ((((int32_t)AC1*4 + X3) << oversampling) + 2) / 4;

#if BMP180_DEBUG_MODE == 1
	/*
	Serial.print("B6 = "); Serial.println(B6);
	Serial.print("X1 = "); Serial.println(X1);
	Serial.print("X2 = "); Serial.println(X2);
	Serial.print("B3 = "); Serial.println(B3);
	 */
#endif

	X1 = ((int32_t)AC3 * B6) >> 13;
	X2 = ((int32_t)B1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = ((uint32_t)AC4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

#if BMP180_DEBUG_MODE == 1
	/*
	Serial.print("X1 = "); Serial.println(X1);
	Serial.print("X2 = "); Serial.println(X2);
	Serial.print("B4 = "); Serial.println(B4);
	Serial.print("B7 = "); Serial.println(B7);
	 */
#endif

	if (B7 < 0x80000000)
	{
		p = (B7 * 2) / B4;
	}
	else
	{
		p = (B7 / B4) * 2;
	}

	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;

#if BMP180_DEBUG_MODE == 1
	/*
	Serial.print("p = "); Serial.println(p);
	Serial.print("X1 = "); Serial.println(X1);
	Serial.print("X2 = "); Serial.println(X2);
	 */
#endif

	p = p + ((X1 + X2 + (int32_t)3791)>>4);

#if BMP180_DEBUG_MODE == 1
	//Serial.print("p = "); Serial.println(p);
#endif

	return p;
}


float BMP180_ReadTemperature(void)
{
	int32_t UT, X1, X2, B5;     // following ds convention
	float temp;

	UT = BMP180_ReadRawTemperature();

#if BMP180_DEBUG_MODE == 1
	// use datasheet numbers!
	UT = 27898;
	AC6 = 23153;
	AC5 = 32757;
	MC = -8711;
	MD = 2868;
#endif

	// step 1
	X1 = (UT - (int32_t)AC6) * ((int32_t)AC5) / pow(2,15);
	X2 = ((int32_t)MC * pow(2,11)) / (X1+(int32_t)MD);
	B5 = X1 + X2;
	temp = (B5+8)/pow(2,4);
	temp /= 10;

	return temp;
}

float BMP180_ReadAltitude(void)
{
	float altitude, pressure;

	pressure = BMP180_ReadPressure();
	altitude = 44330 * (1.0 - pow(pressure/SeaLevelPressure, 0.1903));

	return altitude;
}
