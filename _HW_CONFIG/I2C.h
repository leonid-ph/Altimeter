#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#include <stm32f10x_gpio.h>
#include <stm32f10x_i2c.h>
#include <stm32f10x_rcc.h>

void I2C_Initialization (uint8_t DeviceAddress);

void I2C_write(uint8_t address, uint8_t data);
uint8_t I2C_read(uint8_t address);

uint16_t I2C_16read (uint8_t address);

#endif // I2C_H
