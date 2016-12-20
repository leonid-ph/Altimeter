#include "I2C.h"

uint8_t DevAddr;

void I2C_Initialization (uint8_t DeviceAddress)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	DevAddr = DeviceAddress;

	// А вот и настройка I2C
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;

	// Адрес я тут взял первый пришедший в голову
	I2C_InitStructure.I2C_OwnAddress1 = 1;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitStructure);

	// I2C использует две ноги микроконтроллера, их тоже нужно настроить
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Ну и включаем, собственно, модуль I2C1
	I2C_Cmd(I2C1, ENABLE);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}

void I2C_write(uint8_t address, uint8_t data)
{
	I2C_GenerateSTART(I2C1, ENABLE);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		;
	}
	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		;
	}
	I2C_SendData(I2C1, address);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		;
	}
	I2C_SendData(I2C1, data);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		;
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
}

uint8_t I2C_read(uint8_t address)
{
	uint8_t data;

	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
	{
		;
	}
	I2C_GenerateSTART(I2C1, ENABLE);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		;
	}
	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		;
	}
	I2C_SendData(I2C1, address);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		;
	}
	I2C_GenerateSTART(I2C1, ENABLE);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		;
	}
	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Receiver);

	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		;
	}
	data = I2C_ReceiveData(I2C1);

	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);

	return data;
}

uint16_t I2C_16read (uint8_t address)
{
	return ( ( I2C_read(address)<<8 ) + I2C_read(address + 1) );
}
