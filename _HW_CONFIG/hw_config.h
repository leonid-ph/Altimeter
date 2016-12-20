#ifndef __HW_CONFIG__
#define __HW_CONFIG__

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>

#include "I2C.h"

#include "usb_type.h"

#include "misc.h"


#define MASS_MEMORY_START     	0x04002000
#define BULK_MAX_PACKET_SIZE	0x00000040

#define OUTPUT_BUFFER_DATA_SIZE   2048

// Hardware SPI only!
#define SPI2_BaudRatePrescaler   SPI_BaudRatePrescaler_2	// Can't be less 6 MGz according to datasheet on SR7755 controller

#define 	LCD_GPIO			GPIOB
#define     LCD_APB2_GPIO		RCC_APB2Periph_GPIOB

#define 	LCD_RST_PIN 		GPIO_Pin_14	// PB14 (RST)
#define     LCD_A0_PIN			GPIO_Pin_12	// PB12 (RS)
#define		LCD_CSE_PIN			GPIO_Pin_10	// PB10 (CS)


#define LCD_RST1  GPIO_SetBits(LCD_GPIO, LCD_RST_PIN)
#define LCD_RST0  GPIO_ResetBits(LCD_GPIO, LCD_RST_PIN)

#define LCD_DC1   GPIO_SetBits(LCD_GPIO, LCD_A0_PIN)
#define LCD_DC0   GPIO_ResetBits(LCD_GPIO, LCD_A0_PIN)

#define LCD_CS1   GPIO_SetBits(LCD_GPIO, LCD_CSE_PIN)
#define LCD_CS0   GPIO_ResetBits(LCD_GPIO, LCD_CSE_PIN)



void lcd7735_setup(void);
void lcd7735_senddata(const uint8_t cmd);
void lcd7735_senddata16(const uint16_t data);
void lcd7735_sendCmd(const uint8_t cmd);
void lcd7735_sendData(const uint8_t data);

void Set_USBClock(void);
void USB_Cable_Config (FunctionalState NewState);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void Get_SerialNum(void);
void Handle_USBAsynchXfer (void);
void USB_Send_Data(uint8_t data);
void USB_Send_Str (const char *str);

void USB_Input_CallBack (uint16_t USB_Rx_Cnt, uint8_t *USB_Rx_Buffer);
void USB_Input_CallBack_Init ( void (*USB_Input_HandlerPointer) (uint16_t , uint8_t *) );

void delay_ms (uint32_t delay_value);
void delay_init (void);

#endif /* __HW_CONFIG__ */
