#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hw_config.h"

#include "ST7735.h"
#include "BMP180.h"

#define VERSION		"test 1.0"

const char version_str[] = "Version: " VERSION " (" __DATE__ ")" ;

void USB_Input_Handler (uint16_t USB_Rx_Cnt, uint8_t *USB_Rx_Buffer);

////void PrintAltitude (uint16_t Altitude)
////{
////	char Buffer[8];
////	static char OldBuffer[8];
////
////	uint8_t X_offset = 0;
////	uint16_t COLOR;
////
////	sprintf(Buffer, "%i", Altitude);
////
////	if ( strlen(OldBuffer) > strlen(Buffer) )
////	{
////		lcd7735_setForeground(ST7735_BLACK);
////		lcd7735_print(OldBuffer,X_offset,39,0);
////		//lcd7735_fillRect(0,39,160,50,ST7735_BLACK);
////	}
////
////	if (Altitude > 1300)							COLOR = ST7735_WHITE;
////	if ( (Altitude > 1200) && (Altitude < 1500) )	COLOR = ST7735_YELLOW;
////	if ( (Altitude > 800 ) && (Altitude < 1200) )	COLOR = lcd7735_Color565(255,140,0);
////	if (Altitude < 800)								COLOR = ST7735_RED;
////
////
////	X_offset = ST7735_TFTHEIGHT - 32*(strlen(Buffer));
////
////	lcd7735_setForeground(COLOR);
////	lcd7735_print(Buffer,X_offset,39,0);
////
////	strcpy(OldBuffer,Buffer);
////}
//
//
//void USB_LP_CAN1_RX0_IRQHandler(void)
//{
//	USB_Istr();
//}
//
//
////--------------------------------------------------------------
//int main(void)
//{
////	uint32_t i;
////	char Buffer[64];
////
////	int T;
////
////	unsigned int P;
////	float Alt, ZeroAlt;
////
////
//	Set_USBClock();
//	USB_Interrupts_Config();
//	USB_Init();
//	USB_Input_CallBack_Init (USB_Input_Handler);
////
////	lcd7735_setup();
////	delay_ms(50);
////	lcd7735_initR(INITR_REDTAB);
////
////	lcd7735_setFont((uint8_t *)&SmallFont[0]);
////
////
////
////	lcd7735_fillScreen(ST7735_BLACK);
////
////	lcd7735_setRotation(LANDSAPE);
////
////
////	sprintf(Buffer,version_str);
////	lcd7735_print(Buffer,0,0,0);
////	delay_ms(1000);
////
////	if (BMP180_Init(BMP180_STANDARD) == NO_ERROR)
////	{
////		sprintf(Buffer, "Connection done!");
////	}
////	else
////	{
////		sprintf(Buffer, "Error connecting to BMP180!");
////	}
////
////
////
////	lcd7735_print(Buffer,0,0,0);
////	delay_ms(1000);
////
////
////	lcd7735_fillScreen(ST7735_BLACK);
////
////	ZeroAlt = BMP180_ReadAltitude();
////
////
////	T = (int) 10*( (BMP180_ReadTemperature()) );
////	P = (unsigned int) BMP180_ReadPressure()*0.0075*10;
////	Alt = BMP180_ReadAltitude() - ZeroAlt;
////
////
////	lcd7735_setForeground(ST7735_BLUE);
////
////	sprintf(Buffer, "T = %i.%u C;", T/10,(abs(T))%10 );
////	lcd7735_print(Buffer,0,0,0);
////
////	sprintf(Buffer, "P = %i.%u mmHg;", P/10,(abs(P))%10);
////	lcd7735_print(Buffer,0,15,0);
////
////	sprintf(Buffer, "H = %i m;", (int) Alt);
////	lcd7735_print(Buffer,0,30,0);
////	delay_ms(1000);
////
////	lcd7735_fillScreen(ST7735_BLACK);
////
////	i = 4000;
////
////	lcd7735_setFont((uint8_t *)&SevenSegNumFont[0]);
////
////
//	while (1)
//
//	{
//
//
//		USB_Send_Data('a');
////
////
////		PrintAltitude(i);
//		delay_ms(300);
////		i = i - 30;
////
////
////		if (i < 700) i = 4000;
//	}
//}
//
void USB_Input_Handler (uint16_t USB_Rx_Cnt, uint8_t *USB_Rx_Buffer)
{
	char Buffer[32];

	int i;
//	for (i=0; i < USB_Rx_Cnt; i++)
//	{
//		//USB_SetLeds(USB_Rx_Buffer[i]);
//	}

	sprintf(Buffer, "\r\nReceived %i bytes: ", USB_Rx_Cnt );
	USB_Send_Str(Buffer);

	for (i = 0; i < USB_Rx_Cnt; i++)
		USB_Send_Data(USB_Rx_Buffer[i]);
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	USB_Istr();
}

void USB_print (const char *str)
{
	int n;
	int len = strlen(str);

	for (n = 0; n < len; n++)
	{
		USB_Send_Data((uint8_t) * str++);
	}
}

//--------------------------------------------------------------
void main(void)
{
	uint32_t i;

	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
	USB_Input_CallBack_Init (USB_Input_Handler);

	delay_init();

	while (1)

	{
		USB_Send_Str("Hello world!\r\n");

		delay_ms(300);
	}
}
