/****************************************************************************************
 *
 * Hardware configuration and low level functions
 *
 * The idea of HW initialization and configuration have taken on from
 * http://vg.ucoz.ru/load/stm32_ickhodnye_teksty_programm_na_si/stm32_biblioteka_podkljuchenija_displeja_na_kontrollere_st7735_dlja_stm32f4/16-1-0-41
 ****************************************************************************************/
#include "hw_config.h"

#include <stdio.h>

#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"


ErrorStatus HSEStartUpStatus;

uint8_t  Output_Buffer [OUTPUT_BUFFER_DATA_SIZE];
uint32_t Output_Buffer_ptr_in = 0;
uint32_t Output_Buffer_ptr_out = 0;
uint32_t Output_Buffer_length  = 0;

uint8_t  USB_Tx_State = 0;
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);

static volatile uint32_t TimingDelay;

static void (*USB_Input_HandlerPointer) (uint16_t USB_Rx_Cnt, uint8_t *USB_Rx_Buffer)  = NULL;


// Send byte via SPI to controller
void lcd7735_senddata(const uint8_t data)
{
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2, data);
}

// Send byte via SPI to controller
void lcd7735_senddata16(const uint16_t data)
{
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2, data);
}

// Send control command to controller
void lcd7735_sendCmd(const uint8_t cmd)
{
	LCD_DC0;
	lcd7735_senddata(cmd);

	while(SPI2->SR & SPI_SR_BSY);
}

// Send parameters o command to controller
void lcd7735_sendData(const uint8_t data)
{
	LCD_DC1;
	lcd7735_senddata(data);

	while(SPI2->SR & SPI_SR_BSY);
}

// Init hardware
void lcd7735_setup(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	delay_init();

	RCC_APB2PeriphClockCmd(LCD_APB2_GPIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_15;	// SCK (Pin_13), MOSI (Pin_15) SPI2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Mode        		= SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction   		= SPI_Direction_1Line_Rx;
	SPI_InitStructure.SPI_DataSize 			= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL 				= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA 				= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS 				= SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI2_BaudRatePrescaler;
	SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;

	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);

#ifndef	LCD_SOFT_RESET
	GPIO_InitStructure.GPIO_Pin  			= LCD_CSE_PIN | LCD_A0_PIN | LCD_RST_PIN;
#else
	GPIO_InitStructure.GPIO_Pin  			= LCD_CSE_PIN | LCD_A0_PIN ;
#endif /* LCD_SOFT_RESET */


	GPIO_InitStructure.GPIO_Mode 			= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed			= GPIO_Speed_50MHz;
	GPIO_Init(LCD_GPIO, &GPIO_InitStructure);

	SPI_BiDirectionalLineConfig(SPI2,SPI_Direction_Tx);
}

void delay_ms(uint32_t delay_value) {
	TimingDelay = delay_value;
	while( TimingDelay != 0 );
}

void TimingDelay_Decrement(void) {
	if (TimingDelay != 0x00) TimingDelay--;
}

// override handler
void SysTick_Handler(void) {
	TimingDelay_Decrement();
}

void delay_init (void)
{
	/* Setup SysTick Timer for 1 msec interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1) {}
	}
}


/*******************************************************************************
 * Function Name  : USB_Cable_Config
 * Description    : Software Connection/Disconnection of USB Cable
 * Input          : None.
 * Return         : Status
 *******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
	if (NewState != DISABLE)
	{
		//	Always turned on!
	}
	else
	{
		//	Always turned on!
	}
}

/*******************************************************************************
 * Function Name  : Set_USBClock
 * Description    : Configures USB Clock input (48MHz)
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void Set_USBClock(void)
{
	/* Select USBCLK source */
	RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

	/* Enable the USB clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
 * Function Name  : Enter_LowPowerMode
 * Description    : Power-off system clocks and power while entering suspend mode
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void Enter_LowPowerMode(void)
{
	/* Set the device state to suspend */
	bDeviceState = SUSPENDED;
}

/*******************************************************************************
 * Function Name  : Leave_LowPowerMode
 * Description    : Restores system clocks and power while exiting suspend mode
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void Leave_LowPowerMode(void)
{
	DEVICE_INFO *pInfo = &Device_Info;

	/* Set the device state to the correct state */
	if (pInfo->Current_Configuration != 0)
	{
		/* Device configured */
		bDeviceState = CONFIGURED;
	}
	else
	{
		bDeviceState = ATTACHED;
	}
}

/*******************************************************************************
 * Function Name  : USB_Interrupts_Config
 * Description    : Configures the USB interrupts
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void USB_Interrupts_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
 * Function Name  : Handle_USBAsynchXfer.
 * Description    : send data to USB.
 * Input          : None.
 * Return         : none.
 *******************************************************************************/
void Handle_USBAsynchXfer (void)
{
	uint16_t USB_Tx_ptr;
	uint16_t USB_Tx_length;


	if (Output_Buffer_ptr_out == OUTPUT_BUFFER_DATA_SIZE)
	{
		Output_Buffer_ptr_out = 0;
	}

	if(Output_Buffer_ptr_out == Output_Buffer_ptr_in)
	{
		USB_Tx_State = 0;
		return;
	}

	if(Output_Buffer_ptr_out > Output_Buffer_ptr_in) /* rollback */
	{
		Output_Buffer_length = OUTPUT_BUFFER_DATA_SIZE - Output_Buffer_ptr_out;
	}
	else
	{
		Output_Buffer_length = Output_Buffer_ptr_in - Output_Buffer_ptr_out;
	}

	if (Output_Buffer_length > VIRTUAL_COM_PORT_DATA_SIZE)
	{
		USB_Tx_ptr = Output_Buffer_ptr_out;
		USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;

		Output_Buffer_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
		Output_Buffer_length -= VIRTUAL_COM_PORT_DATA_SIZE;
	}
	else
	{
		USB_Tx_ptr = Output_Buffer_ptr_out;
		USB_Tx_length = Output_Buffer_length;

		Output_Buffer_ptr_out += Output_Buffer_length;
		Output_Buffer_length = 0;
	}
	USB_Tx_State = 1;


	UserToPMABufferCopy(&Output_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
	SetEPTxCount(ENDP1, USB_Tx_length);
	SetEPTxValid(ENDP1);
}

/*******************************************************************************
 * Function Name  : Output_Buffer_To_USB_Send_Data.
 * Description    : send the received data from UART 0 to USB.
 * Input          : None.
 * Return         : none.
 *******************************************************************************/
void USB_Send_Data(uint8_t data)
{
	Output_Buffer[Output_Buffer_ptr_in] = data;
	Output_Buffer_ptr_in++;

	/* To avoid buffer overflow */
	if(Output_Buffer_ptr_in >= OUTPUT_BUFFER_DATA_SIZE)
	{
		Output_Buffer_ptr_in = 0;
	}
}

void USB_Send_Str (const char *str)
{
	while (*str)
	{
		USB_Send_Data((uint8_t) * str++);
	}
}

/*******************************************************************************
 * Function Name  : Get_SerialNum.
 * Description    : Create the serial number string descriptor.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Get_SerialNum(void)
{
	uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

	Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
	Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
	Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

	Device_Serial0 += Device_Serial2;

	if (Device_Serial0 != 0)
	{
		IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
		IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
	}
}

/*******************************************************************************
 * Function Name  : HexToChar.
 * Description    : Convert Hex 32Bits value into char.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
	uint8_t idx = 0;

	for( idx = 0 ; idx < len ; idx ++)
	{
		if( ((value >> 28)) < 0xA )
		{
			pbuf[ 2* idx] = (value >> 28) + '0';
		}
		else
		{
			pbuf[2* idx] = (value >> 28) + 'A' - 10;
		}

		value = value << 4;

		pbuf[ 2* idx + 1] = 0;
	}
}


void USB_Input_CallBack_Init (void (*New_USB_Input_HandlerPointer) (uint16_t , uint8_t *) )
{
	USB_Input_HandlerPointer = New_USB_Input_HandlerPointer;
}

void USB_Input_CallBack (uint16_t USB_Rx_Cnt, uint8_t *USB_Rx_Buffer)
{
	if (USB_Input_HandlerPointer == NULL) return;
	(*USB_Input_HandlerPointer) (USB_Rx_Cnt, USB_Rx_Buffer);
}
