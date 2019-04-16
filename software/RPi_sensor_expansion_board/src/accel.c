/*
* \Name: accel.c
*/

#include "accel.h"

// Auxiliary bit masks
static const u16 MASK_DATA1 = 0xFF;
static const u16 MASK_DATA2	= 0x01;
static const u16 MASK_DATA3 = 0x7F;

/*---------------------------------------------------------------------------*
 *	The following function is used to map the SPI bus read, write and delay
 *	with global structure adxl355_t
 *--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*
 *  By using adxl355 the following structure parameter can be accessed
 *	Bus write function pointer: ADXL355_WR_FUNC_PTR
 *	Bus read function pointer: ADXL355_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *-------------------------------------------------------------------------*/
static s8 SPI_routine(void);

static struct adxl355_t adxl355;

s8 accel_setup(void){

	//Initiating GPIOs & SPI
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //GPIOC
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); //SPI2

	// Configure SPI2 CS (PB9)
	GPIO_InitTypeDef portB;

	portB.GPIO_Pin = GPIO_Pin_9;
	portB.GPIO_Mode = GPIO_Mode_OUT;
	portB.GPIO_OType = GPIO_OType_PP;
	portB.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portB.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &portB);

	GPIO_SetBits(GPIOB,GPIO_Pin_9);


	// Configure SPI2 MISO/MOSI (PC2 / PC3)
	GPIO_InitTypeDef portC;

	portC.GPIO_Pin = GPIO_Pin_2;
	portC.GPIO_Mode = GPIO_Mode_AF;
	portC.GPIO_OType = GPIO_OType_PP;
	portC.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portC.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &portC);

	portC.GPIO_Pin = GPIO_Pin_3;
	portC.GPIO_Mode = GPIO_Mode_AF;
	portC.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portC.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &portC);

	// Configure SPI2 SCLK (PC7)
	portC.GPIO_Pin = GPIO_Pin_7;
	portC.GPIO_Mode = GPIO_Mode_AF;
	portC.GPIO_OType = GPIO_OType_PP;
	portC.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portC.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &portC);

	//temporary data structure to setup SPI
	SPI_InitTypeDef Accel_SPI;

	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_SPI2); //CS - change if hardware NSS
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_SPI2); //MISO
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2); //MOSI
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_SPI2); //CLK

	//configuring SPI

	Accel_SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //we have both lines
	Accel_SPI.SPI_Mode = SPI_Mode_Master;
	Accel_SPI.SPI_DataSize = SPI_DataSize_16b; //8A + 8D
	Accel_SPI.SPI_CPOL = SPI_CPOL_Low;//SPI_CPOL_High
	Accel_SPI.SPI_CPHA = SPI_CPHA_1Edge; //SPI_CPHA_2Edge
	//Accel_SPI.SPI_NSS = SPI_NSS_Hard; change if hardware NSS
	Accel_SPI.SPI_NSS = SPI_NSS_Soft;
	Accel_SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	Accel_SPI.SPI_FirstBit = SPI_FirstBit_MSB;

	//turn SPI on
	SPI_Init(SPI2, &Accel_SPI);
	//SPI_SSOutputCmd(SPI3, ENABLE); // change if hardware NSS
	SPI_Cmd(SPI2, ENABLE);

	//assigns the functions implemented in this file to a local struct
	SPI_routine();
	//passes the local struct to assign its pointer to another local pointer
	// (in external function)
	adxl355_init(&adxl355);

	return ADXL355_SUCCESS;
}


s8 SPI_routine(void){

	adxl355.bus_write = ADXL355_SPI_bus_write;
	adxl355.bus_read = ADXL355_SPI_bus_read;

	return ADXL355_ERROR;

}

s8 ADXL355_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = ADXL355_SUCCESS;

	for(int i = 0; i < cnt; ++i){
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);
		u16 data_packet = 0;
		u16 shift_reg_data = ( (u16) *(reg_data + i));
		u16 shift_reg_address = ((u16) reg_addr + i) & ( (u16) MASK_DATA3);
		shift_reg_address = shift_reg_address << 9;
		data_packet = shift_reg_address | shift_reg_data;
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==RESET){}
		SPI_I2S_SendData(SPI2, data_packet);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)==RESET){}
		GPIO_SetBits(GPIOB,GPIO_Pin_9);
		SPI_I2S_ReceiveData(SPI2); //dummy read

	}

	return (s8)iError;
}


s8 ADXL355_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = ADXL355_SUCCESS;

	for(int i = 0 ; i < cnt; ++i){
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);
		u16 data_packet = 0;
		u16 shift_reg_address = (((u16) reg_addr + i) & ( (u16) MASK_DATA3)) << 1 ;
		data_packet = ( (shift_reg_address | MASK_DATA2) << 8 );

		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==RESET){}
		SPI_I2S_SendData(SPI2, data_packet);

		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)==RESET){}
		GPIO_SetBits(GPIOB,GPIO_Pin_9);
		u16 received_packet = SPI_I2S_ReceiveData(SPI2);

		//truncate to get 8 last significant bits
		*(reg_data + i) = (MASK_DATA1 & received_packet);
	}

	return (s8)iError;
}










