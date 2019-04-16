/*
 * \Name comm.c
 */

#include "comm.h"

// Auxiliary bit masks
static const u16 MASK_DATA1 = 0xFF;

//failure / success definitions
static const s8 COMM_SUCCESS = 0;
static const s8 COMM_FAILURE = -1;

s8 comm_setup(void){

	//Initiating GPIOs & SPI
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //SPI1

	// Configure SPI1 CS (PA4)
	GPIO_InitTypeDef portA;

	portA.GPIO_Pin = GPIO_Pin_4;
	portA.GPIO_Mode = GPIO_Mode_AF;
	portA.GPIO_OType = GPIO_OType_PP;
	portA.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portA.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &portA);

	// Configure SPI1 MISO/MOSI (PA6 / PA7)

	portA.GPIO_Pin = GPIO_Pin_6;
	portA.GPIO_Mode = GPIO_Mode_AF;
	portA.GPIO_OType = GPIO_OType_PP;
	portA.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portA.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &portA);

	portA.GPIO_Pin = GPIO_Pin_7;
	portA.GPIO_Mode = GPIO_Mode_AF;
	portA.GPIO_OType = GPIO_OType_PP;
	portA.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portA.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &portA);

	// Configure SPI1 SCLK (PA5)
	portA.GPIO_Pin = GPIO_Pin_5;
	portA.GPIO_Mode = GPIO_Mode_AF;
	portA.GPIO_OType = GPIO_OType_PP;
	portA.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portA.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &portA);

	//Configuring alternative function

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1); //CS
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); //CLK
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); //MISO
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); //MOSI

	//configuring SPI

	comm_set_SPI_2Lines_FullDuplex();

	return COMM_SUCCESS;
}

void comm_set_SPI_2Lines_FullDuplex(void){
	//temporary data structure to setup SPI
	SPI_InitTypeDef Comm_SPI;

	SPI_Cmd(SPI1, DISABLE);
	Comm_SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //we have both lines
	Comm_SPI.SPI_Mode = SPI_Mode_Slave;
	Comm_SPI.SPI_DataSize = SPI_DataSize_8b;
	Comm_SPI.SPI_CPOL = SPI_CPOL_Low;
	Comm_SPI.SPI_CPHA = SPI_CPHA_1Edge;
	Comm_SPI.SPI_NSS = SPI_NSS_Soft;
	Comm_SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	Comm_SPI.SPI_FirstBit = SPI_FirstBit_MSB;

	//turn SPI on
	SPI_Init(SPI1, &Comm_SPI);
	//SPI_SSOutputCmd(SPI1, ENABLE); //will this change anything?
	SPI_Cmd(SPI1, ENABLE);

}

void comm_set_SPI_1Line_Tx(void){
	//temporary data structure to setup SPI
	SPI_InitTypeDef Comm_SPI;

	SPI_Cmd(SPI1, DISABLE);
	Comm_SPI.SPI_Direction = SPI_Direction_1Line_Tx; //we have both lines
	Comm_SPI.SPI_Mode = SPI_Mode_Slave;
	Comm_SPI.SPI_DataSize = SPI_DataSize_8b;
	Comm_SPI.SPI_CPOL = SPI_CPOL_Low;
	Comm_SPI.SPI_CPHA = SPI_CPHA_1Edge;
	Comm_SPI.SPI_NSS = SPI_NSS_Soft;
	Comm_SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	Comm_SPI.SPI_FirstBit = SPI_FirstBit_MSB;

	//turn SPI on
	SPI_Init(SPI1, &Comm_SPI);
	//SPI_SSOutputCmd(SPI1, ENABLE); //will this change anything?
	SPI_Cmd(SPI1, ENABLE);

}


u8 comm_get_mode(void){

	u16 received_data;

	//TODO optimize the power consumption by using SPI interrupt
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
	received_data = SPI_I2S_ReceiveData(SPI1);

	return ((u8) (received_data & MASK_DATA1));
}


s8 comm_send_error(void){

	u16 sent_data = COMM_ERROR;
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,sent_data);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
	SPI_I2S_ReceiveData(SPI1);//clear the reception register as to begin with clear state

	return COMM_SUCCESS;
}



