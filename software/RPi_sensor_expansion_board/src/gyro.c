/** \name gyro.h
 */

#include "gyro.h"

// Auxiliary bit masks
static const u16 MASK_DATA1 = 0xFF;
static const u16 MASK_DATA2	= 0x80;
static const u16 MASK_DATA3 = 0x7F;

/*---------------------------------------------------------------------------*
*  struct bmg160_t parameters can be accessed by using bmg160
 *	bmg160_t having the following parameters
 *	Bus write function pointer: BMG160_WR_FUNC_PTR
 *	Bus read function pointer: BMG160_RD_FUNC_PTR
 *	Burst read function pointer: BMG160_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
*-------------------------------------------------------------------------*/
static struct bmg160_t bmg160;

//! Set if TIM9_IRQ has been visited, needs manual clear
uint32_t TIM9_IRQ_visited;

/*---------------------------------------------------------------------------*
 *	The following function is used to map the SPI bus read, write and delay
 *	with global structure bmg160_t
 *--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*
 *  By using bmg160 the following structure parameter can be accessed
 *	Bus write function pointer: BMG160_WR_FUNC_PTR
 *	Bus read function pointer: BMG160_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *-------------------------------------------------------------------------*/
static s8 SPI_routine(void);

s8 gyro_setup(void) {

	//Configuring interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	//Initiating GPIOs & SPI & TIM9
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //GPIOC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE); //TIM9
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE); //SPI3

	// Configure SPI3 CS (PA15)
	GPIO_InitTypeDef portA;

	portA.GPIO_Pin = GPIO_Pin_15;
	portA.GPIO_Mode = GPIO_Mode_OUT;
	portA.GPIO_OType = GPIO_OType_PP;
	portA.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portA.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &portA);

	GPIO_SetBits(GPIOA,GPIO_Pin_15);

	// Configure SPI3 MISO/MOSI (PB4 / PB5)
	GPIO_InitTypeDef portB;

	portB.GPIO_Pin = GPIO_Pin_5;
	portB.GPIO_Mode = GPIO_Mode_AF;
	portB.GPIO_OType = GPIO_OType_PP;
	portB.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portB.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &portB);

	portB.GPIO_Pin = GPIO_Pin_4;
	portB.GPIO_Mode = GPIO_Mode_AF;
	portB.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portB.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &portB);

	// Configure SPI3 SCLK (PC10)
	GPIO_InitTypeDef portC;

	portC.GPIO_Pin = GPIO_Pin_10;
	portC.GPIO_Mode = GPIO_Mode_AF;
	portC.GPIO_OType = GPIO_OType_PP;
	portC.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portC.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &portC);

	//temporary data structure to setup SPI
	SPI_InitTypeDef Gyro_SPI;

	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI3); //CS - change if hardware NSS
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3); //MISO
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3); //MOSI
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3); //CLK

	//configuring SPI

	Gyro_SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //we have both lines
	Gyro_SPI.SPI_Mode = SPI_Mode_Master;
	Gyro_SPI.SPI_DataSize = SPI_DataSize_16b; //8A + 8D
	Gyro_SPI.SPI_CPOL = SPI_CPOL_High;//SPI_CPOL_Low;
	Gyro_SPI.SPI_CPHA = SPI_CPHA_2Edge;
	//Gyro_SPI.SPI_NSS = SPI_NSS_Hard; change if hardware NSS
	Gyro_SPI.SPI_NSS = SPI_NSS_Soft;
	Gyro_SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;//SPI_BaudRatePrescaler_256;
	Gyro_SPI.SPI_FirstBit = SPI_FirstBit_MSB;

	//turn SPI on
	SPI_Init(SPI3, &Gyro_SPI);
	//SPI_SSOutputCmd(SPI3, ENABLE); // change if hardware NSS
	SPI_Cmd(SPI3, ENABLE);

	//assigns the functions implemented in this file to a local struct
	SPI_routine();
	//passes the local struct to assign its pointer to another local pointer
	// (in external function)
	bmg160_init(&bmg160);

	//TIM9 base configuration
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 10000;//65535 is the maximum
	TIM_TimeBaseStructure.TIM_Prescaler = 10000; // e.g. if fclk = 72MhZ / 1440 = 50 kHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

	//TIM9CH1 as timer

	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 10; //just one milisecond
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM9, &TIM_OCInitStructure);
	TIM_ITConfig(TIM9, TIM_IT_CC1, ENABLE); //NVIC configuration still required
	//TIM_Cmd(TIM9, ENABLE); enable at the delay function

	return BMG160_INIT_VALUE;
}

s8 SPI_routine(void){

	bmg160.bus_write = BMG160_SPI_bus_write;
	bmg160.bus_read = BMG160_SPI_bus_read;
	bmg160.delay_msec = BMG160_delay_msek;

	return C_BMG160_SUCCESS;

}


void TIM1_BRK_TIM9_IRQHandler(void){

	TIM9_IRQ_visited = 1;
	//checking whether the interrupt is from the first channel
	if(TIM_GetITStatus(TIM9, TIM_IT_CC1) == SET) {
		TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);
	}

}

s8 BMG160_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMG160_INIT_VALUE;

	for(int i = 0; i < cnt; ++i){
		GPIO_ResetBits(GPIOA,GPIO_Pin_15);
		u16 data_packet = 0;
		u16 shift_reg_data = ( (u16) *(reg_data + i));
		u16 shift_reg_address = ((u16) (reg_addr + i)) & ( (u16) MASK_DATA3);
		shift_reg_address = shift_reg_address << 8;
		data_packet = shift_reg_address | shift_reg_data;
		while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE)==RESET){}
		SPI_I2S_SendData(SPI3, data_packet);
		while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE)==RESET){}
		GPIO_SetBits(GPIOA,GPIO_Pin_15);
		SPI_I2S_ReceiveData(SPI3); //dummy read

	}

	return (s8)iError;
}



s8 BMG160_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMG160_INIT_VALUE;

	for(int i = 0 ; i < cnt; ++i){
		GPIO_ResetBits(GPIOA,GPIO_Pin_15);
		u16 data_packet = 0;
		data_packet = ( ((reg_addr + i) | MASK_DATA2) << 8 );

		while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE)==RESET){}
		SPI_I2S_SendData(SPI3, data_packet);

		while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE)==RESET){}
		GPIO_SetBits(GPIOA,GPIO_Pin_15);
		u16 received_packet = SPI_I2S_ReceiveData(SPI3);

		//truncate to get 8 last significant bits
		*(reg_data + i) = (MASK_DATA1 & received_packet);
	}

	return (s8)iError;
}

void BMG160_delay_msek(u32 msek)
{
	while (msek-- > 0 ){

		TIM9_IRQ_visited = 0;
		//reset the counter
		TIM_SetCounter(TIM9,0);
		TIM_Cmd(TIM9, ENABLE);

		while(!TIM9_IRQ_visited){
			__WFI();
		}

		TIM9_IRQ_visited = 0;
		TIM_Cmd(TIM9, DISABLE);
	}
}

