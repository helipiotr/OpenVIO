/*
 * \Name sensor_data_mode.c
 */

#include "modes/sensor_data_mode.h"
#include "stm32f4xx_dma.h" //reinclude?
#include "stm32f4xx_spi.h"

//failure / success definitions
static const s8 SENSOR_DATA_MODE_SUCCESS = 0;
static const s8 SENSOR_DATA_MODE_ERROR = -1;

/*!
 * \Brief: data structure holding the data for the main unit
 *
 * This array holds various values that may be represented as
 * signed or unsigned values: needs to be taken care of. Alignment:
 * \Note: - (u32) sensor type, refer to bit masks in comm_defs.h
 * \Note: - (u32) timestamp in microseconds
 * \Note: - (s32) x data (sensor) or (u32) frame_begin->1,
 * 			frame_end -> 0
 * \Note: - (s32) y data (sensor) or (u32) frame number
 * \Note: - (s32) z data (sensor) or empty
 */
static u32 sensor_readings[SENSOR_DATA_MODE_RECORDS_CAPACITY]
						   [SENSOR_DATA_MODE_RECORDS_WIDTH];

static int FIFO_in = 0; //shows an empty place for the next record
static int FIFO_out = 0; //current location of the output
static int FIFO_err = 0; //to be checked before the transmission
static int tx_complete = 0;
static int rx_complete = 0;

static int strobe = 0; //TODO remove after tests

s8 sensor_data_mode_main(void){
	//reserve the required resources
	shared_lock( SENSOR_DATA_MODE_MUTVAL );

	//start with setting the parameters of the transmission
	if (sensor_data_mode_set_parameters() == SENSOR_DATA_MODE_ERROR){
		comm_send_error();
		return SENSOR_DATA_MODE_ERROR;
	}

	//configure TIM5 to timestamp data and TIM2 to trigger recording
	sensor_data_mode_conf_timer();

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}

	if (SPI_I2S_ReceiveData(SPI1) != (u16) START_ACQUISITION){
		comm_send_error();
		return SENSOR_DATA_MODE_ERROR;
	}

	//turn on timers as we are starting the measurement
	TIM_SetCounter(TIM2, 0);
	TIM_SetCounter(TIM5, 0);
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM5, ENABLE);

	//TODO implement measurement and sending data to the main unit

	s8 comm_result;
	while( 1 ){
		//get requested operation

		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
		SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}

		u16 received_data = SPI_I2S_ReceiveData(SPI1);

		//FlagStatus status = SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE);
		//status = SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE);

		if ( received_data == STOP_ACQUISITION){
			//stop regardless the FIFO status
			sensor_data_mode_shutdown();
			return SENSOR_DATA_MODE_SUCCESS;
		} else if (received_data == SEND_DATA && !FIFO_err) {
			//Send a confirmation
			while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
			SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
			while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
			SPI_I2S_ReceiveData(SPI1);

			comm_result = sensor_data_mode_send_readings();
		}else{
			sensor_data_mode_shutdown();
			//TODO send an error message
			comm_send_error();
			return SENSOR_DATA_MODE_ERROR;
		}

		//go back in case communication failed
		if(comm_result == SENSOR_DATA_MODE_ERROR){
			sensor_data_mode_shutdown();
			//TODO send an error message
			return SENSOR_DATA_MODE_ERROR;
		}

	}

	shared_free( SENSOR_DATA_MODE_MUTVAL );
	return SENSOR_DATA_MODE_SUCCESS;
}

void sensor_data_mode_shutdown(void){
	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM5, DISABLE);
	FIFO_in = 0;
	FIFO_out = 0;
	FIFO_err = 0;
}

s8 sensor_data_mode_send_readings(){

	//TODO send a confirmation that the mode is selected

	//number of uint32_t to be transferred (in words)
	u16 transfer_size_words;
	int mem_layout_cont;

	//lock the value until the transfer is complete -> really thread-safe?
	int FIFO_in_transfer = FIFO_in;

	//calculate wrap-around value
	int mem_layout_thresh = 0;

	// Compute the size of the FIFO block to transfer
	if( (FIFO_in_transfer - FIFO_out ) >= 0){
		transfer_size_words = (FIFO_in_transfer - FIFO_out) *
				SENSOR_DATA_MODE_RECORDS_WIDTH;
		mem_layout_cont = 1;
	}
	else{
		//Transfer just one block on memory
		transfer_size_words = (SENSOR_DATA_MODE_RECORDS_CAPACITY - FIFO_out)
				* SENSOR_DATA_MODE_RECORDS_WIDTH;
		mem_layout_cont = 0;
		mem_layout_thresh = (SENSOR_DATA_MODE_RECORDS_CAPACITY - FIFO_out)
				*SENSOR_DATA_MODE_RECORDS_WIDTH;
	}


	//send the number of the elements stored in the FIFO
	u8 to_send[2];

	to_send[0] = 0xFF & transfer_size_words; //LSB
	to_send[1] = transfer_size_words >> 8; //MSB

	for (int i = 0; i < 2 ; ++i){
		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
		SPI_I2S_SendData(SPI1,to_send[i]);
		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
		SPI_I2S_ReceiveData(SPI1);
	}

	//Configure the interrupt

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_Init(&NVIC_InitStructure);

	//Configure the DMA
	//Turn on the DMA clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_DeInit(DMA2_Stream2);
	DMA_DeInit(DMA2_Stream0);


	//TODO make configuration more robust
	DMA_InitTypeDef DMAinit;
	DMAinit.DMA_PeripheralBaseAddr = (u32) &(SPI1->DR);
	DMAinit.DMA_Memory0BaseAddr = (u32) &sensor_readings[FIFO_out][0];
	DMAinit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAinit.DMA_BufferSize = transfer_size_words * sizeof(u32);
	DMAinit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAinit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAinit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAinit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAinit.DMA_Mode = DMA_Mode_Normal;
	DMAinit.DMA_Priority = DMA_Priority_VeryHigh;
	DMAinit.DMA_Channel = DMA_Channel_2;
	DMAinit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMAinit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAinit.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMAinit.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;

	DMA_Init( DMA2_Stream2,  &DMAinit);

	u16 received_data;
	DMAinit.DMA_Memory0BaseAddr = (u32) &received_data;
	DMAinit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMAinit.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMAinit.DMA_Priority = DMA_Priority_High;
	DMAinit.DMA_Channel = DMA_Channel_3;
	DMA_Init(DMA2_Stream0, &DMAinit);

	//enable DMA
	DMA_Cmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	//Configure DMA interrupt
	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream2, DMA_IT_HT | DMA_IT_TE | DMA_IT_FE, DISABLE);
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream0, DMA_IT_HT | DMA_IT_TE | DMA_IT_FE, DISABLE);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);

	while(!tx_complete){__WFI();}
	while(!rx_complete){__WFI();}

	//TODO wait until SPI is free

	/*while(!tx_complete){
		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
		SPI_I2S_ReceiveData(SPI1);
	}*/


	//receive the last data
	//while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
	//u16 received = SPI_I2S_ReceiveData(SPI1);

	/*FlagStatus status = SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE);
	status = SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE);
	status = SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY);*/

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET){}

	FlagStatus status = SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE);
	status = SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE);
	status = SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY);

	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, DISABLE);
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, DISABLE);

	DMA_Cmd(DMA2_Stream2, DISABLE);
	while(DMA_GetCmdStatus(DMA2_Stream2) == ENABLE ){;}
	DMA_Cmd(DMA2_Stream0, DISABLE);
	while(DMA_GetCmdStatus(DMA2_Stream0) == ENABLE ){;}

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);

	tx_complete = 0;
	rx_complete = 0;

	//update the FIFO state
	if(mem_layout_cont == 1){
		FIFO_out = FIFO_in_transfer;
	}else{
		FIFO_out = 0;
	}

	return SENSOR_DATA_MODE_SUCCESS;
}

void sensor_data_mode_DMA2_Stream2_IRQ(void){
	if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) == SET){
		tx_complete = 1;
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
	}
}

void sensor_data_mode_DMA2_Stream0_IRQ(void){
	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) == SET){
		rx_complete = 1;
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
	}
}

void sensor_data_mode_conf_timer(void){

	//Turn on timer clock

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	//Configuring interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//TIM2 base configuration
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 10000; //trigger at each millisecond
	TIM_TimeBaseStructure.TIM_Prescaler = 10000; //scale the clock
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	//TIM2 is being used to set the pace of samples collection

	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = SENSOR_DATA_MODE_PERIOD; //just one millisecond
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE); //NVIC configuration still required
	//TIM_Cmd(TIM2, ENABLE); enable later to ease synchronization

	//TIM5 is used for timestamping the records
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 100; // scale to microseconds
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

}

s8 sensor_data_mode_set_parameters(void){

	s8 comm_result = SENSOR_DATA_MODE_SUCCESS;
	u16 sensor_selection;

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
	sensor_selection = SPI_I2S_ReceiveData(SPI1);

	// failure if configured for the unimplemented sensors
	// i.e. some sensors other that acc / gyro or no sensor at all
	if (((sensor_selection & (GYRO_MASK | ACCEL_MASK)) == 0 ) ||
			((sensor_selection & ~(GYRO_MASK | ACCEL_MASK)) != 0 ) )
	{
		return SENSOR_DATA_MODE_ERROR;
	}

	if( ( sensor_selection & GYRO_MASK) != 0){
		comm_result = sensor_data_mode_set_gyro();}

	if (comm_result == SENSOR_DATA_MODE_ERROR){
		return SENSOR_DATA_MODE_ERROR;}


	if( ( sensor_selection & ACCEL_MASK) != 0){
		comm_result = sensor_data_mode_set_accel();}

	if (comm_result == SENSOR_DATA_MODE_ERROR){
		return SENSOR_DATA_MODE_ERROR;}

	//confirm that the board is configured

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,PARAMETERS_SET);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
	SPI_I2S_ReceiveData(SPI1); //dummy read


	return SENSOR_DATA_MODE_SUCCESS;
}


s8 sensor_data_mode_set_gyro(void){

	u16 gyro_range, gyro_bw;
	s8 comm_result;

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
	gyro_range = SPI_I2S_ReceiveData(SPI1);

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
	gyro_bw = SPI_I2S_ReceiveData(SPI1);

	//set the data range
	comm_result = bmg160_set_range_reg( (u8) gyro_range );
	if (comm_result == ERROR){
		return SENSOR_DATA_MODE_ERROR;}

	//set both bandwidth and data rate (see bmg160 datasheet)
	comm_result = bmg160_set_bw( (u8) gyro_bw );
	if (comm_result == ERROR){
		return SENSOR_DATA_MODE_ERROR;}

	return SENSOR_DATA_MODE_SUCCESS;

}

s8 sensor_data_mode_set_accel(void){

	u16 accel_range, accel_odr;
	s8 comm_result;

	// power the sensor on
	if ( adxl355_start_sensor() == SENSOR_DATA_MODE_ERROR ){
		return (s8) SENSOR_DATA_MODE_ERROR;}

	//set both bandwidth and data rate (see adxl355 datasheet)

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
	accel_range = SPI_I2S_ReceiveData(SPI1);

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
	accel_odr = SPI_I2S_ReceiveData(SPI1);

	comm_result = adxl355_start_sensor();
	if (comm_result == SENSOR_DATA_MODE_ERROR ){
		return SENSOR_DATA_MODE_ERROR;}

	//set the data range
	comm_result = adxl355_set_range( (u8) accel_range );
	if (comm_result == ERROR){
		return SENSOR_DATA_MODE_ERROR;}

	//set both bandwidth and data rate (see adxl355 datasheet)
	comm_result = adxl355_set_odr( (u8) accel_odr );
	if (comm_result == ERROR){
		return SENSOR_DATA_MODE_ERROR;}

	return SENSOR_DATA_MODE_SUCCESS;
}

void sensor_data_mode_TIM2_IRQ(void){

	//check whether the interrupt is from the first channel
	if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) {

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

		TIM_SetCompare1(TIM2, TIM_GetCapture1(TIM2) +
				SENSOR_DATA_MODE_PERIOD);


		// Get the data from the sensors
		s32 acc_x, acc_y, acc_z;
		//update timestamp
		u32 timestamp = TIM_GetCounter(TIM5);
		adxl355_read_acceleration(&acc_x, &acc_y, &acc_z);

		//perform transmission with a cast
		sensor_data_mode_FIFO_push( (u32) ACCEL_MASK, timestamp,
				(u32) acc_x, (u32) acc_y, (u32) acc_z);

		s16 om_x, om_y, om_z;
		s32 om_x_word, om_y_word, om_z_word;
		//update timestamp
		timestamp = TIM_GetCounter(TIM5);
		bmg160_get_data_X( & om_x);
		bmg160_get_data_Y( & om_y);
		bmg160_get_data_Z( & om_z);

		//convert to a 32 bit format
		om_x_word = (s32) om_x;
		om_y_word = (s32) om_y;
		om_z_word = (s32) om_z;

		sensor_data_mode_FIFO_push((u32) GYRO_MASK, timestamp,
				(u32) om_x_word, (u32) om_y_word, (u32) om_z_word);

		//blink LED for debug
		if(strobe)
		{
			GPIO_SetBits(GPIOC, GPIO_Pin_8);
		}
		else
		{
			GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		}
		strobe = !strobe;

	}
}

s8 sensor_data_mode_FIFO_push(u32 sensor_type, u32 timestamp,
		u32 xdata, u32 ydata, u32 zdata){

	u32 FIFO_new_in = (FIFO_in + 1) %
			SENSOR_DATA_MODE_RECORDS_CAPACITY;

	if (FIFO_new_in == FIFO_out){
		FIFO_err = 1;
		return SENSOR_DATA_MODE_ERROR;
	}

	//save readings
	sensor_readings[FIFO_in][0] = sensor_type;
	sensor_readings[FIFO_in][1] = timestamp;
	sensor_readings[FIFO_in][2] = xdata;
	sensor_readings[FIFO_in][3] = ydata;
	sensor_readings[FIFO_in][4] = zdata;

	FIFO_in = FIFO_new_in;

	return SENSOR_DATA_MODE_SUCCESS;
}


