/*
 * camera_camera_sensor_data_mode.c
 *
 *  Created on: 07.02.2019
 *      Author: helipiotr
 */


/*
 * \Name camera_sensor_data_mode.c
 */

#include "modes/camera_sensor_data_mode.h"
#include "stm32f4xx_dma.h" //reinclude?
#include "stm32f4xx_spi.h"

//failure / success definitions
static const s8 CAMERA_SENSOR_DATA_MODE_SUCCESS = 0;
static const s8 CAMERA_SENSOR_DATA_MODE_ERROR = -1;

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
static u32 volatile sensor_readings[CAMERA_SENSOR_DATA_MODE_RECORDS_CAPACITY]
						   [CAMERA_SENSOR_DATA_MODE_RECORDS_WIDTH];

static int volatile FIFO_in = 0; //shows an empty place for the next record
static int volatile FIFO_out = 0; //current location of the output
static int volatile FIFO_err = 0; //to be checked before the transmission
static int volatile tx_complete = 0;

static int strobe = 0; //TODO remove after tests

//decimate to achieve a certain capture frequency, e.g. 200Hz
static int volatile capture_rate_decimation = 1;

struct active_sens_str active_sensors = {0, 0, 0};

s8 camera_sensor_data_mode_main(void){

	shared_lock( CAMERA_SENSOR_DATA_MODE_MUTVAL );

	//start with setting the parameters of the transmission
	if (camera_sensor_data_mode_set_parameters() == CAMERA_SENSOR_DATA_MODE_ERROR){
		comm_send_error();
		return CAMERA_SENSOR_DATA_MODE_ERROR;
	}

	//configure TIM5 to timestamp data and TIM2 to trigger recording
	camera_sensor_data_mode_conf_timer();

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}

	if (SPI_I2S_ReceiveData(SPI1) != (u16) START_ACQUISITION){
		comm_send_error();
		return CAMERA_SENSOR_DATA_MODE_ERROR;
	}

	//turn on timers as we are starting the measurement
	TIM_SetCounter(TIM2, 0);
	TIM_SetCounter(TIM5, 0);
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM5, ENABLE);

	s8 comm_result;
	while( 1 ){
		//get requested operation

		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
		SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}

		u16 received_data = SPI_I2S_ReceiveData(SPI1);

		if ( received_data == STOP_ACQUISITION){
			//stop regardless the FIFO status
			camera_sensor_data_mode_shutdown();
			return CAMERA_SENSOR_DATA_MODE_SUCCESS;
		} else if (received_data == SEND_DATA && !FIFO_err) {
			//Send a confirmation
			while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
			SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
			while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
			SPI_I2S_ReceiveData(SPI1);

			comm_result = camera_sensor_data_mode_send_readings();
		}else{
			camera_sensor_data_mode_shutdown();
			comm_send_error();
			return CAMERA_SENSOR_DATA_MODE_ERROR;
		}

		//go back in case communication failed
		if(comm_result == CAMERA_SENSOR_DATA_MODE_ERROR){
			camera_sensor_data_mode_shutdown();
			comm_send_error();
			return CAMERA_SENSOR_DATA_MODE_ERROR;
		}

	}

	return CAMERA_SENSOR_DATA_MODE_SUCCESS;
}

void camera_sensor_data_mode_shutdown(void){
	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM5, DISABLE);
	FIFO_in = 0;
	FIFO_out = 0;
	FIFO_err = 0;
	EXTI_DeInit();
	shared_free( CAMERA_SENSOR_DATA_MODE_MUTVAL );
}

s8 camera_sensor_data_mode_send_readings(){

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
				CAMERA_SENSOR_DATA_MODE_RECORDS_WIDTH;
		mem_layout_cont = 1;
	}
	else{
		//Transfer just one block on memory
		transfer_size_words = (CAMERA_SENSOR_DATA_MODE_RECORDS_CAPACITY - FIFO_out)
				* CAMERA_SENSOR_DATA_MODE_RECORDS_WIDTH;
		mem_layout_cont = 0;
		mem_layout_thresh = (CAMERA_SENSOR_DATA_MODE_RECORDS_CAPACITY - FIFO_out)
				*CAMERA_SENSOR_DATA_MODE_RECORDS_WIDTH;
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

	//return when there is nothing to transfer
	if (transfer_size_words == 0){
		return CAMERA_SENSOR_DATA_MODE_SUCCESS;}

	//Configure the SPI to transmit only

	comm_set_SPI_1Line_Tx();

	//Configure the interrupt

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//Configure the DMA
	//Turn on the DMA clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_DeInit(DMA2_Stream2);

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

	//enable DMA
	DMA_Cmd(DMA2_Stream2, ENABLE);

	//Configure DMA interrupt
	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream2, DMA_IT_HT | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME, DISABLE);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

	while(!tx_complete){__WFI();}

	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, DISABLE);

	DMA_Cmd(DMA2_Stream2, DISABLE);
	while(DMA_GetCmdStatus(DMA2_Stream2) == ENABLE ){;}

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

	tx_complete = 0;

	//update the FIFO state
	if(mem_layout_cont == 1){
		FIFO_out = FIFO_in_transfer;
	}else{
		FIFO_out = 0;
	}

	//set the SPI back to bidirectional mode
	comm_set_SPI_2Lines_FullDuplex();

	//send the confirmation that the transfer was successfull
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
	SPI_I2S_ReceiveData(SPI1);


	return CAMERA_SENSOR_DATA_MODE_SUCCESS;
}

void camera_sensor_data_mode_DMA2_Stream2_IRQ(void){
	if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) == SET){
		tx_complete = 1;
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
	}else{
		//the program should never come here: fault handler can be inserted
		FlagStatus status = DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2);
	}
}


void camera_sensor_data_mode_conf_timer(void){

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
	//approx 1 second wrap-around
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = CAMERA_SENSOR_DATA_MODE_PERIOD ;
	TIM_TimeBaseStructure.TIM_Prescaler = 10000; //scale the clock
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	//TIM2 is being used to set the pace of samples collection

	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CAMERA_SENSOR_DATA_MODE_PULSE_PERIOD; //just one millisecond
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

s8 camera_sensor_data_mode_set_parameters(void){

	s8 comm_result = CAMERA_SENSOR_DATA_MODE_SUCCESS;
	u16 sensor_selection;

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET){}
	sensor_selection = SPI_I2S_ReceiveData(SPI1);

	// failure if configured for the unimplemented sensors
	// i.e. some sensors other that acc / gyro /cam or no sensor at all
	if (((sensor_selection & (GYRO_MASK | ACCEL_MASK | CAM_MASK)) == 0 ) ||
			((sensor_selection & ~(GYRO_MASK | ACCEL_MASK | CAM_MASK)) != 0 ) )
	{
		return CAMERA_SENSOR_DATA_MODE_ERROR;
	}

	if( ( sensor_selection & GYRO_MASK) != 0){
		comm_result = camera_sensor_data_mode_set_gyro();}

	if (comm_result == CAMERA_SENSOR_DATA_MODE_ERROR){
		return CAMERA_SENSOR_DATA_MODE_ERROR;}


	if( ( sensor_selection & ACCEL_MASK) != 0){
		comm_result = camera_sensor_data_mode_set_accel();}

	if (comm_result == CAMERA_SENSOR_DATA_MODE_ERROR){
		return CAMERA_SENSOR_DATA_MODE_ERROR;}

	if( (sensor_selection & CAM_MASK) != 0 ){
		camera_sensor_data_mode_set_cam();}

	//get the decimation rate
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,COMM_ACKNOWLEDGE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
	capture_rate_decimation = SPI_I2S_ReceiveData(SPI1);

	//decimation rate soundness check
	if (!((capture_rate_decimation == CAPTURE_RATE_100HZ) ||
		  (capture_rate_decimation == CAPTURE_RATE_200HZ) ||
		  (capture_rate_decimation == CAPTURE_RATE_500HZ) ||
		  (capture_rate_decimation == CAPTURE_RATE_1000HZ))){
		return CAMERA_SENSOR_DATA_MODE_ERROR;
	}

	//confirm that the board is configured

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET){}
	SPI_I2S_SendData(SPI1,PARAMETERS_SET);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
	SPI_I2S_ReceiveData(SPI1); //dummy read


	return CAMERA_SENSOR_DATA_MODE_SUCCESS;
}

void camera_sensor_data_mode_set_cam(void){

	//Interrupt on PB6

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource6);

	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //_Falling
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//clear just to be sure
	//EXTI_ClearITPendingBit(EXTI_Line6);

}

//! An interrupt to save frame sync timestamp
void camera_sensor_data_mode_EXTI9_5_IRQ(void){

	//TODO add the frame number

	if( EXTI_GetITStatus(EXTI_Line6) == SET )
	{


		//get the time
		u32 timestamp = TIM_GetCounter(TIM5);

		FlagStatus bitstatus1 =  SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE);

		FlagStatus bitstatus2 = SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE);

		//send the recorded time to the memory
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == SET ){
			camera_sensor_data_mode_FIFO_push(CAM_MASK, timestamp,
					CAMERA_SENSOR_DATA_MODE_FRAME_BEGIN, 0, 0);
		}else{
			camera_sensor_data_mode_FIFO_push(CAM_MASK, timestamp,
					CAMERA_SENSOR_DATA_MODE_FRAME_END, 0, 0);
		}

	}

	return;
}


s8 camera_sensor_data_mode_set_gyro(void){

	u16 gyro_range, gyro_bw;
	s8 comm_result;

	active_sensors.gyro = 1;

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
		return CAMERA_SENSOR_DATA_MODE_ERROR;}

	//set both bandwidth and data rate (see bmg160 datasheet)
	comm_result = bmg160_set_bw( (u8) gyro_bw );
	if (comm_result == ERROR){
		return CAMERA_SENSOR_DATA_MODE_ERROR;}

	return CAMERA_SENSOR_DATA_MODE_SUCCESS;

}

s8 camera_sensor_data_mode_set_accel(void){

	u16 accel_range, accel_odr;
	s8 comm_result;

	active_sensors.accel = 1;

	// power the sensor on
	if ( adxl355_start_sensor() == CAMERA_SENSOR_DATA_MODE_ERROR ){
		return (s8) CAMERA_SENSOR_DATA_MODE_ERROR;}

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
	if (comm_result == CAMERA_SENSOR_DATA_MODE_ERROR ){
		return CAMERA_SENSOR_DATA_MODE_ERROR;}

	//set the data range
	comm_result = adxl355_set_range( (u8) accel_range );
	if (comm_result == ERROR){
		return CAMERA_SENSOR_DATA_MODE_ERROR;}

	//set both bandwidth and data rate (see adxl355 datasheet)
	comm_result = adxl355_set_odr( (u8) accel_odr );
	if (comm_result == ERROR){
		return CAMERA_SENSOR_DATA_MODE_ERROR;}

	return CAMERA_SENSOR_DATA_MODE_SUCCESS;
}

void camera_sensor_data_mode_TIM2_IRQ(void){

	//check whether the interrupt is from the first channel
	if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) {

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

		//see whether there was wrap-around at the counter

		u32 next_CC = TIM_GetCapture1(TIM2) +
				CAMERA_SENSOR_DATA_MODE_PULSE_PERIOD;

		//equal happens just at the end before counter wraps around
		if (next_CC > CAMERA_SENSOR_DATA_MODE_PERIOD){
			TIM_SetCompare1(TIM2, CAMERA_SENSOR_DATA_MODE_PULSE_PERIOD);
		}else{
			TIM_SetCompare1(TIM2, next_CC);
		}

		//check for the decimation, return if not checked
		if( next_CC % (CAMERA_SENSOR_DATA_MODE_PULSE_PERIOD *
				capture_rate_decimation ) != 0 ){
			return;
		}

		// Get the data from the sensors
		s32 acc_x, acc_y, acc_z;
		//update timestamp
		u32 timestamp = TIM_GetCounter(TIM5);

		if(active_sensors.accel){
			adxl355_read_acceleration(&acc_x, &acc_y, &acc_z);
			//perform transmission with a cast
			camera_sensor_data_mode_FIFO_push( (u32) ACCEL_MASK, timestamp,
					(u32) acc_x, (u32) acc_y, (u32) acc_z);
		}

		s16 om_x, om_y, om_z;
		s32 om_x_word, om_y_word, om_z_word;
		//update timestamp
		timestamp = TIM_GetCounter(TIM5);

		if(active_sensors.gyro){
			bmg160_get_data_X( & om_x);
			bmg160_get_data_Y( & om_y);
			bmg160_get_data_Z( & om_z);

			//convert to a 32 bit format
			om_x_word = (s32) om_x;
			om_y_word = (s32) om_y;
			om_z_word = (s32) om_z;

			camera_sensor_data_mode_FIFO_push((u32) GYRO_MASK, timestamp,
					(u32) om_x_word, (u32) om_y_word, (u32) om_z_word);
		}

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

s8 camera_sensor_data_mode_FIFO_push(u32 sensor_type, u32 timestamp,
		u32 xdata, u32 ydata, u32 zdata){

	u32 FIFO_new_in = (FIFO_in + 1) %
			CAMERA_SENSOR_DATA_MODE_RECORDS_CAPACITY;

	if (FIFO_new_in == FIFO_out){
		FIFO_err = 1;
		return CAMERA_SENSOR_DATA_MODE_ERROR;
	}

	//save readings
	sensor_readings[FIFO_in][0] = sensor_type;
	sensor_readings[FIFO_in][1] = timestamp;
	sensor_readings[FIFO_in][2] = xdata;
	sensor_readings[FIFO_in][3] = ydata;
	sensor_readings[FIFO_in][4] = zdata;

	FIFO_in = FIFO_new_in;

	return CAMERA_SENSOR_DATA_MODE_SUCCESS;
}


