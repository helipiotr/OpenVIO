/*
 * shared.c
 *
 *  Created on: 07.02.2019
 *      Author: helipiotr
 */

#include "modes/shared.h"

int shared_mutex = EMPTY_MUTVAL;

extern void camera_sensor_data_mode_TIM2_IRQ(void);
extern void sensor_data_mode_TIM2_IRQ(void);
extern void sensor_data_mode_DMA2_Stream2_IRQ(void);
extern void sensor_data_mode_DMA2_Stream0_IRQ(void);
extern void camera_sensor_data_mode_DMA2_Stream2_IRQ(void);
extern void camera_sensor_data_mode_DMA2_Stream0_IRQ(void);
extern void camera_sensor_data_mode_EXTI9_5_IRQ(void);


void TIM2_IRQHandler(void){

	//get to the appropriate function according to the set mode
	switch (shared_mutex){
		case CAMERA_SENSOR_DATA_MODE_MUTVAL:
			camera_sensor_data_mode_TIM2_IRQ();
			return;
		case SENSOR_DATA_MODE_MUTVAL:
			sensor_data_mode_TIM2_IRQ();
			return;
		default:
			return;
	}

}

void DMA2_Stream2_IRQHandler(void){
	//get to the appropriate function according to the set mode
	switch (shared_mutex){
		case CAMERA_SENSOR_DATA_MODE_MUTVAL:
			camera_sensor_data_mode_DMA2_Stream2_IRQ();
			return;
		case SENSOR_DATA_MODE_MUTVAL:
			sensor_data_mode_DMA2_Stream2_IRQ();
			return;
		default:
			return;
	}
}

void DMA2_Stream0_IRQHandler(void){
	//get to the appropriate function according to the set mode
	switch (shared_mutex){
		case CAMERA_SENSOR_DATA_MODE_MUTVAL:
			//not present in the new release
			//camera_sensor_data_mode_DMA2_Stream0_IRQ();
			return;
		case SENSOR_DATA_MODE_MUTVAL:
			sensor_data_mode_DMA2_Stream0_IRQ();
			return;
		default:
			return;
	}
}

void EXTI9_5_IRQHandler(void){
	//get to the appropriate function according to the set mode
	switch (shared_mutex){
		case CAMERA_SENSOR_DATA_MODE_MUTVAL:
			camera_sensor_data_mode_EXTI9_5_IRQ();
			if( EXTI_GetITStatus(EXTI_Line6) == SET ){
				EXTI_ClearITPendingBit(EXTI_Line6);}
			return;
		case SENSOR_DATA_MODE_MUTVAL:
			//no camera in sensor data mode
			return;
		default:
			//program must not land here
			return;
	}
}


s8 shared_lock(int request_source){

	if (shared_mutex == EMPTY_MUTVAL)
	{
		shared_mutex = request_source;
		return MUTEX_SUCCESS;
	}
	else
	{
		return MUTEX_ERROR;
	}

}


s8 shared_free(int request_source){
	if ( (shared_mutex != EMPTY_MUTVAL) && (shared_mutex == request_source) )
	{
		shared_mutex = EMPTY_MUTVAL;
		return MUTEX_SUCCESS;
	}
	else
	{
		return MUTEX_ERROR;
	}

}
