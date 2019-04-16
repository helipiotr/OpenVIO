/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "system.h"
#include "gyro.h"
#include "accel.h"
#include "cam.h"
#include "comm.h"
#include "modes/sensor_data_mode.h"
#include "modes/camera_sensor_data_mode.h"

void NVIC_Conf(void){
	//Setting global priority group
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0 );
}


int main(void)
{

	//setting the main quartz oscillator and PLL
	RCC_Conf();

	NVIC_Conf(); // NVIC requires clocked peripherals

	gyro_setup();
	accel_setup();
	cam_setup();
	comm_setup();

	GPIO_InitTypeDef portC;

	portC.GPIO_Pin = GPIO_Pin_8;
	portC.GPIO_Mode = GPIO_Mode_OUT;
	portC.GPIO_OType = GPIO_OType_PP;
	portC.GPIO_PuPd = GPIO_PuPd_NOPULL;
	portC.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &portC);

	//main loop waiting and redirecting to the proper communication mode
	while(1){

		u8 required_mode = comm_get_mode();
		s8 operation_result = 0;

		 switch (required_mode) {
		   case SENSOR_DATA_MODE:
			   GPIO_SetBits(GPIOC, GPIO_Pin_8);
			   //sensor_data_mode_set_parameters();
			   operation_result = sensor_data_mode_main();
			   //insert code here
		     break;
		   case CAMERA_SENSOR_DATA_MODE:
			   GPIO_SetBits(GPIOC, GPIO_Pin_8);
			   operation_result = camera_sensor_data_mode_main();
			   //to be used after a proper module is written
		     break;
		   default:
			   GPIO_ResetBits(GPIOC, GPIO_Pin_8);
			   comm_send_error();
		     break;
		 }
		/*GPIO_SetBits(GPIOC, GPIO_Pin_8);
		BMG160_delay_msek(500);
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		BMG160_delay_msek(500);*/
	}
}



