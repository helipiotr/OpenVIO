/*
 * camera_camera_sensor_data_mode.h
 *
 *  Created on: 07.02.2019
 *      Author: helipiotr
 */

#ifndef MODES_CAMERA_SENSOR_DATA_MODE_H_
#define MODES_CAMERA_SENSOR_DATA_MODE_H_

#include "comm.h"
#include "bmg160.h"
#include "adxl355.h"
#include "cam.h"
#include "shared.h"

#define CAMERA_SENSOR_DATA_MODE_RECORDS_CAPACITY 		(5000)
#define CAMERA_SENSOR_DATA_MODE_RECORDS_WIDTH 			(5)
#define CAMERA_SENSOR_DATA_MODE_PULSE_PERIOD			(10)
#define CAMERA_SENSOR_DATA_MODE_PERIOD					(1000)

#define CAMERA_SENSOR_DATA_MODE_FRAME_BEGIN		(1)
#define CAMERA_SENSOR_DATA_MODE_FRAME_END		(0)

//! Type holding the active sensors information
struct active_sens_str {
		int accel;
		int gyro;
		int cam;
};

//! Main function in this operation mode
s8 camera_sensor_data_mode_main(void);

//! Acquire and pass the required configuration to the sensor modules
s8 camera_sensor_data_mode_set_parameters(void);

//! Configure the gyroscope module
s8 camera_sensor_data_mode_set_gyro(void);

//! Configure the accelerometer module
s8 camera_sensor_data_mode_set_accel(void);

//! Configure the camera frame time capture
void camera_sensor_data_mode_set_cam(void);

//! Configure the timer to timestamp the samples
void camera_sensor_data_mode_conf_timer(void);

//! Transfer readings to the main unit
s8 camera_sensor_data_mode_send_readings(void);

//! Put the readings inside a FIFO
s8 camera_sensor_data_mode_FIFO_push(u32 sensor_type, u32 timestamp,
		u32 xdata, u32 ydata, u32 zdata);

//! Turn off the timers and reset fifo
void camera_sensor_data_mode_shutdown(void);

//! Handler for the timer: should be called each millisecond during the capture
void camera_sensor_data_mode_TIM2_IRQ(void);

//! Handler for the DMA interrupts
void camera_sensor_data_mode_DMA2_Stream2_IRQ(void);

#endif /* MODES_CAMERA_SENSOR_DATA_MODE_H_ */
