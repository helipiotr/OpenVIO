/*  \Name: sensor_data_mode.h
 *  \Brief: Mode that allows to collect the sensor values
 */

#ifndef MODES_SENSOR_DATA_MODE_H_
#define MODES_SENSOR_DATA_MODE_H_

#include "comm.h"
#include "bmg160.h"
#include "adxl355.h"
#include "shared.h"

#define SENSOR_DATA_MODE_RECORDS_CAPACITY 		(1000)
#define SENSOR_DATA_MODE_RECORDS_WIDTH 			(5)
#define SENSOR_DATA_MODE_PERIOD					(10)

//! Main function in this operation mode
s8 sensor_data_mode_main(void);

//! Acquire and pass the required configuration to the sensor modules
s8 sensor_data_mode_set_parameters(void);

//! Configure the gyroscope module
s8 sensor_data_mode_set_gyro(void);

//! Configure the accelerometer module
s8 sensor_data_mode_set_accel(void);

//! Configure the timer to timestamp the samples
void sensor_data_mode_conf_timer(void);

//! Transfer readings to the main unit
s8 sensor_data_mode_send_readings(void);

//! Put the readings inside a FIFO
s8 sensor_data_mode_FIFO_push(u32 sensor_type, u32 timestamp,
		u32 xdata, u32 ydata, u32 zdata);

//! Turn off the timers and reset fifo
void sensor_data_mode_shutdown(void);

#endif /* MODES_SENSOR_DATA_MODE_H_ */
