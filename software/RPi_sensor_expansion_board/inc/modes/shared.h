/*
 * \Name shared.h
 * \Brief use for resources shared among many operation modes
 */

#ifndef MODES_SHARED_H_
#define MODES_SHARED_H_

#include "stm32f4xx.h"

#define EMPTY_MUTVAL 						(0)
#define CAMERA_SENSOR_DATA_MODE_MUTVAL		(1)
#define SENSOR_DATA_MODE_MUTVAL				(2)

#define MUTEX_SUCCESS						(0)
#define MUTEX_ERROR							(-1)

s8 shared_lock(int request_source);
s8 shared_free(int request_source);


#endif /* MODES_SHARED_H_ */
