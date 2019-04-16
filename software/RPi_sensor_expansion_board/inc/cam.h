/*
 * \Name: cam.h
 * \Brief: Configure system to capture frame sync pulses
 *
 */

#ifndef CAM_H_
#define CAM_H_

#include "stm32f4xx.h"

//!
/* \Brief: Setups hardware for capturing frame sync pulses
 * \Return: returns if the setup has been successful
 */
void cam_setup(void);

#endif /* CAM_H_ */
