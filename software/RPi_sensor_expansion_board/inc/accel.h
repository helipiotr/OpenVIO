/*  \Name: accel.h
 *  \Brief: Support library for ADXL355 driver
 *  \Note: Sets up required hardware and implements functions required for the
 *  interface.
 */

#ifndef ACCEL_H_
#define ACCEL_H_

#include "adxl355.h"

//!
/* \Brief: Setups interface for the accelerometer chip
 * \Note: Might require setup of NVIC priority setting (NVIC_PriorityGroup_0)
 *  when used in synchronization with gyroscope
 * \Return: returns if the setup has been successful
 */
s8 accel_setup(void);


/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write*/
s8 ADXL355_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);


/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read */
s8 ADXL355_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);


#endif /* ACCEL_H_ */
