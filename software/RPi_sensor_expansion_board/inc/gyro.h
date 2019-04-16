/* gyro.h
 *  \Brief: Support library for BMG160 driver
 *  \Note: Sets up required hardware and implements functions required for the
 *  interface.
 */

#ifndef GYRO_H_
#define GYRO_H_

#include "bmg160.h"
#include "stm32f4xx_spi.h"
#include "system.h"


#define C_BMG160_TWO_U8X			(2)
#define SPI_BUFFER_LEN 				(5)



/* \Brief: Setups interface for the gyroscope chip
 * \Note: Requires setup of the following peripherals: NVIC priority setting
 * (NVIC_PriorityGroup_0), must be configured prior to use of this function
 * \Return: returns if the setup has been successful
 */
s8 gyro_setup(void);

/*	\Brief: The function is used as I2C bus write
*	\Return : Status of the I2C write
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register, will data is going to be written
*	\param reg_data : It is a value hold in the array,
*		will be used for write the value into the register
*	\param cnt : The no of byte of data to be write
*/
s8 BMG160_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read */
s8 BMG160_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*	\Brief : The delay routine
 *	\param : delay in ms
 *	\Note : can be used only after gyroscope has been initiated, not meant to be
 *	be used outside scope of the gyro driver. Necessary for some configuration modes.
*/
void BMG160_delay_msek(u32 msek);


#endif /* GYRO_H_ */
