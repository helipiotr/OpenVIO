/** \file adxl355.h
*   \Brief: Header for ADXL355 API
*   \Note: This file is intended to enable platform - independent ADXL355 support
*/


#ifndef ADXL355_H_
#define ADXL355_H_

#include "stm32f4xx.h"

/** \Name: ADXL355 registers addresses */

#define DEVID_AD                 (0x00)
#define DEVID_MST                (0x01)
#define PARTID                   (0x02)
#define REVID                    (0x03)
#define STATUS                   (0x04)
#define FIFO_ENTRIES             (0x05)
#define TEMP2                    (0x06)
#define TEMP1                    (0x07)
#define XDATA3                   (0x08)
#define XDATA2                   (0x09)
#define XDATA1                   (0x0A)
#define YDATA3                   (0x0B)
#define YDATA2                   (0x0C)
#define YDATA1                   (0x0D)
#define ZDATA3                   (0x0E)
#define ZDATA2                   (0x0F)
#define ZDATA1                   (0x10)
#define FIFO_DATA                (0x11)
#define OFFSET_X_H               (0x1E)
#define OFFSET_X_L               (0x1F)
#define OFFSET_Y_H               (0x20)
#define OFFSET_Y_L               (0x21)
#define OFFSET_Z_H               (0x22)
#define OFFSET_Z_L               (0x23)
#define ACT_EN                   (0x24)
#define ACT_THRESH_H             (0x25)
#define ACT_THRESH_L             (0x26)
#define ACT_COUNT                (0x27)
#define FILTER                   (0x28)
#define FIFO_SAMPLES             (0x29)
#define INT_MAP                  (0x2A)
#define SYNC                     (0x2B)
#define RANGE                    (0x2C)
#define POWER_CTL                (0x2D)
#define SELF_TEST                (0x2E)
#define RESET_REG                (0x2F)

/** \Name: register function bit masks */
#define POWER_CTL_MODE_MASK		 (0x01)
#define FILTER_ODR_MASK			 (0x0F)
#define RANGE_MASK				 (0x03)

/** \Name: configuration values of the registers */
#define POWER_CTL_MEASUREMENT	 (0x00)
#define POWER_CTL_STANDBY	 	 (0x01)

#define FILTER_ODR_4000 		 (0x00)
#define FILTER_ODR_2000			 (0x01)
#define FILTER_ODR_1000 		 (0x02)
#define FILTER_ODR_500  		 (0x03)
#define FILTER_ODR_250			 (0x04)
#define FILTER_ODR_125			 (0x05)
#define FILTER_ODR_62 			 (0x06)
#define FILTER_ODR_32			 (0x07)
#define FILTER_ODR_16			 (0x08)
#define FILTER_ODR_8			 (0x09)
#define FILTER_ODR_4			 (0x0A)

#define RANGE_2G				 (0x01)
#define RANGE_4G				 (0x02)
#define RANGE_8G				 (0x03)


/** \Name: auxiliary definitions */

#define ADXL355_ERROR							((s8)-1)
#define ADXL355_SUCCESS							((s8)0)
#define	ADXL355_GEN_READ_WRITE_DATA_LENGTH		((u8)1)
#define ADXL355_ACCEL_READ_WRITE_DATA_LENGTH	((u8)3)
#define	ADXL355_INIT_VALUE						((u8)0)

/** \Brief: Read and write function pointer definitions
*   \Note: format: device address, address on device, source/destination,
*   data length.
*/
#define ADXL355_WR_FUNC_PTR s8 (*bus_write) (u8, u8, u8 *, u8)
#define ADXL355_RD_FUNC_PTR s8 (*bus_read) (u8, u8, u8 *, u8)
#define ADXL355_BRD_FUNC_PTR s8 (*burst_read) (u8, u8, u8 *, u32)


/**
*	\Brief: adxl355_t structure
*	\Note: This structure holds all relevant information about adxl355
*/
struct adxl355_t {
	u8 chip_id;/**< chip id of ADXL355 */
	u8 dev_addr;/**< device address of ADXL355 */

	ADXL355_BRD_FUNC_PTR;/**< burst read function pointer of BMG160 */
	ADXL355_WR_FUNC_PTR;/**< bus write function pointer of BMG160 */
	ADXL355_RD_FUNC_PTR;/**< bus read function pointer of BMG160 */
	//void (*delay_msec)(ADXL355_MDELAY_DATA_TYPE);
	/**< delay function pointer of ADXL355 */
};



/*!
*	\Brief: This function is used for initialize
*	the bus read and bus write functions
*   and assign the chip id read in the register 0x00
*
*	\param: adxl355 structure pointer.
*
*	\Note: While changing the parameter of the adxl355_t
*	consider the following point:
*	\Note: Changing the reference value of the parameter
*	will changes the local copy or local reference
*	make sure your changes will not
*	affect the reference value of the parameter
*	(Better case don't change the reference value of the parameter)
*
*	\Return: results of bus communication function
*	\Retval: 0 -> Success
*	\Retval: -1 -> Error
*/

s8 adxl355_init(struct adxl355_t *ext_adxl355);

/*!
 * \Brief: Changes the power mode to measurement
 * \Return: results of bus communication function
 * \Retval: 0 -> Success
 * \Retval: -1 -> Error
 */
s8 adxl355_start_sensor(void);

/*!
 * \Brief: Changes the power mode to standby
 * \Return: results of bus communication function
 * \Retval: 0 -> Success
 * \Retval: -1 -> Error
 */
s8 adxl355_stop_sensor(void);

/*!
 * \Brief: Sets the output data rate and bandwidth
 * \Return: results of bus communication function
 * \Retval: 0 -> Success
 * \Retval: -1 -> Error
 */
s8 adxl355_set_odr(u8 odr);

/*!
 * \Brief: Sets the range of the sensor
 * \Return: results of bus communication function
 * \Retval: 0 -> Success
 * \Retval: -1 -> Error
 */
s8 adxl355_set_range(u8 range);

/*!
 * \Brief: Reads one of the acceleration registers
 * \Param: the first acceleration register
 * \Param: pointer to the value to be written with
 * with acceleration data
 * \Return: results of bus communication function
 * \Retval: 0 -> Success
 * \Retval: -1 -> Error
 */
s8 adxl355_read_accel_reg( u8 first_register , s32 * acceleration);

/*!
 * \Brief: Reads one of the acceleration registers
 * \Param: Three pointers to memory locations to be written
 * \Return: results of bus communication function
 * \Retval: 0 -> Success
 * \Retval: -1 -> Error
 */
s8 adxl355_read_acceleration(s32* acc_x, s32* acc_y, s32* acc_z);

//TODO remove in final release
void adxl355_debug(void);


#endif /* ADXL355_H_ */
