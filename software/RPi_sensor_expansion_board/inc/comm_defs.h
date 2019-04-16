/*  \Name: comm_defs.h
 *  \Brief: Definitions of values used in the communication implementation
 *  \Note: This lists values shared with the main unit that allow the communication
 */

#ifndef COMM_DEFS_H_
#define COMM_DEFS_H_


// Possible operating modes
#define SENSOR_DATA_MODE			(0)
#define CAMERA_SENSOR_DATA_MODE		(1)
#define NAVIGATION_MODE				(2)

// Communication codes
#define COMM_ERROR 					(0)
#define COMM_ACKNOWLEDGE			(1)
#define PARAMETERS_SET				(2)
#define START_ACQUISITION			(3)
#define STOP_ACQUISITION			(4)
#define SEND_DATA					(5)

// Bit masks of the sensors
#define GYRO_MASK					(1)
#define ACCEL_MASK					(1<<1)
#define MAG_MASK					(1<<2)
#define BAR_MASK					(1<<3)
#define CAM_MASK					(1<<4)

//Capture rate definitions
#define CAPTURE_RATE_100HZ			(10)
#define CAPTURE_RATE_200HZ			(5)
#define CAPTURE_RATE_500HZ			(2)
#define CAPTURE_RATE_1000HZ			(1)


// Copy some of the definitions for the use of the main unit
#ifndef __BMG160_H__
/***********************************************/
/**\name	RANGE DEFINITIONS*/
/**********************************************/
#define BMG160_RANGE_2000	(0x00)
#define BMG160_RANGE_1000	(0x01)
#define BMG160_RANGE_500	(0x02)
#define BMG160_RANGE_250	(0x03)
#define BMG160_RANGE_125	(0x04)

/***********************************************/
/**\name	BANDWIDTH DEFINITIONS*/
/**********************************************/
#define C_BMG160_NO_FILTER_U8X			(0)
#define	C_BMG160_BW_230HZ_U8X			(1)
#define	C_BMG160_BW_116HZ_U8X			(2)
#define	C_BMG160_BW_47HZ_U8X			(3)
#define	C_BMG160_BW_23HZ_U8X			(4)
#define	C_BMG160_BW_12HZ_U8X			(5)
#define	C_BMG160_BW_64HZ_U8X			(6)
#define	C_BMG160_BW_32HZ_U8X			(7)

#define BMG160_BW_500_HZ	(0x01)
#define BMG160_BW_230_HZ	(0x01)
#define BMG160_BW_116_HZ	(0x02)
#define BMG160_BW_47_HZ		(0x03)
#define BMG160_BW_23_HZ		(0x04)
#define BMG160_BW_12_HZ		(0x05)
#define BMG160_BW_64_HZ		(0x06)
#define BMG160_BW_32_HZ		(0x07)


#endif /* __BMG160_H__ */

// Copy some of the definitions for the use of the main unit
#ifndef ADXL355_H_

/***********************************************/
/**\name	BANDWIDTH DEFINITIONS*/
/**********************************************/

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


/***********************************************/
/**\name	RANGE DEFINITIONS*/
/**********************************************/

#define RANGE_2G				 (0x01)
#define RANGE_4G				 (0x02)
#define RANGE_8G				 (0x03)



#endif /* ADXL355_H_ */






#endif /* COMM_DEFS_H_ */
