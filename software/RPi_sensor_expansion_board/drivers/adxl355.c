/*! \file adxl355.c
*   \Brief: Driver for BMG160
*/


#include "adxl355.h"

// pointer to the struct defining the interface
static struct adxl355_t * p_adxl355;

s8 adxl355_init(struct adxl355_t *ext_adxl355)
{
	// variable used to return the bus communication status
	s8 comres = ADXL355_ERROR;
	//Assign zero to check if it gets overwritten with 0xAD
	u8 v_data_u8  = 0;
	// Assign the initial function pointers
	p_adxl355 = ext_adxl355;
	/*Read Analog Devices ID */
	comres = p_adxl355->bus_read(p_adxl355->dev_addr,
			DEVID_AD, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);
	p_adxl355->chip_id = v_data_u8;
	return comres;
}

s8 adxl355_start_sensor(void)
{
	s8 comres = ADXL355_ERROR;
	u8 v_data_u8 = 0;

	// see the current operating mode
	comres = p_adxl355->bus_read(p_adxl355->dev_addr,
			POWER_CTL, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);

	if (comres == ADXL355_ERROR){
		return ADXL355_ERROR;}

	//set the bits related to the measurement
	v_data_u8 = ( (~POWER_CTL_MODE_MASK) & v_data_u8) | POWER_CTL_MEASUREMENT;
	comres = p_adxl355->bus_write(p_adxl355->dev_addr,
			POWER_CTL, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);

	// begin debug

	//comres = p_adxl355->bus_read(p_adxl355->dev_addr,
	//		POWER_CTL, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);

	//v_data_u8 = 0;
	//end debug

	return comres;
}

s8 adxl355_stop_sensor(void)
{
	s8 comres = ADXL355_ERROR;
	u8 v_data_u8 = 0;

	// see the current operating mode
	comres = p_adxl355->bus_read(p_adxl355->dev_addr,
			POWER_CTL, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);

	if (comres == ADXL355_ERROR){
		return ADXL355_ERROR;}

	//set the bits related to the standby
	v_data_u8 = ( (~POWER_CTL_MODE_MASK) & v_data_u8) | POWER_CTL_STANDBY;
	comres = p_adxl355->bus_write(p_adxl355->dev_addr,
			POWER_CTL, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);

	return comres;
}

s8 adxl355_set_odr(u8 odr)
{
	s8 comres = ADXL355_ERROR;
	u8 v_data_u8 = 0;

	// see the current operating mode
	comres = p_adxl355->bus_read(p_adxl355->dev_addr,
			FILTER, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);

	if (comres == ADXL355_ERROR){
		return ADXL355_ERROR;}

	//make sure the user doesn't write to any other bits
	odr = odr & FILTER_ODR_MASK;

	//set the bits related to the measurement
	v_data_u8 = ( (~FILTER_ODR_MASK) & v_data_u8) | odr;
	comres = p_adxl355->bus_write(p_adxl355->dev_addr,
			FILTER, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);

	return comres;
}

s8 adxl355_set_range(u8 range)
{
	s8 comres = ADXL355_ERROR;
	u8 v_data_u8 = 0;

	// see the current operating mode
	comres = p_adxl355->bus_read(p_adxl355->dev_addr,
			RANGE, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);

	if (comres == ADXL355_ERROR){
		return ADXL355_ERROR;}

	//make sure the user doesn't write to any other bits
	range = range & RANGE_MASK;

	//set the bits related to the measurement
	v_data_u8 = ( (~RANGE_MASK) & v_data_u8) | range;
	comres = p_adxl355->bus_write(p_adxl355->dev_addr,
			RANGE, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);

	return comres;
}

s8 adxl355_read_acceleration(s32* acc_x, s32* acc_y, s32* acc_z)
{
	s8 comres;
	comres = adxl355_read_accel_reg( XDATA3, acc_x );
	if (comres == ADXL355_ERROR){
		return ADXL355_ERROR;}
	comres = adxl355_read_accel_reg( YDATA3, acc_y );
	if (comres == ADXL355_ERROR){
		return ADXL355_ERROR;}
	adxl355_read_accel_reg( ZDATA3, acc_z );
	if (comres == ADXL355_ERROR){
		return ADXL355_ERROR;}

	return ADXL355_SUCCESS;
}

s8 adxl355_read_accel_reg( u8 first_register , s32 * acceleration)
{
	//holds the value of the registers
	u8 data[3];
	s8 comres;

	comres = p_adxl355->bus_read(p_adxl355->dev_addr,
			first_register, data, ADXL355_ACCEL_READ_WRITE_DATA_LENGTH);
	if (comres == ADXL355_ERROR){
		return ADXL355_ERROR;}

	u32 sensor_data;
	sensor_data =  (data[0] << 12 ) | (data[1] << 4 ) | (data[2] >> 4);

	const u32 accelation_negative_bit = (1<<19); // 20-bit negative bit in 2's complement

	if( (sensor_data & accelation_negative_bit )  == accelation_negative_bit ){
		//or with singed 32 int bit mask
		*acceleration = (sensor_data | 0xFFF00000);
	}else{
		*acceleration = sensor_data;
	}

	return ADXL355_SUCCESS;
}

//TODO remove at final relase
void adxl355_debug(void)
{
	u8 v_data_u8;

	p_adxl355->bus_read(p_adxl355->dev_addr,
			POWER_CTL, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);
	v_data_u8 = 0; //dbg

	p_adxl355->bus_read(p_adxl355->dev_addr,
			RANGE, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);
	v_data_u8 = 0; //dbg

	p_adxl355->bus_read(p_adxl355->dev_addr,
				PARTID, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);
	v_data_u8 = 0; //dbg

	p_adxl355->bus_read(p_adxl355->dev_addr,
			DEVID_MST, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);
	v_data_u8 = 0; //dbg

	p_adxl355->bus_read(p_adxl355->dev_addr,
			DEVID_AD, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);
	v_data_u8 = 0; //dbg

	s8 comres = p_adxl355->bus_read(p_adxl355->dev_addr,
			FILTER, &v_data_u8, ADXL355_GEN_READ_WRITE_DATA_LENGTH);
	v_data_u8 = 0; //dbg


}
