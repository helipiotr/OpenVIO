#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "RPi_camera_sensor_data_mode.h"

//hardcoded for the time being, can be modified with an optarg
static const char *device = "/dev/spidev0.0";
	
static uint8_t bits = 8; //maximum number allowed by the driver
static uint32_t speed = 100000;
static uint32_t spi_max_words = 1024;

//a default value of 10 seconds
static unsigned int record_time = 10;
static unsigned int default_delay_us = 300;

int main(int argc, char * argv[])
{	
	parse_opts(argc, argv);
	
	//int bmg_range_dummy = BMG160_RANGE_250;
	//printf("BMG160_RANGE_250 value is: %d \n", bmg_range_dummy);
	
	int fd;
	fd = open(device, O_RDWR);
	set_spi(fd);
	uint8_t tx = CAMERA_SENSOR_DATA_MODE;
	
	//set the operating mode to sensor data mode
	
	uint8_t rx = send_receive_single( 
		fd, tx, default_delay_us);
		
	if (rx == COMM_ERROR)
		pabort("Can't set peripheral board operation mode");
	
	//select the sensors
	uint8_t sensor_selection = GYRO_MASK | ACCEL_MASK | CAM_MASK;
	tx = sensor_selection;  
	rx = send_receive_single(fd, tx, default_delay_us);
	if (rx == COMM_ERROR)
		pabort("Wrong sensor selection");
	
	//set up the gyro 
	if (sensor_selection & GYRO_MASK){
		//set range -> lowest as we deal with indoor navigation
		tx = BMG160_RANGE_125;
		rx = send_receive_single(fd, tx, default_delay_us);
		if (rx == COMM_ERROR)
			pabort("Gyro range selection error");
		
		//set bandwidth -> around capture value, 100 Hertz
		tx = BMG160_BW_116_HZ;
		rx = send_receive_single(fd, tx, default_delay_us);
		if (rx == COMM_ERROR)
			pabort("Gyro bandwidth selection error");
		
		printf("Gyro has been configured \n");
	}
	
	//set up the accelerometer
	
	if(sensor_selection & ACCEL_MASK)
	{
		//set accelerometer range
		tx = RANGE_2G;
		rx = send_receive_single(fd, tx, default_delay_us);
		if (rx == COMM_ERROR)
			pabort("Accelerometer range selection error");
		
		//set output data rate
		tx = FILTER_ODR_500;
		rx = send_receive_single(fd, tx, default_delay_us);
		if (rx == COMM_ERROR)
			pabort("Accelerometer ODR selection error");
		
		printf("Accelerometer has been configured \n");
	
	}
	
	//the camera sensor requires no configuration in this release
	
	//confirm the setup
	tx = 0;
	rx = send_receive_single(fd, tx, default_delay_us);
	if (rx != PARAMETERS_SET)
		pabort("Sensor configuration error");
	
	printf("The board has been succesfully configured!\n");
	
	//start the timers
	tx = START_ACQUISITION;
	rx = send_receive_single(fd, tx, default_delay_us); //TODO change delay
	if (rx != COMM_ACKNOWLEDGE)
		pabort("Timers can't have been set");
	
	printf("Timers and the measurement started\n");
	
	//wait for some time to gather data
	//usleep(10000);
	//usleep(10000000);
	
	//sleep period 100ms
	uint32_t sleep_period = 30000;
	usleep(sleep_period);
	
	//receive data length, receive the stored data
	
	uint8_t rx2[2];
	
	uint8_t * rx_ptr;
	uint8_t * tx_ptr;
	FILE * write_ptr;
	
	write_ptr = fopen("measurements.bin","wb");
	
	if(write_ptr == NULL)
	{
		pabort("can't open a file to write");
	}
	
	//Perform multiple data transfers with a delay
	for (int i = 0; i < (record_time*1000000/sleep_period); ++i){
		
		//request the data transfer
		tx = SEND_DATA;
		rx = send_receive_single(fd, tx, default_delay_us); 
		//printf("rx: %d \n", rx);
		if (rx != COMM_ACKNOWLEDGE)
			pabort("Wrong acknowledge to send the data \n ");
		
		//see if we do not have an overflow of the FIFO
		tx = 0;
		rx = send_receive_single(fd, tx, default_delay_us);
		if (rx != COMM_ACKNOWLEDGE)
			pabort("Error, likely a FIFO overflow");
		
		tx = 0;
		rx2[0] = send_receive_single(fd, tx, default_delay_us);
		rx2[1] = send_receive_single(fd, tx, default_delay_us);
		
		uint16_t transfer_size_words = (rx2[1] << 8 ) | rx2[0];
		
		//TODO create multiple transfers in case the there is too much data
		
		//printf("The transfer size (number of u32) is %d \n",  transfer_size_words);
		
		if(transfer_size_words > 0){

			//read the records
			rx_ptr = malloc(sizeof(uint32_t) * transfer_size_words);
			
			//just to prevent segfault, doesn't really matter 
			tx_ptr = malloc(sizeof(uint32_t) * transfer_size_words);
			
			//wait for some time for the microcontroller DMA to prepare
			
			//usleep(sleep_period);
			
			//transfer part-by-part
			uint32_t transfer_mult = transfer_size_words / spi_max_words;
			uint32_t transfer_rem = transfer_size_words % spi_max_words;
			
			//send blocks of max-size
			for (int i = 0 ; i < transfer_mult; ++i){
			
				struct spi_ioc_transfer tr = {
					.tx_buf = (unsigned long) (tx_ptr + i*spi_max_words),
					.rx_buf = (unsigned long) rx_ptr,
					.len = (sizeof(uint32_t) * spi_max_words), //nb of bytes
					.delay_usecs = default_delay_us,
					.speed_hz = 10000000,
					.bits_per_word = bits,
				};
				
				//printf("Starting fast data transfer \n");
				int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
				if (ret < 1)
					pabort("can't send spi message");
			
			}
			
			struct spi_ioc_transfer tr = {
				.tx_buf = (unsigned long) (tx_ptr + transfer_mult * spi_max_words),
				.rx_buf = (unsigned long) rx_ptr,
				.len = (sizeof(uint32_t) * transfer_rem), //nb of bytes
				.delay_usecs = default_delay_us,
				.speed_hz = 10000000,
				.bits_per_word = bits,
			};
			
			//printf("Starting fast data transfer \n");
			int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
			if (ret < 1)
				pabort("can't send spi message");
			
			fwrite(rx_ptr,sizeof(uint32_t), transfer_size_words, write_ptr);
			free(tx_ptr);
			free(rx_ptr);
				

			
			//confirm the transfer
			tx = COMM_ACKNOWLEDGE;
			rx = send_receive_single(fd, tx, default_delay_us); //TODO change delay
			if (rx != COMM_ACKNOWLEDGE){

				printf("Transfer confirmation message invalid: %02X, ignored \n", rx);
				//the DMA on the microcontroller sometimes "shifts" by one byte and needs to be pushed forward
				//no idea why this happens, might be something hardware-related. It however is certain that
				//this workaround (sending additional byte) does not corrupt the data (the number of bytes written to
				//the file is correct, so is the actual data).
				tx = COMM_ACKNOWLEDGE;
				rx = send_receive_single(fd, tx, default_delay_us); //TODO change delay
			}
			
			
		} else {
			printf("Transfer size is zero, no bytes transmitted. \n");
			//usleep(5*sleep_period);
		}
		
		usleep(sleep_period);
	}
	


	
	// Stop the acquisition 
	
	tx = STOP_ACQUISITION;
	rx = send_receive_single(fd, tx, default_delay_us);
	if (rx != COMM_ACKNOWLEDGE)
		pabort("Can't stop acquisistion");
	
	printf("Acquisistion stopped \n");
	
	fclose(write_ptr);
	close(fd);
	
	return 0;
}

void pabort(const char *s)
{
	perror(s);
	abort();
}

//void send_receive_multiple(int fd, uint8_t * tx, uint8_t * rx,
//		uint16_t records_number,  uint16_t delay)

uint8_t send_receive_single(int fd, uint8_t tx, uint16_t delay)
{
	uint8_t rx;
	int ret;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long) &tx,
		.rx_buf = (unsigned long) &rx,
		.len = 1, //just one byte
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
	
	return rx;
}

void parse_opts(int argc, char * argv[])
{
	while (1)
	{
		static const struct option lopts[] = {
			{"record_time", 1, 0, 't'},
			{NULL, 0, 0, 0},
			//does effectively nothing but does
			//a splendid job not allowing for a segfault				
		};
		
		int c;
		
		c = getopt_long(argc, argv, "t:", lopts, NULL);
		
		//only way to exit this loop
		if ( c == -1)
			break;
		
		switch (c) {
			case 't':
				record_time = atoi(optarg);
				printf("Set operation time %d seconds \n",record_time);
				break;
			default:
				print_usage();
				break;
		}
	}
}

void print_usage(void)
{
	printf("Usage: [-t]");
	puts("-t --record_time \t time of required operation");
	
	exit(1);
}

void set_spi(int fd)
{
	// set the SPI operating mode
	
	int ret = 0;
	uint8_t mode = 0; //can be ORed in the future
	
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");
	
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");
	
	// set the number of bits to be transferred

	
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");
	
	//set the speed of the SPI
	
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");
	
	
	//print the information about the SPI configuration
	printf("Spi mode: %d\n", mode);
	printf("Bits per word: %d\n", bits);
	printf("Max speed: %d Hz (%d KHz)\n", speed, speed/1000);
	
}


