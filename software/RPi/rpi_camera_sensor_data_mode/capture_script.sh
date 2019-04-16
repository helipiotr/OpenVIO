#!/bin/bash


#please bear in ming that the video is likely not to begin from the first frame,
#hence for correct time synchronization frame number must be read from the image
# (annotation)

#taskset is required as raspivid process would likely stall sensor data collection and
#overflow the peripheral FIFO

taskset 0x1 raspivid --framerate 10 \
	--width 3280 \
	--height 2464 \
	--output video.mjpg \
	--codec MJPEG \
	--timeout 30000 \
	--verbose \
	--nopreview \
	--annotate 512 \
	--shutter 1000\
	& taskset 0x2 ./RPi_camera_sensor_data_mode.bin -t 30
