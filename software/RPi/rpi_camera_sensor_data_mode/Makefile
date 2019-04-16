RPi_sensor_data_mode: RPi_camera_sensor_data_mode.o
	gcc -g RPi_camera_sensor_data_mode.o -o RPi_camera_sensor_data_mode.bin
RPi_sensor_data_mode.o: RPi_camera_sensor_data_mode.c comm_defs.h
	gcc -g -c RPi_camera_sensor_data_mode.c 
clean:
	$(RM) *.o RPi_camera_sensor_data_mode.bin