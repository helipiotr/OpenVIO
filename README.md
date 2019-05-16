# OpenVIO - Open visual-inertial navigation hardware

OpenVIO is an open-source hardware project which aims to allow anyone to collect precisely synchronised visual and inertial data. It has been engineered due to the fact that the existing hardware is either too expensive or hard to get, barring anyone interested in this discipline from accesing the raw data. Despite its simplicity and design quirks, it is theoretically able to achieve accuracy and performance on par with the existing VIO solutions. Contributions and issues are welcome.

# Brief description
The system is composed of the following sub-systems:
- Raspberry Pi (version higher than 3) - collects the visual data via the PiCamera. Also, it generates the frame synchronization pulses [RPi_frame_sync](https://www.raspberrypi.org/forums/viewtopic.php?t=190314) by altering the configuration blob so that the Broadcom SoC pins are connected directly to the header.
- An expansion board featuring a STM32 microcontroller, that also has ADXL355 chip and BMG160 gyroscope. Its purpurse is to collect the inertial data in real time and send it Raspberry Pi in lax timing constraints. Alternatively, it can be used as an INS (inertial navigation system) that receives the pose corrections from the Raspberry Pi.
- A camera connected to the Raspberry Pi

The example software that was written with an intent to capture the raw data is included. It consists of camera sensor data mode on the microcontroller side and a program to read and store binary blobs that are being sent from the microcontroller. They can be analised using e.g. octave, python, or any other preferred language, but must be parsed to keep the data types first. Main features:
- Reading sensors in specified interval
- Possible synchronization of the sensor reading times with the image capture intervals
- Versatile sensor configuration
- High performance ADXL355 acceleration sensor (drivers ready)
- BMG160 gyroscope (drivers ready)
- DPS310 barometer
- BMA150 magnetometer

# Getting the hardware
It is possible to either use the attached Altium Designer files or use the generated Gerber files, depending on the preference. If some of the BOM are not easily accessible, please report this in issues and I'll consider replacing the component. Even better, consider contributing to the repository. 

# Programming and development
Before any programming, it is required to re-configure the pinout of the Raspberry Pi as described in [RPi_frame_sync](https://www.raspberrypi.org/forums/viewtopic.php?t=190314). Only after that RPi can be used with the sensor expansion board. GPU memory increase for the camera might be required for capturing the full sensor resolution, this can be done by changing a line in a configuration file.

Regarding the software embedded in STM32, AC6 development software is preffered to be used with the existing project. The microcontroller can be programmed using any STM32F4 compatible programmer. 


# Known issues
- The time synchronization is a bit off, this should be like 1ms but is actually closer to 0.991ms. Likely a timer issue.
- The DMA that is being used for the communication between Raspberry Pi and STM32 is failing quite routinely. Maybe a change in protocol / communication paradigm (DMA timeouts?) will bring this to an end.
- As DMA quite routinely fails, this messes up the timing data completely. Please don't be surprised if the saved binary blob contains nonsense, very often acceleration / angular velocity data is just fine with timing being just garbage.
- Gyroscope in hardware version 1.2 is not yet supported, but the library can be easily added from the Bosch Sensortronic website. 

# License
All hardware must not be used for any activity other than academic or educational due to the licensing. Software that contains a copyright notice from Bosch Sensortec GmbH has to be distributed within the limits of this license. All other components are subject to the MIT license. 

