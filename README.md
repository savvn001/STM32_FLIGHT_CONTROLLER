# STM32_FLIGHT_CONTROLLER
Open source flight controller running on an STM32F411RE MCU and a custom designed PCB. Project written in STM32cubeIDE. 

The other modules this flight controller currently uses:

- NRF24L01PA for RF comms
- GPS is provided by a uBlox NEO6M GPS module
- Orientation from an MPU9250 9-axis IMU.

Current features:
- User control
- Packetised RF communications
- GPS 
- In flight monitoring of telemetry information (battery level, orientation, GPS co-ordinates etc.)
- More to come!


The first revision of the flight controller designed in eagle is shown below.

![Alt text](pic2.jpg?raw=true "Flight controller PCB")


The controller features headers for I2C, 4 GPIO pins, UART and an NRF24L01 module. The code for the associated user controller/transmitter is here: https://github.com/savvn001/DRONE_CONTROLLER_F4746NG
