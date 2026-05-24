# Aeronautics and Rocketry Enterprise at Michigan Technological University
## Keeweenaw Propulsion Lab - AFC Drone

## Goal
The goal of this project is to serve as a test platform for a self guided rocket. Due to the inherent risks involved with liquid rocket fuel we felt it would be safer to start with an electric propellor test vehicle. 

## Code Structure


## Control Loop
### 1. Start radio initialization and wait for connection with ground station. 
- In order to ensure the safety of personel and the vehicle no actions will occur until either radio communications or serial communication has been established. 

### 2. Sensors and Mechanism Initialization
- Once comms have been established and the opperator gives the go ahead, sensors and mechanisms will begin to initialize.

* Initializations include,
    - Gyro startup and calibration
    - Gimbal self test and accuation
    - ~~GPS startup and warmup~~

### Errors
Any errors of sufficient severity will cause the system to halt initialization and broadcast an error message over any connected comms. 

## LED Indicators

The LED status indicator light (undefined) can be used to determine
what state the vehicle is currently in. 

Pattern | Interval | State | Meaning
--------|-----------|--------|----
Blink | 100ms | RADIO_SETUP | The controller is currently initializing the radio and waiting for a connection from the BaseStation
Blink | 300ms | SENSOR_SETUP | The controller has made a connection to the BaseStation. It is now setting up sensors such as IMU and GPS.
Fade | 2000ms | READY_ARMED | The vehicle has completed initial setup and is in a ready state. 
Double Blink | 1200ms | FLIGHT | The vehicle is in a state that allows flight. This could include a ready for take off or landing state. 
Blink | 50ms | FAULT_ERROR | The controller has detected an error. 