# Aeronautics and Rocketry Enterprise at Michigan Technological University

## Keweenaw Propulsion Laboratory — AFC Drone

## Project Goal

The AFC Drone is an autonomous electric test vehicle developed by the Keweenaw Propulsion Laboratory.

The goal of the project is to create a safe and reusable platform for developing technologies that may later be applied to a self-guided rocket. Because testing guidance and control systems on a liquid-fueled rocket presents significant safety and operational risks, the project begins with an electrically powered propeller vehicle.

The drone allows the team to develop and test:

* Radio communication with a ground station
* Flight-computer startup and fault handling
* Sensor initialization and orientation tracking
* Thrust and gimbal control
* Autonomous guidance algorithms
* Telemetry and command protocols
* Hardware-in-the-loop and ground testing procedures

The project is still under active development. The startup architecture and subsystem interfaces are in place, while the full flight-control and periodic telemetry logic are still being integrated.

---

## Code Structure

The flight software is divided into several subsystem-focused modules.

| Module           | Responsibility                                                                                 |
| ---------------- | ---------------------------------------------------------------------------------------------- |
| `main.cpp`       | Arduino entry point and timing for the main control loop                                       |
| `drone.h/.cpp`   | Top-level drone state machine, startup sequence, target selection, and vehicle coordination    |
| `radio.h/.cpp`   | RFM69 initialization, ground-station connection, command reception, and telemetry transmission |
| `gyro.h/.cpp`    | BNO08x IMU initialization and quaternion-to-Euler conversion                                   |
| `gimbal.h/.cpp`  | Gimbal servo control, mechanical correction, and position interpolation                        |
| `error.h/.cpp`   | Error storage and system fault reporting                                                       |
| `debug.h/.cpp`   | USB serial detection and conditional debug output                                              |
| `configs.h/.cpp` | Placeholder for settings that will eventually be stored in nonvolatile memory                  |

The `Drone` class coordinates the individual subsystems and tracks the overall state of the vehicle.

---

## System States

The drone uses a state machine to control startup and operation.

| State          | Description                                                         |
| -------------- | ------------------------------------------------------------------- |
| `BOOT`         | Initializes the controller and checks for a USB serial connection   |
| `RADIO_SETUP`  | Initializes the RFM69 and waits for the ground station              |
| `SENSOR_SETUP` | Initializes the IMU and other sensors                               |
| `READY_ARMED`  | Startup is complete and the vehicle is ready for further commands   |
| `FLIGHT`       | The vehicle is allowed to perform flight operations                 |
| `FAULT_ERROR`  | A startup or runtime error has placed the system into a fault state |

The vehicle advances through these states in order. A failure during radio or sensor initialization moves the system into `FAULT_ERROR`.

---

## Startup Control Flow

The Arduino `setup()` function repeatedly calls `Drone::startup()` until the vehicle reaches `READY_ARMED`.

### 1. Boot and Debug Connection

The flight controller first configures the status LED and checks for a USB serial connection.

The current configuration has `waitForSerial` enabled. This means startup pauses until a serial connection is detected. Serial output is then used for setup messages and debugging.

Persistent configuration storage is planned so this behavior can later be enabled or disabled without rebuilding the firmware.

### 2. Radio Initialization

After the boot checks are complete, the drone enters `RADIO_SETUP`.

The radio setup sequence:

1. Resets the RFM69 module.
2. Initializes the RadioHead driver.
3. Configures the radio for 915 MHz.
4. Applies the shared encryption key.
5. Configures the high-power transmitter for 20 dBm.
6. Sends the `AFCDrone` connection identifier.
7. Waits for an acknowledgment from the ground station.

If an acknowledgment is not received within one second, the connection attempt is retried.

No flight or sensor setup is intended to occur until communication with the ground station has been established.

### 3. Sensor Initialization

After the radio connection is complete, the drone enters `SENSOR_SETUP`.

The current sensor setup initializes the BNO08x inertial measurement unit over I²C and enables the stabilized rotation-vector report at 100 Hz.

Planned or partially implemented initialization tasks include:

* Gyroscope and IMU startup
* Orientation-report configuration
* Gimbal setup and self-test
* Servo actuation checks
* GPS startup and warmup

GPS support is currently planned but is not enabled in the startup sequence.

### 4. Ready State

When all required sensors report that setup is complete, the drone enters `READY_ARMED`.

At this point:

* Radio configuration is complete.
* The ground station has acknowledged the vehicle.
* Required sensors have initialized.
* The controller is ready to begin higher-level operation.

Entering `READY_ARMED` does not automatically begin flight. A separate transition is expected before the vehicle enters the `FLIGHT` state.

---

## Main Control Loop

The main loop is designed to run at approximately **1 kHz**, giving each cycle a target period of 1,000 microseconds.

Each loop is intended to perform the following work:

1. Receive and process radio commands.
2. Read and update sensor data.
3. Run stabilization and guidance calculations.
4. Select the active command target.
5. Update the gimbal and motor outputs.
6. Queue telemetry for the ground station.
7. Update error and status indicators.
8. Record loop timing information.
9. Wait until the 1,000-microsecond loop period has elapsed.

The timing system tracks values such as:

* Last loop execution time
* Worst observed loop time
* Best observed loop time
* Rolling average loop time

The current `Drone::update()` function is a placeholder. The subsystem update calls and flight-control calculations are expected to be added there as development continues.

---

## Command Targets

The current control architecture provides two command target slots.

A command from the ground station can:

* Update one target slot
* Select which target slot is active
* Set the requested gimbal X position
* Set the requested gimbal Y position
* Set two motor-speed targets

This allows the ground station to prepare a new target separately and then switch the active target.

Gimbal commands are transmitted as signed integer values and converted to degrees by the flight controller. The gimbal mechanism currently operates over an approximately ±20-degree commanded range.

The motor and roll-control portions of the command interface are still being completed.

---

## Radio and Telemetry

The drone uses an RFM69 radio operating at 915 MHz.

Normal radio messages contain:

* A packet sequence number
* A message-type identifier
* An 8-byte payload

Defined message categories include:

* General system status
* Gimbal and actuator status
* Orientation quaternion
* Acceleration
* Velocity
* Position
* GPS coordinates
* Ground-station commands
* Configuration data

General system telemetry is intended to report:

* Average loop time
* Maximum loop time
* Vehicle runtime
* Radio signal strength
* Current drone state

Several telemetry messages are defined but are not yet fully populated by the current firmware.

See the Radio API documentation for the detailed packet and payload definitions.

---

## Error Handling

Errors are stored in an internal error buffer. Each error contains:

* An error code
* A severity level

Severity levels are defined as:

| Severity | Meaning           |
| -------: | ----------------- |
|      `0` | Low               |
|      `1` | Medium            |
|      `2` | High              |
|      `3` | Critical or fatal |

Current radio initialization errors include:

* Radio driver initialization failure
* Radio frequency configuration failure

An initialization failure moves the drone into `FAULT_ERROR`.

The intended design is for errors to be reported through every available communication interface, including USB serial and the radio. Radio error reporting and persistent internal logging are still under development.

---

## LED Indicators

The status LED patterns provide a visual indication of the current system state.

| Pattern      |         Timing | State          | Meaning                                                                    |
| ------------ | -------------: | -------------- | -------------------------------------------------------------------------- |
| Blink        |         100 ms | `RADIO_SETUP`  | The controller is initializing the radio or waiting for the ground station |
| Blink        |         300 ms | `SENSOR_SETUP` | Radio setup is complete and sensors are being initialized                  |
| Fade         | 2,000 ms cycle | `READY_ARMED`  | Startup is complete and the vehicle is ready                               |
| Double flash | 1,200 ms cycle | `FLIGHT`       | The vehicle is in a flight-capable operating state                         |
| Rapid blink  |          50 ms | `FAULT_ERROR`  | The controller has detected a fault                                        |

The LED behavior has been defined in the flight software, but the periodic call that updates the indicator is still being integrated into the main control loop.

---

## Hardware Layout

![AFC Drone Flight Computer wiring diagram](docs/front_page/AFC-Drone_Flight_Computer.png)

The wiring diagram was created using KiCad 9.0 and is maintained in the [AFC Wiring Diagram repository](https://github.com/Keweenaw-Propulsion-Laboratory/AFC-Wiring-Diagram).

---

## Current Development Status

The current firmware includes:

* A structured startup state machine
* USB serial detection and debug output
* RFM69 configuration and connection-handshake logic
* BNO08x IMU initialization
* Quaternion-to-Euler conversion
* Gimbal servo control
* Bilinear interpolation for gimbal correction
* Two command target slots
* Message definitions for commands and telemetry
* Error buffering
* Loop-timing infrastructure
* State-based LED patterns

Work still in progress includes:

* Integrating subsystem updates into `Drone::update()`
* Completing motor control
* Completing roll control
* Scheduling telemetry messages
* Processing live IMU data in the main loop
* Implementing command timeouts and failsafe behavior
* Completing GPS support
* Persisting configuration to nonvolatile memory
* Reporting errors over the radio
* Finalizing the transition from `READY_ARMED` to `FLIGHT`
