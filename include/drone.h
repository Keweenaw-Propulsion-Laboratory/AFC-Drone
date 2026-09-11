#pragma once

#include <Arduino.h>
#include <cstdint>

namespace Drone {

    struct Target_t {
        int16_t gimbalX;
        int16_t gimbalY;
        uint8_t bottomMotor;
        uint8_t topMotor;
    };

    enum class States: uint8_t {
        BOOT, 
        RADIO_SETUP,  
        SENSOR_SETUP,
        CONTROL_SETUP,
        READY_ARMED,
        FLIGHT,
        FAULT_ERROR
    };

    /**
     * One coherent snapshot of everything the control tick can see, captured
     * by the control ISR at CONTROL_LOOP_HZ. loop() copies it with
     * getTelemetry() and transmits it at its own, much slower, cadence.
     *
     * Only ISR-visible state lives here. Quantities owned by loop()-context
     * code - radio RSSI, battery voltage, GPS - are read live by the senders
     * instead, because the control tick never touches them.
     *
     * The timing fields describe the PREVIOUS tick: they are measured around
     * the call to update(), so the snapshot update() takes carries the last
     * completed measurement. At 1 kHz that is 1 ms stale.
     */
    struct Telemetry_t {
        // Control loop timing, in microseconds
        uint16_t loopTimeLast;
        uint16_t loopTimeAvg;
        uint16_t loopTimeMax;
        uint16_t loopTimeMin;
        uint32_t missedTicks;   // Ticks where update() overran CONTROL_LOOP_US
        uint32_t runtimeSec;
        States   state;

        // Gimbal setpoints and raw servo commands, in degrees
        float   gimbalPitch;
        float   gimbalYaw;
        int16_t topServoSet;
        int16_t bottomServoSet;

        // Motor setpoints, 0-255
        uint16_t topMotorSet;
        uint16_t bottomMotorSet;

        // Orientation quaternion in the drone body frame
        float qR, qI, qJ, qK;

        // World frame
        float accelX, accelY, accelZ;
        float velX, velY, velZ;
        float posX, posY, posZ;
    };

    //MARK: Helpers

    /**
     * @returns The current state of the drone
     */
    States getState();

    uint16_t getLastLoopTime();
    uint16_t getWorstTime();
    uint16_t getBestTime();
    uint16_t getRollAvg();
    uint32_t getMissedTicks();

    /**
     * Copies the most recent control-tick snapshot into @p out.
     *
     * Take ONE snapshot per telemetry frame and hand it to every sender, so
     * all the packets in that frame describe the same instant.
     *
     * @warning Briefly disables interrupts to copy the struct atomically with
     * respect to the control ISR. The copy is well under a microsecond of the
     * control loop's 1000 us period.
     */
    void getTelemetry(Telemetry_t& out);

    void setTarget(Target_t target);

    /**
     * This function contains the setup state machine. Any systems that have a non blocking setup
     * routine should be placed here in the appropriate stage. 
     * 
     * @warning No physical setup should occur here. This should only be electronics. 
     */
     bool startup();

    /**
     * Everything the drone does periodically, at CONTROL_LOOP_HZ (1 kHz).
     * Runs the flight control algorithm and records the telemetry snapshot.
     *
     * @warning Called directly from the control timer ISR, so this and
     * everything it calls run in INTERRUPT CONTEXT. No I2C, SPI, Serial, heap
     * allocation, delay(), or pushing to the radio/USB tx queues - those
     * queues are drained by loop() and are not interrupt safe. Transmission is
     * loop()'s job; this function only records.
     */
     void update();

    /**
     * @brief Checks the status of the flight watchdog timmer.
     * 
     * If the watchdog has expired then movement is assumed to be prohibited.
     * All physical mechanisms on the vehicle should use this gaurd to 
     * prevent unauthorized movement in the case of a communication failure. 
     * 
     * @returns Will return true if the watchdog is still alive.
     * Will return false if the watchdog has expired. 
     */
    bool getFlightWatchdogStatus();

    /**
     * Feeds the flight watchdog
     * 
     * @related bool getFlightWatchdogStatus()
     */
    void feedFlightWatchdog();

};