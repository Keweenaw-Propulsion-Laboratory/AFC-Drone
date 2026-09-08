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

    extern volatile bool controlTick;
    extern uint16_t rollAvg;


    enum class States: uint8_t {
        BOOT, 
        RADIO_SETUP,  
        SENSOR_SETUP,
        CONTROL_SETUP,
        READY_ARMED,
        FLIGHT,
        FAULT_ERROR
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

    void setTarget(Target_t target);

    /**
     * This function contains the setup state machine. Any systems that have a non blocking setup
     * routine should be placed here in the appropriate stage. 
     * 
     * @warning No physical setup should occur here. This should only be electronics. 
     */
     bool startup();

    /**
     * This function contains all of the functions that the drone is expected to do periodically at a cycle of 1 KHz.
     * 
     * 
     */
     void update();


};