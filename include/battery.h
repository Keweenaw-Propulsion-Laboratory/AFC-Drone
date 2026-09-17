#pragma once

#include <cstdint>

/** 
* Battery Monitoring System
*/
namespace Battery {
    
    /**
     * Set Teensy analog input pin
     * Set resolution to 12 bits
     */
    bool setup();

    /**
     * Read the battery and update its rolling voltage average.
     */
    void update();

    /**
     * @return True if battery monitoring has been initialized. 
     */
    bool setupComplete();

    /** 
    * Get the current voltage of the battery
    */
    float getVoltage();

    /** 
    * Get the current percentage of the battery left
    * This is calculated based on a voltage range,
    * In this case 13.09V to 16.5V for a 4S LiPo battery.
    * getBatteryState() reports FAULT_ERROR for readings more than 2.5% outside
    * this range.
    * percent = (voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100.0f
    */
    float getPercent();

    /** 
    * Based on the current percentage of the battery left, 
    * Return the current state of the battery
    * CRITICAL_BATTERY: dangerous level, harmful to the battery health 20% battery left
    * LOW_BATTERY: low battery level, recommend landing the drone immediately 
    * MIDPOINT: half of safe battery level left
    * MAXIMUM: fully charged battery
    * FAULT_ERROR: This should never happen
    */
    enum class BatteryStates : uint8_t {
        FAULT_ERROR,
        CRITICAL_BATTERY, 
        LOW_BATTERY,
        MIDPOINT, 
        MAXIMUM, 
    };

    /**
     * Get the current state of the battery
     * This accounts for full battery,
     * Not just the "safe" battery levels (20%-100%)
    */
    BatteryStates getBatteryState();
}