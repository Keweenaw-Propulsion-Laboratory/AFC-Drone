#pragma once

#include <cstdint>

namespace Battery {
    
    void setup(); // Tell teensy input pin, sets it to analog

    float getVoltage(); // Get the current voltage of the battery

    float getVoltPercent(); // Get the current voltage of the battery as a percentage

    enum class BatteryStates : uint8_t { //uint8_t is an unsinged 8 bit interger, t is a naming convention for type
        FAULT_ERROR,
        CRITICAL_BATTERY, //DANGER battery level, harmful to the battery health 20% battery left
        LOW_BATTERY, //Recommend to land drone immediatly
        MIDPOINT, //Half of safe battery level left 
        MAXIMUM, //Fully charged battery
    };

    BatteryStates getBatteryState(); // Get the current state of the battery
}

