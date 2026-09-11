
/**
 * Development Notes
 * Show Voltage
 * The battery is a standard 4s Lipo battery, 
 * which has a nominal voltage of 14.8V and a maximum voltage of 16.8V when fully charged. 
 * The battery voltage can be measured using an analog input pin on the microcontroller. 
 * The voltage reading can be converted to a percentage to indicate the remaining battery life.
 * 
 * Set lowest voltage to 15.0V, and highest voltage to 16.8V, and then map the current voltage to a percentage between 0% and 100%.
 * pull the current voltage from the battery and save that value,
 * Show percent
 * Static Allocation, compiler tells how much memory you need
 * extern uint8_t motor_topSetSpeed;
extern uint8_t motor_bottomSetSpeed;
 */
#pragma once


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

