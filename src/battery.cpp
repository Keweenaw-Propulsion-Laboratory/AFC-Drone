#include "battery.h"
#include <Arduino.h>

namespace Battery{
    // UNSIGNED analog pin value, set to -1 for now.
    static constexpr uint8_t BATTERY_PIN = -1; // Analog input pin
    static constexpr int R_BITS = 12;   // ADC resolution (bits)

    void setup() {
        pinMode(BATTERY_PIN, INPUT); 
        analogReadResolution(R_BITS);
    }

    // Get the current voltage of the battery *Mainly copied from ArduinoDocs for analogRead()*
    static constexpr float V_REF = 16.5;     // Analog reference voltage
    static constexpr int ADC_STEPS = (1 << R_BITS) - 1; // Number of steps (2^R_BITS - 1)

    float getVoltage() {
        int rawValue = analogRead(BATTERY_PIN); // Read the analog input
        // Have to cast rawValue to a float during the division. As it will truncate to 0 for any ADC reading less than 4095.
        float voltage = (static_cast<float>(rawValue) / ADC_STEPS) * V_REF; // Convert to voltage
        return voltage;
    }

    static constexpr float minVoltage = 13.09; // Minimum voltage for 4S LiPo, based on chart
    static constexpr float maxVoltage = 16.5; // Maximum voltage for 4S LiPo, got from charging our battery to full
    static constexpr float nominalVoltage = 14.8; // Nominal voltage for 4S LiPo, mostly for reference, not used in calculation

    float getPercent() {
        float voltage = getVoltage();
        // Map the voltage to a percentage between 0% and 100%
        float percent = (voltage - minVoltage) / (maxVoltage - minVoltage) * 100.0;
        percent = constrain(percent, 0.0, 100.0); // Ensure percentage is within bounds
        return percent;
    }

    Battery::BatteryStates getBatteryState() {
        float percent = getPercent();

        if (percent <= 20.0) {
            return BatteryStates::CRITICAL_BATTERY;
        } else if (percent <= 40.0) {
            return BatteryStates::LOW_BATTERY;
        } else if (percent <= 60.0) {
            return BatteryStates::MIDPOINT;// midway between 20&100, the safe % levels.
        } else if (percent <= 100.0) {
            return BatteryStates::MAXIMUM;
        } else {
            return BatteryStates::FAULT_ERROR; // This should never happen, but just in case
        }
    }
}