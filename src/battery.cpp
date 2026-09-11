#include "battery.h"

#include <Arduino.h>


namespace Battery{


// UNSIGNED analog pin value, set to -1 for now.
static constexpr uint8_t BATTERY_PIN = -1; // Analog input pin

void setup() {
    pinMode(BATTERY_PIN, INPUT); // Set the analog input pin for battery voltage measurement
    // Serial.begin(9600); // Initialize serial communication
    // Serial.println(ADC_STEPS);
}

// Get the current voltage of the battery *Mainly copied from ArduinoDocs for analogRead()*
static constexpr float V_REF = 16.5;     // Analog reference voltage (e.g., 5V or 3.3V)
static constexpr float R_BITS = 12.0;   // ADC resolution (bits)
constexpr float ADC_STEPS = (1 << int(R_BITS)) - 1; // Number of steps (2^R_BITS - 1)

float getVoltage() {
    int rawValue = analogRead(BATTERY_PIN); // Read the analog input
    float voltage = (rawValue / ADC_STEPS) * V_REF; // Convert to voltage
    return voltage;
}

constexpr float minVoltage = 13.09; // Minimum voltage for 4S LiPo, based on chart
constexpr float maxVoltage = 16.5; // Maximum voltage for 4S LiPo, got from charging our battery to full
constexpr float nominalVoltage = 14.8; // Nominal voltage for 4S LiPo, mostly for reference, not used in calculation

float getVoltPercent() {
    float voltage = getVoltage();

    // Map the voltage to a percentage between 0% and 100%
    float percent = (voltage - minVoltage) / (maxVoltage - minVoltage) * 100.0;
    percent = constrain(percent, 0.0, 100.0); // Ensure percentage is within bounds
    return percent;
}

Battery::BatteryStates getBatteryState() {
    float percent = getVoltPercent();

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