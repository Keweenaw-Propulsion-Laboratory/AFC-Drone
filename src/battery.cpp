#include "battery.h"

#include "usb.h"

#include <Arduino.h>

namespace Battery{
    // Analog pin value, set to -1 until the hardware pin is configured.
    static constexpr int BATTERY_PIN = -1; // Analog input pin
    static constexpr int ADC_RESOLUTION_BITS = 12;   // ADC resolution (bits)
    static constexpr float VOLTAGE_ALPHA = 1.0f / 16.0f; // 16-sample equivalent rolling average

    static float rollingVoltage = 0.0f;
    static bool rollingVoltageInitialized = false;
    static bool initialized = false;

    bool setup() {
        if (BATTERY_PIN < 0) {
            USB::sendText("DRONE: Battery pin not configured. Please set BATTERY_PIN in battery.cpp");
            initialized = false;
            return true;
        }

        pinMode(BATTERY_PIN, INPUT); 
        analogReadResolution(ADC_RESOLUTION_BITS);
        initialized = true;
        update();
        return true;
    }

    // Get the current voltage of the battery *Mainly copied from ArduinoDocs for analogRead()*
    static constexpr float V_REF = 16.5f;     // Analog reference voltage
    static constexpr int ADC_STEPS = (1 << ADC_RESOLUTION_BITS) - 1; // Number of steps (2^ADC_RESOLUTION_BITS - 1)

    float getVoltage() {
        return rollingVoltage;
    }

    void update() {
        if (!initialized) {
            return;
        }

        int rawValue = analogRead(BATTERY_PIN); // Read the analog input
        // Have to cast rawValue to a float during the division. As it will truncate to 0 for any ADC reading less than 4095.
        float voltage = (static_cast<float>(rawValue) / ADC_STEPS) * V_REF; // Convert to voltage

        if (!rollingVoltageInitialized) {
            rollingVoltage = voltage;
            rollingVoltageInitialized = true;
        } else {
            rollingVoltage += VOLTAGE_ALPHA * (voltage - rollingVoltage);
        }
    }

    static constexpr float MIN_VOLTAGE = 13.09f; // Minimum voltage for 4S LiPo, based on chart
    static constexpr float MAX_VOLTAGE = 16.5f; // Maximum voltage for 4S LiPo, got from charging our battery to full
    static constexpr float NOMINAL_VOLTAGE = 14.8f; // Nominal voltage for 4S LiPo, mostly for reference, not used in calculation
    static constexpr float VOLTAGE_ERROR_MARGIN = 0.025f; // Allow 2.5% measurement error at the voltage limits
    static constexpr float MIN_VALID_VOLTAGE = MIN_VOLTAGE * (1.0f - VOLTAGE_ERROR_MARGIN);
    static constexpr float MAX_VALID_VOLTAGE = MAX_VOLTAGE * (1.0f + VOLTAGE_ERROR_MARGIN);

    static float calculatePercent(float voltage) {
        return (voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100.0f;
    }

    float getPercent() {
        return calculatePercent(getVoltage());
    }

    Battery::BatteryStates getBatteryState() {
        float voltage = getVoltage();
        if (voltage < MIN_VALID_VOLTAGE || voltage > MAX_VALID_VOLTAGE) {
            return BatteryStates::FAULT_ERROR;
        }

        float percent = calculatePercent(voltage);

        if (percent <= 20.0f) {
            return BatteryStates::CRITICAL_BATTERY;
        } else if (percent <= 40.0f) {
            return BatteryStates::LOW_BATTERY;
        } else if (percent <= 60.0f) {
            return BatteryStates::MIDPOINT;// midway between 20&100, the safe % levels.
        } else {
            return BatteryStates::MAXIMUM;
        }
    }
}