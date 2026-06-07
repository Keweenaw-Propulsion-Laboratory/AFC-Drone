#include "drone.h"

#include "radio.h"
#include "gyro.h"
#include "gimbal.h"
#include "error.h" 
#include "debug.h"

/**
 * Performs the startup sequence
 */
bool Drone::startup() {
    // Step 1 Radio
    
    switch (state)
    {
    case DroneStates::BOOT:
        // This state handles any internal initialization that the controller may need to do
        pinMode(STATUS_LED, OUTPUT);

        // Check if there is a serial connection.
        // If true then move on with loop
        if(!Debug::checkSerial()) {return;}
        
        
        // Transition to next state
        state = DroneStates::RADIO_SETUP;
        Debug::println("DRONE: State progressing from BOOT to RADIO_SETUP");
        break;
    
    case DroneStates::RADIO_SETUP :
        if(!Radio::setup()) {
            state = DroneStates::FAULT_ERROR;
            Debug::println("DRONE: SETUP FAILURE in stage RADIO_SETUP");
        }

        if (Radio::setupComplete()) {
            state = DroneStates::SENSOR_SETUP;
            Debug::println("DRONE: State progressing from RADIO_SETUP to SENSOR_SETUP");
        }
        break;

    case DroneStates::SENSOR_SETUP :
        // Needs to init Gryo. Any other sensors can also go in here

        // Run setup functions here
        if (!Gyro::setup()) {
            state = DroneStates::FAULT_ERROR;
            Debug::println("DRONE: SETUP FAILURE in stage SENSOR_SETUP -> GYRO");
        }

        // if (!GPS::setup()) {
        //     state = DroneStates::FAULT_ERROR;
        //     Debug::println("DRONE: SETUP FAILURE in stage SENSOR_SETUP -> GPS")
        // }

        // Check for complete here. 
        if (Gyro::setupComplete()) { // Add && GPS::setupComplete()
            state = DroneStates::READY_ARMED;
        }

        break;

    default:
        break;
    }

    return state == DroneStates::READY_ARMED;

}

void Drone::update() {
    
    return;
}


/**
 * Helper class to update status LEDS to inform us of current state
 */
void Drone::updateLEDS() {
    static bool ledOn = false;
    static uint32_t lastLEDToggle = 0;

    uint32_t blinkInterval = 500; // Nominal blink interval every 500ms

    switch (state) {
        case DroneStates::RADIO_SETUP :
            blinkInterval = 100; // Blink every 100 ms during radio setup
            break;
        
        case DroneStates::SENSOR_SETUP :
            blinkInterval = 300; // Blink every 300 ms during sensor setup
            break;
        case DroneStates::READY_ARMED :
            ledFader();
            return;

        case DroneStates::FLIGHT :
            doubleFlash();
            return;

        case DroneStates::FAULT_ERROR :
            blinkInterval = 50; // PaNiC
        default :
            break;    
        }

    if (millis() - lastLEDToggle >= blinkInterval) {
        ledOn = !ledOn;
        // Digital write forces the LED to either a solid 0 or 255 duty cycle
        digitalWrite(STATUS_LED, ledOn ? HIGH : LOW);
        lastLEDToggle = millis();
    }
}

void Drone::ledFader() {
    // The total time in milliseconds for one full breathe cycle (inhale + exhale)
    const float breathePeriodMs = 2000.0f; 
    
    // Convert the current time into a continuous radian angle
    float radians = (2.0f * PI * (float)millis()) / breathePeriodMs;
    
    // Calculate a smooth sine wave scaled between 0.0 and 255.0
    float smoothValue = 127.5f * (1.0f + sin(radians));
    
    // Output a true hardware PWM duty cycle to your fade pin
    // Note: Make sure your chosen LED pin supports PWM! (On Teensy 4.1, almost all pins do)
    analogWrite(LED_BUILTIN, (int)smoothValue);
    return; // Exit early so standard blinking code below doesn't override this
}

void Drone::doubleFlash() {
const uint32_t cycleDuration = 1200; // Total duration of the pattern in ms
        
        // This collapses the infinite timeline of millis() into a repeating 0-1199ms window
        uint32_t currentCycleTime = millis() % cycleDuration;

        if (currentCycleTime < 100) {
            // 0ms to 99ms -> First Strobe
            digitalWrite(LED_BUILTIN, HIGH);
        } 
        else if (currentCycleTime >= 100 && currentCycleTime < 250) {
            // 100ms to 249ms -> Dark gap
            digitalWrite(LED_BUILTIN, LOW);
        } 
        else if (currentCycleTime >= 250 && currentCycleTime < 350) {
            // 250ms to 349ms -> Second Strobe
            digitalWrite(LED_BUILTIN, HIGH);
        } 
        else {
            // 350ms to 1199ms -> Long dark pause before cycle resets
            digitalWrite(LED_BUILTIN, LOW);
        }
        
}
