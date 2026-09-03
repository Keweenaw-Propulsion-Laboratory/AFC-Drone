#include <Arduino.h>

#include "drone.h"
#include "gimbal.h"
#include "radio.h"
#include "gyro.h"
#include "usb.h"
#include "configs.h"

#define onboard 13

#define AlPHA_SMOOTHING 0.1f

#define LOOP_STATUS_INTERVAL 2000 // 2000 ms

void setup() {
    config_load(); // Load configs from flash

    while (!Drone::startup()) {}

}

void loop() {
    // Serviced every pass, independent of the control loop tick, so radio/USB
    // I/O and sensor fusion stay responsive between control ticks.
    radio_update();
    usb_update();
    Gyro::update();

    // Drone::controlTick is set by a hardware timer ISR (see Drone::onControlTick
    // in drone.cpp) at a fixed CONTROL_LOOP_HZ cadence. Gating the flight control
    // algorithm on it, rather than busy-waiting on micros(), gives it a jitter-free
    // hardware-timed cycle while leaving the CPU free to service I/O in between.
    if (Drone::controlTick) {
        Drone::controlTick = false;

        const uint32_t startTime = micros();

        // MARK: Control Loop Logic
        Drone::update(); // Perform flight logic
        // End of Control Loop Logic

        const uint32_t currentLoopCost = micros() - startTime;

        static float rollingAverage = 0.0f;
        // Alpha controls smoothing. 0.01 means the average changes smoothly over ~100 loops.
        const float alpha = AlPHA_SMOOTHING;

        if (rollingAverage == 0.0f) {
            rollingAverage = (float)currentLoopCost; // Initialize on the very first boot loop
        } else {
            rollingAverage = (alpha * (float)currentLoopCost) + ((1.0f - alpha) * rollingAverage);
        }

        Drone::lastLoopTime = static_cast<uint16_t>(currentLoopCost);
        drone_rollAvg = static_cast<uint16_t>(rollingAverage);
        if (Drone::lastLoopTime > Drone::worstTime) {Drone::worstTime = Drone::lastLoopTime;}
        if (Drone::lastLoopTime < Drone::bestTime) {Drone::bestTime = Drone::lastLoopTime;}
    }
}

