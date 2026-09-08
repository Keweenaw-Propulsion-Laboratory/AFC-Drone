#include <Arduino.h>

#include "drone.h"
#include "gimbal.h"
#include "radio.h"
#include "gyro.h"
#include "usb.h"
#include "configs.h"

#define onboard 13

#define LOOP_STATUS_INTERVAL 2000 // 2000 ms

void setup() {
    Configs::load(); // Load configs from flash

    while (!Drone::startup()) {}

}

void loop() {
    // Serviced every pass, independent of the control loop tick, so radio/USB
    // I/O and sensor fusion stay responsive between control ticks.
    Radio::update();
    USB::update();
    Gyro::update();

    // Telemetry. The control ISR records a snapshot every tick; sending it is
    // loop()'s job, because pushing to the radio and USB tx queues is not
    // interrupt safe. One snapshot is taken per frame and handed to every
    // sender, so all eight packets describe the same instant.
    static uint32_t lastTelemetryMs = 0;
    constexpr uint32_t telemetryIntervalMs = 100;
    const uint32_t now = millis();
    if (now - lastTelemetryMs >= telemetryIntervalMs) {
        Drone::Telemetry_t snapshot;
        Drone::getTelemetry(snapshot);

        USB::sendTelemetry(snapshot);
        Radio::sendStatus0(snapshot);
        Radio::sendStatus1(snapshot);
        Radio::sendStatus2(snapshot);
        Radio::sendStatus3(snapshot);
        Radio::sendStatus4(snapshot);
        Radio::sendStatus5(snapshot);
        Radio::sendStatus6(snapshot);
        lastTelemetryMs = now;
    }

    // The flight control algorithm is NOT run from here. Drone::update() is
    // called directly by the control timer ISR (Drone::onControlTick in
    // drone.cpp) so its cadence does not depend on how long this pass through
    // radio/USB/gyro servicing takes. Its own timing is measured in that ISR
    // and reported through the telemetry snapshot above.
}

