#pragma once

#include <Adafruit_BNO08x.h>

constexpr int GYRO_RESET = -1;

namespace Gyro {

    struct state_t {
        double x;
        double y;
        double z;
    };

    struct DroneState{
        state_t body_accel; // Acceleration in the drone's own body frame (z = vertical/thrust axis)
        state_t norm_accel; // Acceleration normalized to gravity.
        state_t velocity; // World-frame velocity (z = vertical)
        state_t position; // World-frame position (z = vertical)

    };

    bool setup();
    bool setupComplete();

    void update();

    float getPitch();
    float getYaw();
    float getRoll();

    /**
     * Orientation quaternion remapped into the drone's own body frame (see the
     * mounting remap comment in update()). This is what telemetry consumers
     * should use to report vehicle attitude; the raw BNO08x sensor-frame
     * quaternion stays private to gyro.cpp.
     */
    float getQuatReal();
    float getQuatI();
    float getQuatJ();
    float getQuatK();

    /** World-frame linear acceleration, in m/s^2. */
    float getWorldAccelX();
    float getWorldAccelY();
    float getWorldAccelZ();

    /** Dead-reckoned world-frame velocity and position. */
    const DroneState& getDroneState();

}
