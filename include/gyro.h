#pragma once

#include <Adafruit_BNO08x.h>

#define GYRO_RESET -1

class Gyro{
    public: 
    struct euler_t {
        float yaw;
        float pitch;
        float roll;
    };

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
    
    static euler_t ypr;
    static DroneState droneState;
    static uint32_t lastCheck;

    // Latest orientation quaternion in the raw BNO08x sensor frame (real, i, j, k), updated in update()
    static float quatReal;
    static float quatI;
    static float quatJ;
    static float quatK;

    // Same orientation quaternion remapped into the drone's own body frame
    // (see the mounting remap comment in update()). This is what telemetry
    // consumers should use to report vehicle attitude.
    static float droneQuatReal;
    static float droneQuatI;
    static float droneQuatJ;
    static float droneQuatK;

    static float worldAccelX;
    static float worldAccelY;
    static float worldAccelZ;

    // Functions
    static bool setup();
    static bool setupComplete();

    static void update();

    static float getPitch();
    static float getYaw();
    static float getRoll();

    static void updateDeadReckoning(float wX, float wY, float wZ);

    // Helpers
    static void quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees = false);
    static void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, bool degrees = false);
    static void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees = false);

    static void transformToWorldFrame(float qW, float qX, float qY, float qZ, float ax, float ay, float az, 
                           float& worldX, float& worldY, float& worldZ);

    static void debug();

    static Adafruit_BNO08x gyro;
    static sh2_SensorValue_t sensorValue;

    private: 

        enum GyroSetupStates : uint8_t {
            I2C,
            EnableReport,
            Complete
        };

        static GyroSetupStates state;
};
