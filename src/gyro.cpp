#include "gyro.h"
#include "usb.h"

Adafruit_BNO08x Gyro::gyro(GYRO_RESET);
Gyro::euler_t Gyro::ypr;
sh2_SensorValue_t Gyro::sensorValue;

Gyro::DroneState Gyro::droneState;

Gyro::GyroSetupStates Gyro::state = Gyro::GyroSetupStates::I2C;

float Gyro::worldAccelX = 0.0f;
float Gyro::worldAccelY = 0.0f;
float Gyro::worldAccelZ = 0.0f;

// Identity quaternion until the first rotation vector report arrives
float Gyro::quatReal = 1.0f;
float Gyro::quatI = 0.0f;
float Gyro::quatJ = 0.0f;
float Gyro::quatK = 0.0f;

float Gyro::droneQuatReal = 1.0f;
float Gyro::droneQuatI = 0.0f;
float Gyro::droneQuatJ = 0.0f;
float Gyro::droneQuatK = 0.0f;

bool Gyro::setup() {

    switch (state)
    {
    case Gyro::GyroSetupStates::I2C :
        if (! gyro.begin_I2C()) {
            // TODO add error
            usb_send_text("Gyro I2C failed", 15);
            return false;
        }
        // begin_I2C() leaves the bus at Teensy's default 100 kHz; the BNO08x
        // is polled over I2C every main-loop iteration, so bump to 400 kHz
        // Fast Mode (the documented safe max) to keep that poll cheap.
        Wire.setClock(400000);
        state = EnableReport;
        break;
    
    case Gyro::GyroSetupStates::EnableReport :
        usb_send_text("BN0085 connected", 16);
        /*
        This section of the setup determines what kind of data we want to 
        get from the Gyro. The different report types can be found at the link below
        https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/report-types

        */
        if (! gyro.enableReport(SH2_GAME_ROTATION_VECTOR, 5000)) { // 200 hz
            usb_send_text("Could not enable stabilized rotation vector", 41);
            return false;
        }
        if (! gyro.enableReport(SH2_LINEAR_ACCELERATION, 5000)) {
            usb_send_text("Could not enable Linear Acceleration report", 43);
            return false;
        }
        usb_send_text("Gyro Complete", 13);
        state = Complete;
        break;  
    default:
        return false;
        break;

    }
    return true;
}

bool Gyro::setupComplete() {
    return state == GyroSetupStates::Complete;
}

void Gyro::update(){
    // If gyro is not initialized, skip
    if (state != GyroSetupStates::Complete) return;

    // Reports arrive at 200 Hz (5 ms); polling I2C faster than that just
    // blocks on empty transactions. Throttle to every 2 ms (2.5x
    // oversampling) so most main-loop iterations skip the I2C call entirely.
    static elapsedMicros sincePoll = 0;
    if (sincePoll < 2000) return;
    sincePoll = 0;

    // Check if Gyro has new data
    if (!gyro.getSensorEvent(&sensorValue)) {
        // No available data
        return;
    }



    // Setup only enables SH2_GAME_ROTATION_VECTOR, so that is the report that
    // actually arrives here; SH2_ROTATION_VECTOR is a different report ID and
    // would never match, leaving the quaternion stuck at identity.
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        sh2_RotationVector_t quad = sensorValue.un.gameRotationVector;

        quatReal = quad.real;
        quatI = quad.i;
        quatJ = quad.j;
        quatK = quad.k;

        quaternionToEuler(quatReal, quatI, quatJ, quatK, true);

        // Re-express the same rotation using the drone's own body axes
        // (Xd = gyro Zs, Yd = -gyro Xs, Zd = -gyro Ys). The rotation angle
        // (real part) is basis-independent, so only the axis (vector) part
        // needs to be remapped.
        droneQuatReal = quatReal;
        droneQuatI = quatK;
        droneQuatJ = -quatI;
        droneQuatK = -quatJ;
    }

    // Handle incoming Acceleration packet
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        float ax = sensorValue.un.linearAcceleration.x;
        float ay = sensorValue.un.linearAcceleration.y;
        float az = sensorValue.un.linearAcceleration.z;

        // BNO08x mounting relative to the drone body:
        //   gyro Y = drone -Z
        //   gyro X = drone -Y
        //   gyro Z = drone  X
        // i.e. drone Xd = gyro Zs, Yd = -gyro Xs, Zd = -gyro Ys. Remap raw
        // sensor-frame acceleration into the drone's own body frame so
        // body_accel.z is always the drone's vertical/thrust axis,
        // regardless of chip mounting.
        droneState.body_accel.x = az;
        droneState.body_accel.y = -ax;
        droneState.body_accel.z = -ay;

        // Rotate raw (sensor-frame) acceleration into the world frame using
        // the last known quaternion orientation. This is independent of the
        // body_accel mounting remap above: SH2_GAME_ROTATION_VECTOR's
        // reference frame is Z-up by hardware definition, so applying the
        // raw quaternion to the raw acceleration already yields a world
        // frame with Z vertical, regardless of how the chip is mounted.
        transformToWorldFrame(quatReal, quatI, quatJ, quatK, ax, ay, az, worldAccelX, worldAccelY, worldAccelZ);

        updateDeadReckoning(worldAccelX, worldAccelY, worldAccelZ);
    }
}

float Gyro::getPitch() {
    return ypr.pitch;
}

float Gyro::getYaw() {
    return ypr.yaw;
}

float Gyro::getRoll() {
    return ypr.roll;
}


uint32_t Gyro::lastCheck = 0;
void Gyro::updateDeadReckoning(float wX, float wY, float wZ) {
    uint32_t now = micros();

    // Skip first loop to get accurate times
    if (lastCheck == 0) {
        lastCheck = now;
        return;
    }
    float dt = (now - lastCheck) / 1000000.0f;
    lastCheck = now;

    // Fail-safe against loop timing hiccups (don't process if loop stalled)
    if (dt <= 0.0f || dt > 0.1f) return;

    // 1. Dynamic Deadband: Ignore baseline sensor hiss/vibrations
    const float accelNoiseThreshold = 0.08f; // m/s^2
    if (abs(wX) < accelNoiseThreshold) wX = 0.0f;
    if (abs(wY) < accelNoiseThreshold) wY = 0.0f;
    if (abs(wZ) < accelNoiseThreshold) wZ = 0.0f;

    // 2. Standard Integration Step
    droneState.velocity.x += wX * dt;
    droneState.velocity.y += wY * dt;
    droneState.velocity.z += wZ * dt;

    droneState.position.x += droneState.velocity.x * dt;
    droneState.position.y += droneState.velocity.y * dt;
    droneState.position.z += droneState.velocity.z * dt;

    // 3. The Leak: Slowly drain energy so baseline drift doesn't stack to infinity
    // 0.985 means it retains 98.5% of its velocity/position per loop iteration
    const float leakFactor = 0.985f; 
    if (wX == 0.0f) { droneState.velocity.x *= leakFactor;}; //droneState.position.x *= leakFactor; }
    if (wY == 0.0f) { droneState.velocity.y *= leakFactor;}; //droneState.position.y *= leakFactor; }
    if (wZ == 0.0f) { droneState.velocity.z *= leakFactor;}; //droneState.position.z *= leakFactor; }
}



void Gyro::quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr.yaw *= RAD_TO_DEG;
      ypr.pitch *= RAD_TO_DEG;
      ypr.roll *= RAD_TO_DEG;
    }
}

void Gyro::quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, bool degrees) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, degrees);
}

void Gyro::quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, degrees);
}

// Transforms raw sensor acceleration into stable world-frame acceleration
void Gyro::transformToWorldFrame(float qW, float qX, float qY, float qZ, 
                           float ax, float ay, float az, 
                           float& worldX, float& worldY, float& worldZ) {
    // 3D Rotation Matrix derived directly from the orientation quaternion
    float r11 = 1.0f - 2.0f * (qY * qY + qZ * qZ);
    float r12 = 2.0f * (qX * qY - qW * qZ);
    float r13 = 2.0f * (qX * qZ + qW * qY);

    float r21 = 2.0f * (qX * qY + qW * qZ);
    float r22 = 1.0f - 2.0f * (qX * qX + qZ * qZ);
    float r23 = 2.0f * (qY * qZ - qW * qX);

    float r31 = 2.0f * (qX * qZ - qW * qY);
    float r32 = 2.0f * (qY * qZ + qW * qX);
    float r33 = 1.0f - 2.0f * (qX * qX + qY * qY);

    // Multiply the matrix by the body acceleration vector
    worldX = r11 * ax + r12 * ay + r13 * az;
    worldY = r21 * ax + r22 * ay + r23 * az;
    worldZ = r31 * ax + r32 * ay + r33 * az;
}



elapsedMillis debugTimmer;
uint8_t report = 0;
void Gyro::debug() {
//     if (gyro.getSensorEvent(&sensorValue)) {
//         // in this demo only one report type will be received depending on FAST_MODE define (above)
//         switch (sensorValue.sensorId) {
//         case SH2_ARVR_STABILIZED_RV:
//             quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, true);
//         case SH2_GYRO_INTEGRATED_RV:
//             // faster (more noise?)
//             quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, true);
//             break;
//         }
//     static long last = 0;
//     long now = micros();
//     Serial.print(now - last);             Serial.print("\t");
//     last = now;
//     Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
//     Serial.print(ypr.yaw);                Serial.print("\t");
//     Serial.print(ypr.pitch);              Serial.print("\t");
//     Serial.println(ypr.roll);
//   }

    if ( debugTimmer > 500 ) {
        char debugOut[128];

        switch (report)
        {
        case 0:
            sprintf(debugOut, "Pitch %.3f, Yaw %.3f, Roll %.3f", getPitch(), getYaw(), getRoll());
            usb_send_text(debugOut, strlen(debugOut));
            break;
        case 1:
            sprintf(debugOut, "PosX %.3f, PosY %.3f, PosZ %.3f", droneState.position.x, droneState.position.y, droneState.position.z);
            usb_send_text(debugOut, strlen(debugOut));
            break;
        default:
            break;
        }

        report = !report;

        debugTimmer -= 500;
    }
}