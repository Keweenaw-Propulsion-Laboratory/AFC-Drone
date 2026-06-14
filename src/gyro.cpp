#include "gyro.h"
#include "debug.h"


Adafruit_BNO08x Gyro::gyro(GYRO_RESET);
Gyro::euler_t Gyro::ypr;
sh2_SensorValue_t Gyro::sensorValue;

Gyro::DroneState Gyro::droneState;

Gyro::GyroSetupStates Gyro::state = Gyro::GyroSetupStates::I2C; 

float Gyro::worldAccelX = 0.0f;
float Gyro::worldAccelY = 0.0f;
float Gyro::worldAccelZ = 0.0f;
uint32_t Gyro::lastIntegrationTime = 0;

// Shared temporaries for quaternions
static float current_qW = 1.0f, current_qX = 0.0f, current_qY = 0.0f, current_qZ = 0.0f;

bool Gyro::setup() {

    switch (state)
    {
    case Gyro::GyroSetupStates::I2C :
        if (! gyro.begin_I2C()) {
            // TODO add error
            Debug::println("Gyro I2C failed");
            return false;
        }
        state = EnableReport;
        break;
    
    case Gyro::GyroSetupStates::EnableReport :
        Debug::println("BN0085 connected");
        /*
        This section of the setup determines what kind of data we want to 
        get from the Gyro. The different report types can be found at the link below
        https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/report-types

        */
        if (! gyro.enableReport(SH2_GAME_ROTATION_VECTOR, 5000)) { // 100 hz
            Debug::println("Could not enable stabilized rotation vector");
            return false;
        }
        if (! gyro.enableReport(SH2_LINEAR_ACCELERATION, 5000)) {
            Debug::println("Could not enable Linear Acceleration report");
            return false;
        }

        Debug::println("Gyro Complete");
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

    // Check if Gyro has new data
    if (!gyro.getSensorEvent(&sensorValue)) {
        // No available data
        return;
    }



    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        sh2_RotationVectorWAcc_t quad = sensorValue.un.rotationVector;

        // Convert raw Quaternions to standard Radians/Degrees
        current_qW = quad.real;
        current_qX = quad.i;
        current_qY = quad.j;
        current_qZ = quad.k;

        quaternionToEuler(current_qW, current_qX, current_qY, current_qZ, true);

        // // 3D Math to extract Pitch, Roll, Yaw
        // float ysqr = current_qY * current_qY;
        
        // // Pitch (X-axis rotation)
        // float t0 = +2.0f * (current_qW * current_qX + current_qW * current_qZ);
        // float t1 = +1.0f - 2.0f * (current_qX * current_qX + ysqr);
        // ypr.pitch = atan2(t0, t1) * RAD_TO_DEG;

        // // roll (Y-axis rotation)
        // float t2 = +2.0f * (current_qW * current_qY - current_qZ * current_qX);
        // t2 = t2 > 1.0f ? 1.0f : t2;
        // t2 = t2 < -1.0f ? -1.0f : t2;
        // ypr.roll = asin(t2) * RAD_TO_DEG;

        // // Yaw (Z-axis rotation)
        // float t3 = +2.0f * (current_qW * current_qZ + current_qX * current_qY);
        // float t4 = +1.0f - 2.0f * (ysqr + current_qZ * current_qZ);
        // ypr.yaw = atan2(t3, t4) * RAD_TO_DEG;
    }

    // Handle incoming Acceleration packet
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        float ax = sensorValue.un.linearAcceleration.x;
        float ay = sensorValue.un.linearAcceleration.y;
        float az = sensorValue.un.linearAcceleration.z;

        // 1. Rotate raw acceleration into world frame using the last known quaternion orientation
        transformToWorldFrame(current_qW, current_qX, current_qY, current_qZ, ax, ay, az, worldAccelX, worldAccelY, worldAccelZ);

        updateDeadReckoning(worldAccelX, worldAccelY, worldAccelZ);

        // // 2. Compute loop timing
        // uint32_t now = micros();
        // float dt = (now - lastIntegrationTime) / 1000000.0f;
        // lastIntegrationTime = now;
        // if (dt <= 0.0f || dt > 0.1f) return; // Fail-safe against loop timing anomalies

        // // 3. Apply Deadband filtering to isolate ambient structural vibrations
        // const float deadband = 0.08f; // m/s^2 
        // if (abs(worldAccelX) < deadband) worldAccelX = 0.0f;
        // if (abs(worldAccelY) < deadband) worldAccelY = 0.0f;
        // if (abs(worldAccelZ) < deadband) worldAccelZ = 0.0f;

        // // 4. Double Integration Step
        // droneState.velocity.x += worldAccelX * dt;
        // droneState.velocity.y += worldAccelY * dt;
        // droneState.velocity.z += worldAccelZ * dt;

        // droneState.position.x += droneState.velocity.x * dt;
        // droneState.position.y += droneState.velocity.y * dt;
        // droneState.position.z += droneState.velocity.z * dt;

        // // 5. Apply the Leaky Integrator (Drain accumulation when standing completely still)
        // const float leak = 0.985f;
        // if (worldAccelX == 0.0f) { droneState.velocity.x *= leak; droneState.position.x *= leak; }
        // if (worldAccelY == 0.0f) { droneState.velocity.y *= leak; droneState.position.y *= leak; }
        // if (worldAccelZ == 0.0f) { droneState.velocity.z *= leak; droneState.position.z *= leak; }
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
            Debug::println(debugOut);
            break;
        case 1:
            sprintf(debugOut, "PosX %.3f, PosY %.3f, PosZ %.3f", droneState.position.x, droneState.position.y, droneState.position.z);
            Debug::println(debugOut);
        default:
            break;
        }

        report = !report;

        debugTimmer -= 500;
    }
}