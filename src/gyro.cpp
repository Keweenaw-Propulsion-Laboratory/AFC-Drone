#include "gyro.h"
#include "debug.h"


Adafruit_BNO08x Gyro::gyro(GYRO_RESET);
Gyro::euler_t Gyro::ypr;
sh2_SensorValue_t Gyro::sensorValue;

Gyro::GyroSetupStates Gyro::state = Gyro::GyroSetupStates::I2C; 

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
        if (! gyro.enableReport(SH2_ARVR_STABILIZED_RV, 10000)) { // 100 hz
            Debug::println("Could not enable stabilized remote vector");
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

    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
        sh2_RotationVectorWAcc_t quad = sensorValue.un.rotationVector;

        // Convert raw Quaternions to standard Radians/Degrees
        float r = quad.real;
        float i = quad.i;
        float j = quad.j;
        float k = quad.k;

        // 3D Math to extract Pitch, Roll, Yaw
        float ysqr = j * j;
        
        // Pitch (X-axis rotation)
        float t0 = +2.0f * (r * i + j * k);
        float t1 = +1.0f - 2.0f * (i * i + ysqr);
        ypr.pitch = atan2(t0, t1) * RAD_TO_DEG;

        // roll (Y-axis rotation)
        float t2 = +2.0f * (r * j - k * i);
        t2 = t2 > 1.0f ? 1.0f : t2;
        t2 = t2 < -1.0f ? -1.0f : t2;
        ypr.roll = asin(t2) * RAD_TO_DEG;

        // Yaw (Z-axis rotation)
        float t3 = +2.0f * (r * k + i * j);
        float t4 = +1.0f - 2.0f * (ysqr + k * k);
        ypr.yaw = atan2(t3, t4) * RAD_TO_DEG;
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

elapsedMillis debugTimmer;

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

        Debug::print("Pitch ");
        Debug::println(getPitch());

        Debug::print("Yaw ");
        Debug::println(getYaw());

        Debug::print("Roll ");
        Debug::println(getRoll());

        debugTimmer -= 500;
    }
}