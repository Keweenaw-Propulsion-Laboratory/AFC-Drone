#include "gyro.h"

Adafruit_BNO08x Gyro::gyro(GYRO_RESET);
Gyro::euler_t Gyro::ypr;

void Gyro::setup() {
    while(!Serial) delay(10); // Wait for serial to be available

    Serial.println("Gyro BN0085 setup");

    // Begin setup

    if (! gyro.begin_I2C()) {
        Serial.println("Failed to start gyro");
    }

    Serial.println("BN0085 connected");

    /*
    This section of the setup determines what kind of data we want to 
    get from the Gyro. The different report types can be found at the link below
    https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/report-types

    */
    if (! gyro.enableReport(SH2_ARVR_STABILIZED_RV, 10000)) { // 100 hz
        Serial.println("Could not enable stabilized remote vector");
    }
    
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

void Gyro::debug() {
    if (gyro.getSensorEvent(&sensorValue)) {
        // in this demo only one report type will be received depending on FAST_MODE define (above)
        switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
            quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, true);
        case SH2_GYRO_INTEGRATED_RV:
            // faster (more noise?)
            quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, true);
            break;
        }
    static long last = 0;
    long now = micros();
    Serial.print(now - last);             Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.println(ypr.roll);
  }
}