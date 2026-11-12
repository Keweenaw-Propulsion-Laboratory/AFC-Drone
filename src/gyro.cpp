#include "gyro.h"
#include "usb.h"
#include "configs.h"
#include "logger.h"

namespace Gyro {

// ---- module state: private to this file (conventions section 5) ----

struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

enum GyroSetupStates : uint8_t {
    I2C,
    EnableReport,
    Complete,
    /**
     * IMU turned off by config, so bring-up never touched the I2C bus.
     * Distinct from Complete because update() must not poll a sensor that may
     * not be fitted, while setupComplete() still has to report success.
     */
    Disabled
};

static Adafruit_BNO08x gyro(GYRO_RESET);
static euler_t ypr;

static DroneState droneState;

static GyroSetupStates state = GyroSetupStates::I2C;

static float worldAccelX = 0.0f;
static float worldAccelY = 0.0f;
static float worldAccelZ = 0.0f;

// Identity quaternion until the first rotation vector report arrives.
// Raw BNO08x sensor frame - not published; consumers want the drone-frame
// quaternion below.
static float quatReal = 1.0f;
static float quatI = 0.0f;
static float quatJ = 0.0f;
static float quatK = 0.0f;

static float droneQuatReal = 1.0f;
static float droneQuatI = 0.0f;
static float droneQuatJ = 0.0f;
static float droneQuatK = 0.0f;

static uint32_t lastCheck = 0;

/**
 * BNO08x H_INTN, active low: asserted while the sensor has a transfer waiting,
 * released when the host starts reading it. Its falling edge is the reference
 * the SH-2 report timestamps count back from, so stamping it gives every report
 * its true sample time. -1 falls back to polling every 2 ms, with sample times
 * up to a poll interval late.
 */
static constexpr int GYRO_INT_PIN = 22;

/** Report intervals, us. The calibrated gyro is what the pendulum test reads. */
static constexpr uint32_t GYRO_INTERVAL_US = 2500;         // 400 Hz
static constexpr uint32_t ROTATION_INTERVAL_US = 5000;     // 200 Hz
static constexpr uint32_t LINEAR_ACCEL_INTERVAL_US = 5000; // 200 Hz

/** Transfers serviced per update() at most, so a backlog cannot stall loop(). */
static constexpr uint8_t MAX_TRANSFERS_PER_UPDATE = 4;

/** An INT edge older than this cannot belong to the transfer being read. */
static constexpr uint32_t INT_STALE_US = 20000;

static volatile uint32_t lastIntUs = 0;

/** Reference time for the transfer sh2_service() is reading; see onSensorEvent(). */
static uint32_t serviceRefUs = 0;
static bool serviceRefFromInt = false;

static void onIntFalling() { lastIntUs = micros(); }
static void onSensorEvent(void* cookie, sh2_SensorEvent_t* event);
static void handleSensorValue(const sh2_SensorValue_t& value);

// ---- internal helpers ----

static void updateDeadReckoning(float wX, float wY, float wZ);
static void quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees = false);
[[maybe_unused]] static void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, bool degrees = false);
[[maybe_unused]] static void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees = false);
static void transformToWorldFrame(float qW, float qX, float qY, float qZ, float ax, float ay, float az,
                                  float& worldX, float& worldY, float& worldZ);
[[maybe_unused]] static void debug();

// ---- read accessors for state the rest of the vehicle consumes ----

float getQuatReal() {return droneQuatReal;}
float getQuatI() {return droneQuatI;}
float getQuatJ() {return droneQuatJ;}
float getQuatK() {return droneQuatK;}

float getWorldAccelX() {return worldAccelX;}
float getWorldAccelY() {return worldAccelY;}
float getWorldAccelZ() {return worldAccelZ;}

const DroneState& getDroneState() {return droneState;}

bool setup() {

    // Checked before the switch, because the I2C case below calls begin_I2C()
    // which fails outright with no BNO08x on the bus, and the caller treats
    // that as a fatal setup error.
    if (!Configs::get().gyroEnabled) {
        if (state != GyroSetupStates::Disabled) {
            state = GyroSetupStates::Disabled;
            USB::sendText("Gyro disabled by config");
        }
        return true;
    }

    switch (state)
    {
    case GyroSetupStates::I2C :
        if (! gyro.begin_I2C()) {
            // TODO add error
            USB::sendText("Gyro I2C failed");
            return false;
        }
        // begin_I2C() leaves the bus at Teensy's default 100 kHz; the BNO08x
        // is polled over I2C every main-loop iteration, so bump to 400 kHz
        // Fast Mode (the documented safe max) to keep that poll cheap.
        Wire.setClock(400000);

        // Route reports to onSensorEvent() instead of the library's handler,
        // which keeps only the last report of a transfer and drops the rest.
        // Installed before any report is enabled, so none reach the old one.
        sh2_setSensorCallback(onSensorEvent, nullptr);

        if (GYRO_INT_PIN >= 0) {
            pinMode(GYRO_INT_PIN, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(GYRO_INT_PIN), onIntFalling, FALLING);
        }
        state = EnableReport;
        break;
    
    case GyroSetupStates::EnableReport :
        USB::sendText("BN0085 connected");
        /*
        This section of the setup determines what kind of data we want to 
        get from the Gyro. The different report types can be found at the link below
        https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/report-types

        */
        if (! gyro.enableReport(SH2_GAME_ROTATION_VECTOR, ROTATION_INTERVAL_US)) {
            USB::sendText("Could not enable stabilized rotation vector");
            return false;
        }
        if (! gyro.enableReport(SH2_LINEAR_ACCELERATION, LINEAR_ACCEL_INTERVAL_US)) {
            USB::sendText("Could not enable Linear Acceleration report");
            return false;
        }
        if (! gyro.enableReport(SH2_GYROSCOPE_CALIBRATED, GYRO_INTERVAL_US)) {
            USB::sendText("Could not enable calibrated gyroscope report");
            return false;
        }
        USB::sendText("Gyro Complete");
        state = Complete;
        break;  
    default:
        return false;
        break;

    }
    return true;
}

bool setupComplete() {
    // Disabled counts as complete for the same reason as Radio: the boot
    // sequence only needs to know it may move on. Every getter keeps returning
    // its zero-initialized value, so the control loop sees a level, still
    // vehicle rather than stale orientation.
    return state == GyroSetupStates::Complete ||
           state == GyroSetupStates::Disabled;
}

void update(){
    // If gyro is not initialized, skip
    if (state != GyroSetupStates::Complete) return;

    if (GYRO_INT_PIN >= 0) {
        // INT says when a transfer is waiting, so the bus is only touched when
        // there is something to read.
        for (uint8_t i = 0; i < MAX_TRANSFERS_PER_UPDATE; i++) {
            if (digitalRead(GYRO_INT_PIN) != LOW) return;

            // INT stays low until the read starts, so the latest falling edge
            // is the one for the transfer about to be read.
            noInterrupts();
            const uint32_t edge = lastIntUs;
            interrupts();
            serviceRefFromInt = (micros() - edge) < INT_STALE_US;
            serviceRefUs = serviceRefFromInt ? edge : micros();

            sh2_service(); // Reports arrive in onSensorEvent()
        }
        return;
    }

    // No INT line: reports arrive at up to 400 Hz; poll every 2 ms so most
    // main-loop iterations skip the I2C call entirely.
    static elapsedMicros sincePoll = 0;
    if (sincePoll < 2000) return;
    sincePoll = 0;

    serviceRefFromInt = false;
    serviceRefUs = micros();
    sh2_service();
}

/**
 * Called by sh2_service() once per report in the transfer it just read.
 *
 * The Adafruit I2C HAL reports the host interrupt time as 0, so the SH-2
 * timestamp works out to the report's age at the INT edge, negative, in
 * 100 us steps. Adding it to the edge time gives the sample time.
 */
static void onSensorEvent(void*, sh2_SensorEvent_t* event) {
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) return;

    const int32_t ageUs = static_cast<int32_t>(static_cast<uint32_t>(event->timestamp_uS));

    Logger::ImuRecord rec{};
    rec.sensorId = value.sensorId;
    rec.sequence = value.sequence;
    rec.status = value.status;
    rec.flags = serviceRefFromInt ? Logger::IMU_FLAG_INT_TIMED : 0;
    rec.intUs = serviceRefUs;
    rec.sampleUs = serviceRefUs + ageUs;

    switch (value.sensorId) {
    case SH2_GYROSCOPE_CALIBRATED:
        rec.v[0] = value.un.gyroscope.x;
        rec.v[1] = value.un.gyroscope.y;
        rec.v[2] = value.un.gyroscope.z;
        Logger::recordImu(rec);
        break;
    case SH2_GAME_ROTATION_VECTOR:
        rec.v[0] = value.un.gameRotationVector.i;
        rec.v[1] = value.un.gameRotationVector.j;
        rec.v[2] = value.un.gameRotationVector.k;
        rec.v[3] = value.un.gameRotationVector.real;
        Logger::recordImu(rec);
        break;
    case SH2_LINEAR_ACCELERATION:
        rec.v[0] = value.un.linearAcceleration.x;
        rec.v[1] = value.un.linearAcceleration.y;
        rec.v[2] = value.un.linearAcceleration.z;
        Logger::recordImu(rec);
        break;
    default:
        break;
    }

    handleSensorValue(value);
}

/** Fold one report into the orientation and dead-reckoning state. */
static void handleSensorValue(const sh2_SensorValue_t& sensorValue) {
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

float getPitch() {
    return ypr.pitch;
}

float getYaw() {
    return ypr.yaw;
}

float getRoll() {
    return ypr.roll;
}


static void updateDeadReckoning(float wX, float wY, float wZ) {
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



static void quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees) {

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

static void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, bool degrees) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, degrees);
}

static void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, degrees);
}

// Transforms raw sensor acceleration into stable world-frame acceleration
static void transformToWorldFrame(float qW, float qX, float qY, float qZ, 
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



static elapsedMillis debugTimer;
static uint8_t report = 0;
static void debug() {
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

    if ( debugTimer > 500 ) {
        char debugOut[128];

        switch (report)
        {
        case 0:
            sprintf(debugOut, "Pitch %.3f, Yaw %.3f, Roll %.3f", getPitch(), getYaw(), getRoll());
            USB::sendText(debugOut, strlen(debugOut));
            break;
        case 1:
            sprintf(debugOut, "PosX %.3f, PosY %.3f, PosZ %.3f", droneState.position.x, droneState.position.y, droneState.position.z);
            USB::sendText(debugOut, strlen(debugOut));
            break;
        default:
            break;
        }

        report = !report;

        debugTimer -= 500;
    }
}

} // namespace Gyro
