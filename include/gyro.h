#include <Adafruit_BNO08x.h>

#define GYRO_RESET -1

class Gyro{
    public: 
    struct euler_t {
        float yaw;
        float pitch;
        float roll;
    };

    static euler_t ypr;

    // Functions
    static void setup();
    static void quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees = false);
    static void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, bool degrees = false);
    static void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees = false);

    static void debug();

    static Adafruit_BNO08x gyro;
    static sh2_SensorValue_t sensorValue;

};