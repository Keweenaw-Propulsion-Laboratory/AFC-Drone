#include <SPI.h>
#include <RH_RF95.h>

#define RF69_FREQ 915.0

#define RFM69_CS    4  //
#define RFM69_INT   3  //
#define RFM69_RST   2  // "A"
#define LED        13

class Radio {
    public:
        void setup();
        void sendMessage();
        void getMessage();
        

    private:
    static RH_RF95 radio;

    #pragma pack(push, 1)
    /**
     * General status message
     */
    struct statusMsg1 {
        uint8_t msgNum; // Packet number
        //Data
        uint8_t gimbalPitch;
        uint8_t gimbalYaw;
        uint8_t motor1Power;
        uint8_t motor2Power;
        uint8_t gyroPitch;
        uint8_t gyroYaw;
        uint8_t gyroRoll;
    };
    #pragma pack(pop)

    #pragma pack(push, 1)
    struct statusMsg2 {
        uint8_t msgNum;

        uint8_t voltage; // Battery Voltage
        
        uint8_t gyroAccelX;
        uint8_t gyroAccelY;
        uint8_t gyroAccelZ;
        uint8_t posX;
        uint8_t posY;
        uint8_t posZ;

    };
    #pragma pack(pop)

    #pragma pack(push, 1)
    /**
     * Gimbal data message
     */
    struct GyroData {
        uint8_t msgNum; // Packet Number
        // TODO Implement gyro data message
        

    };
    #pragma pack(pop)
};