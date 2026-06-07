#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0

#define RFM69_CS    10  //
#define RFM69_INT   40 //
#define RFM69_RST   41  // "A"
#define LED        13

class Radio {
    public:
        enum MessageType {
            SETUP = 0,   
            TELEMETRY = 1,
            LOOPTIMES = 2
        };

        enum RadioStates {
            HARDWARE_INIT,

        };


        static bool setup();
        static bool setupComplete();
        static void sendMessage(uint8_t data[], uint8_t dataSize, MessageType type);
        static bool getMessage(uint8_t (&buffer)[RH_RF69_MAX_MESSAGE_LEN]
                                , uint8_t& bufferLength );
        


    private:
    static RH_RF69 radio;
    static uint8_t packetNum;
    
    static RadioSetupStates setupState;

    static constexpr uint8_t ACK[8] = {0x69,0x69,0x69,0x69,0x69,0x69,0x69,0x69};

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