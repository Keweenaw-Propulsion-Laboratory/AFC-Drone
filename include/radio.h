#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0

#define RFM69_CS    10  //
#define RFM69_INT   40 //
#define RFM69_RST   41  // "A"
#define LED        13

class Radio {
    public:

        /**
        * Keeps track of the different stages of Radio setup
        */
        enum RadioSetupStates: uint8_t{
            RESET1,
            RESET2,
            RADIO_INIT,
            SET_CONFIG,
            SEND_CONN,
            WAIT_ACK,
            COMPLETE
        };

        static bool setup();
        static bool setupComplete();


        // MARK: Message structure
        enum MessageType {
            SETUP = 0,   
            TELEMETRY = 1,
            LOOPTIMES = 2
        };

        enum RadioStates {
            HARDWARE_INIT,
            TRANSMIT,
            RECV,
            READY
        };

        #pragma pack(push, 1)
        struct header_t {
            uint8_t msgNum;
            uint8_t packetType;
        };

        /**
         * General status messages
         */
        #pragma pack(push, 1)   // Packs the struct so it takes up exactly as much space as we tell it.
                                // The compiler would otherwise align everything to 4 bytes 
        struct statusMsg0 {
            
        };
        #pragma pack(pop)

        #pragma pack(push, 1)
        struct statusMsg1 {
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


        static void sendMessage(uint8_t data[], uint8_t dataSize, MessageType type);
        static bool getMessage(uint8_t (&buffer)[RH_RF69_MAX_MESSAGE_LEN]
                                , uint8_t& bufferLength );

        /**
         * All of the tasks that the radio needs to do during the periodic loop
         */
        static void update();
        


    private:
    static RH_RF69 radio; // Singleton Radio object
    static uint8_t packetNum; // The last sent or recv packet number. This should match the BaseStation
    
    static RadioSetupStates setupState; // Current setup state

    // acknowledgement for a message.  
    static constexpr uint8_t ACK[8] = {0x69,0x69,0x69,0x69,0x69,0x69,0x69,0x69};


};