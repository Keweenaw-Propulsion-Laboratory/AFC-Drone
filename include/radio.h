#include "RH_RF69.h"

#define RF69_FREQ 915.0

constexpr int RFM69_CS = 10;  //
constexpr int RFM69_INT = 40; //
constexpr int RFM69_RST = 41;  // "A"
constexpr int LED = 13;



class Radio {
    public:

        /**
        * Keeps track of the different stages of Radio setup
        */
        enum class RadioSetupStates : uint8_t{
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

        enum class RadioStates : uint8_t{
            HARDWARE_INIT,
            TRANSMIT,
            RECV,
            READY
        };

        struct __attribute__((packed)) header_t {
            uint8_t msgNum;
            uint8_t packetType;
        };

        /**
         * General status messages
         */
        struct __attribute__((packed)) StatusMsg0_t {
            uint16_t loopTimeAvg; // Average loop time in micros
            uint16_t loopTimeMax; // Max loop time in micros
            uint16_t RunTime; // Time that the vehicle has been powered on in seconds
            uint8_t rssi; // The strength of the radio connection
            uint8_t currentMode; // The current mode that the vehicle is in. 
        };

        struct __attribute__((packed)) StatusMsg1_t {
            int8_t gimbalPitchNorm; // Normalized gimbal pitch
            int8_t gimbalYawNorm;  // Normalized gimbal yaw
            uint8_t topServoSet; // The raw setpoint in degrees 
            uint8_t bottomServoSet; // The raw setpoint in degrees. 
            uint8_t motor1set; // Motor 1 set point
            uint8_t motor2set; // Motor 2 set point
            uint16_t voltage; // Current voltage of the battery. 
        };

        struct __attribute__((packed)) StatusMsg2_t {
            int16_t qR;
            int16_t qI;
            int16_t qJ;
            int16_t qK;
        };

        struct __attribute__((packed)) StatusMsg3_t {
            int16_t accelX;
            int16_t accelY;
            int16_t accelZ;
            int16_t empty;
        };

        struct __attribute__((packed)) StatusMsg4_t {
            int16_t velX;
            int16_t velY;
            int16_t velZ;
            int16_t empty;
        };

        struct __attribute__((packed)) StatusMsg5_t {
            int16_t posX;
            int16_t posY;
            int16_t posZ;
            int16_t empty;
        };

        struct __attribute__((packed)) StatusMsg6_t {
            float latitude;
            float longitude;
        };

        

        struct __attribute__((packed)) Command_t {
            struct __attribute__((packed)) flags {
                uint8_t targSlot : 1; /**The slot to be configured */
                uint8_t activeSlot : 1; /** The slot to be currently active */
                uint8_t empty : 6; // 6 Unused flags
            } flags;
            int16_t gimbalX;
            int16_t gimbalY;
            uint8_t motor0Speed;
            uint8_t motor1Speed;
            uint8_t empty0; // Unused command field

        };

        // uinion all of the radio messages for type safety
        union RadioMessage {
            uint64_t raw;
            StatusMsg0_t status0;
            StatusMsg1_t status1;
            StatusMsg2_t status2;
            StatusMsg3_t status3;
            StatusMsg4_t status4;
            StatusMsg5_t status5;
            StatusMsg6_t status6;
            Command_t command;
            char textArray[8];
            
        };
    
        // Ensure that all messages are 8 bytes
        static_assert(sizeof(RadioMessage) == sizeof(uint64_t), "Radio messages must be 8 bytes");
        
        // MARK: Message structure
        enum class MessageType : uint8_t {
            SETUP = 0,
            STATUS0 = 1,
            STATUS1 = 2,
            STATUS2 = 3,
            STATUS3 = 4,
            STATUS4 = 5,
            STATUS5 = 6,
            STATUS6 = 7,
            COMMAND = 8,
            CONFIG = 9,

        };


        static void sendStatus0();
        static void sendStatus1();
        static void sendStatus2();
        static void sendStatus3();
        static void sendStatus4();
        static void sendStatus5();
        static void sendStatus6();
        
        static void sendMessage(RadioMessage data, MessageType type);
               

        static bool getMessage(uint8_t (&buffer)[RH_RF69_MAX_MESSAGE_LEN]
                                , uint8_t& bufferLength );

        /**
         * All of the tasks that the radio needs to do during the periodic loop
         */
        static void update();
        


    private:
    // static RH_RF69 radio; // Singleton Radio object
    // static uint8_t globalPacketNum; // The last sent or recv packet number. This should match the BaseStation
    // static int16_t lastRssi;

    // static RadioSetupStates setupState; // Current setup state

    // acknowledgement for a message.  
    static constexpr uint8_t ACK[8] = {0x69,0x69,0x69,0x69,0x69,0x69,0x69,0x69};
        

};