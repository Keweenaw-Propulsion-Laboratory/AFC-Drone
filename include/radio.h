#include "RH_RF69.h"
#include "configs.h"

#define RF69_FREQ 915.0

constexpr int RFM69_CS = 10;  //
constexpr int RFM69_INT = 40; //
constexpr int RFM69_RST = 41;  // "A"
constexpr int LED = 13;

extern int16_t radio_avgRSSI;


        /**
        * Keeps track of the different stages of Radio setup
        */
        enum class radio_SetupStates : uint8_t{
            RESET1,
            RESET2,
            RADIO_INIT,
            SET_CONFIG,
            SEND_CONN,
            WAIT_ACK,
            COMPLETE
        };

        bool radio_setup();
        bool radio_setupComplete();

        enum class RadioStates : uint8_t{
            HARDWARE_INIT,
            TRANSMIT,
            RECV,
            READY
        };

        struct __attribute__((packed)) radio_Header {
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
            int16_t gimbalPitchNorm; // Normalized gimbal pitch
            int16_t gimbalYawNorm;  // Normalized gimbal yaw
            uint16_t topServoSet; // The raw setpoint in degrees 
            uint16_t bottomServoSet; // The raw setpoint in degrees. 
        };


        struct __attribute__((packed)) StatusMsg2_t {
            uint16_t motor1set; // Motor 1 set point
            uint16_t motor2set; // Motor 2 set point
            uint16_t voltage; // Current voltage of the battery. 
            uint16_t empty; // Reserved
        };

        struct __attribute__((packed)) StatusMsg3_t {
            int16_t qR;
            int16_t qI;
            int16_t qJ;
            int16_t qK;
        };

        struct __attribute__((packed)) StatusMsg4_t {
            int16_t accelX;
            int16_t accelY;
            int16_t accelZ;
            int16_t empty;
        };

        struct __attribute__((packed)) StatusMsg5_t {
            int16_t velX;
            int16_t velY;
            int16_t velZ;
            int16_t empty;
        };

        struct __attribute__((packed)) StatusMsg6_t {
            int16_t posX;
            int16_t posY;
            int16_t posZ;
            int16_t empty;
        };

        struct __attribute__((packed)) StatusMsg7_t {
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

        struct __attribute__((packed)) ConfigPacket{
            uint8_t version;
            ConfigState state;
            ConfigKey configKey;
            uint32_t value;
        };

        // uinion all of the radio messages for type safety
        union radio_Message {
            uint64_t raw;
            StatusMsg0_t status0;
            StatusMsg1_t status1;
            StatusMsg2_t status2;
            StatusMsg3_t status3;
            StatusMsg4_t status4;
            StatusMsg5_t status5;
            StatusMsg6_t status6;
            Command_t command;
            ConfigPacket config;

            char textArray[8];
            
        };
    
        // Ensure that all messages are 8 bytes
        static_assert(sizeof(radio_Message) == sizeof(uint64_t), "Radio messages must be 8 bytes");
        
        // MARK: Message structure
        enum class radio_MessageType : uint8_t {
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

    struct __attribute__((packed)) radio_Packet {
        radio_Message message;
        radio_MessageType type;

        // 1. Constructor allowing implicit conversion from '0' (fixes the Circular_Buffer fallback)
        radio_Packet(int = 0) 
            : message{0}, type(radio_MessageType::SETUP) {}

        // 2. Multi-argument constructor for initializing packets cleanly
        radio_Packet(radio_Message msg, radio_MessageType t) 
            : message(msg), type(t) {}
    };

        // Fixed-point scale factors for packing Gyro floats into int16 status fields.
        constexpr float RADIO_QUAT_SCALE = 32767.0f;  // Quaternion components are unit range [-1, 1]
        constexpr float RADIO_ACCEL_SCALE = 1000.0f;  // m/s^2 -> mm/s^2
        constexpr float RADIO_VEL_SCALE = 1000.0f;    // m/s -> mm/s
        constexpr float RADIO_POS_SCALE = 100.0f;     // m -> cm

        inline int16_t radio_floatToFixed(float value, float scale) {
            float scaled = value * scale;
            if (scaled > 32767.0f) scaled = 32767.0f;
            if (scaled < -32768.0f) scaled = -32768.0f;
            return static_cast<int16_t>(scaled);
        }

        void radio_sendStatus0();
        void radio_sendStatus1();
        void radio_sendStatus2();
        void radio_sendStatus3();
        void radio_sendStatus4();
        void radio_sendStatus5();
        void radio_sendStatus6();
        

        /**
         * All of the tasks that the radio needs to do during the periodic loop
         */
        void radio_update();
        
    union ACK {
        uint8_t array[8];
        uint64_t raw;
    };

    inline constexpr ACK ack = {0x69,0x69,0x69,0x69,0x69,0x69,0x69,0x69};
