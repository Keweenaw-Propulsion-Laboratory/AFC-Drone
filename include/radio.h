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
            STATUS0 = 1,
            STATUS1 = 2,
            STATUS2 = 3,
            STATUS3 = 4,
            STATUS4 = 5,
            STATUS5 = 6,
            STATUS6 = 7,
            COMMAND = 8

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
        struct StatusMsg0_t {
            uint16_t loopTimeAvg; // Average loop time in micros
            uint16_t loopTimeMax; // Max loop time in micros
            uint16_t RunTime; // Time that the vehicle has been powered on in seconds
            uint8_t rssi; // The strength of the radio connection
            uint8_t currentMode; // The current mode that the vehicle is in. 
        };
        #pragma pack(pop)

        #pragma pack(push, 1)
        struct StatusMsg1_t {
            int8_t gimbalPitchNorm; // Normalized gimbal pitch
            int8_t gimbalYawNorm;  // Normalized gimbal yaw
            uint8_t topServoSet; // The raw setpoint in degrees 
            uint8_t bottomServoSet; // The raw setpoint in degrees. 
            uint8_t motor1set; // Motor 1 set point
            uint8_t motor2set; // Motor 2 set point
            uint16_t voltage; // Current voltage of the battery. 
        };
        #pragma pack(pop)

        #pragma pack(push, 1)
        struct StatusMsg2_t {
            int16_t qR;
            int16_t qI;
            int16_t qJ;
            int16_t qK;
        };
        #pragma pack(pop)

        #pragma pack(push, 1)
        struct StatusMsg3_t {
            int16_t accelX;
            int16_t accelY;
            int16_t accelZ;
            int16_t empty;
        };
        #pragma pack(pop)

        #pragma pack(push, 1)
        struct StatusMsg4_t {
            int16_t velX;
            int16_t velY;
            int16_t velZ;
            int16_t empty;
        };
        #pragma pack(pop)

        #pragma pack(push, 1)
        struct StatusMsg5_t {
            int16_t posX;
            int16_t posY;
            int16_t posZ;
            int16_t empty;
        };
        #pragma pack(pop)

        #pragma pack(push, 1)
        struct StatusMsg6_t {
            float latitude;
            float longitude;
        };
        #pragma pack(pop)

        #pragma pack(push, 1)
        struct Command_t {
            uint8_t flags;
            int8_t targetPosX;
            int8_t targetPosY;
            int8_t targetPosZ;
            uint8_t targetRoll;
            uint8_t empty0;
            uint8_t empty1;
            uint8_t empty2;
        };
        #pragma pack(pop)

        static void sendStatus0();
        static void sendStatus1();
        static void sendStatus2();
        static void sendStatus3();
        static void sendStatus4();
        static void sendStatus5();
        static void sendStatus6();
        
        // static void sendMessage(uint8_t data[], uint8_t dataSize, MessageType type);
        
        template <typename T>
        static void sendMessage(T& packetData, MessageType type) {
            if (!_initialized) return;
            if (rf69.mode() == RHGenericDriver::RHModeTx) return; // Drop if actively transmitting

            // 1. Automatically populate the mandatory header items safely
            packetData.header.packetID = globalPacketNum++;
            packetData.header.packetType = (uint8_t)type;

            // 2. Cast the entire struct directly into a raw byte array pointer 
            //    and let the compiler calculate its exact memory footprint (sizeof)
            uint8_t* rawDataPtr = (uint8_t*)&packetData;
            size_t packetSize = sizeof(T);

            // 3. Drop it directly into the hardware FIFO buffer instantly
            rf69.send(rawDataPtr, packetSize);
        };
        

        static bool getMessage(uint8_t (&buffer)[RH_RF69_MAX_MESSAGE_LEN]
                                , uint8_t& bufferLength );

        /**
         * All of the tasks that the radio needs to do during the periodic loop
         */
        static void update();
        


    private:
    static RH_RF69 radio; // Singleton Radio object
    static uint8_t globalPacketNum; // The last sent or recv packet number. This should match the BaseStation
    static int16_t lastRssi;

    static RadioSetupStates setupState; // Current setup state

    // acknowledgement for a message.  
    static constexpr uint8_t ACK[8] = {0x69,0x69,0x69,0x69,0x69,0x69,0x69,0x69};


};