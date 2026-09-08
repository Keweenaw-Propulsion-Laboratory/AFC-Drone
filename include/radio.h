#pragma once

#include "RH_RF69.h"
#include "configs.h"
#include "drone.h"

namespace Radio {



extern int16_t radio_avgRSSI;


        /**
        * Stage 1 of radio bring-up: getting the RFM69 itself configured.
        *
        * This is the only part the boot state machine blocks on. It talks to
        * local hardware only, so it either completes in a few milliseconds or
        * fails outright - it can never stall waiting on another vehicle.
        */
        enum class SetupStates : uint8_t{
            RESET1,
            RESET2,
            RADIO_INIT,
            SET_CONFIG,
            COMPLETE
        };

        /**
        * Stage 2 of radio bring-up: finding the base station.
        *
        * This runs in the background from radio_update() and is deliberately
        * NOT part of Radio::setupComplete(). Losing or never finding the base
        * station must not keep the vehicle from arming, so the drone reaches
        * READY_ARMED regardless of what this reports.
        */
        enum class LinkStates : uint8_t{
            DISCONNECTED, // No ping outstanding; next poll will send one
            AWAITING_ACK, // Ping sent, waiting for the base station to answer
            CONNECTED     // Base station has acknowledged
        };

        bool setup();

        /**
         * True once the RFM69 is configured and able to send and receive.
         * Does NOT imply a base station is listening - see radio_linkConnected().
         */
        bool setupComplete();

        /** True once the base station has acknowledged our connection ping. */
        bool linkConnected();

        enum class TransceiverStates : uint8_t{
            HARDWARE_INIT,
            TRANSMIT,
            RECV,
            READY
        };

        struct __attribute__((packed)) PacketHeader {
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
            uint8_t currentMode; // The current mode that the vehicle is in. 
            uint8_t empty; // Reserved
        };

        struct __attribute__((packed)) StatusMsg1_t {
            int16_t gimbalPitchNorm; // Normalized gimbal pitch
            int16_t gimbalYawNorm;  // Normalized gimbal yaw
            uint16_t topServoSet; // The raw setpoint in degrees 
            uint16_t bottomServoSet; // The raw setpoint in degrees. 
        };


        struct __attribute__((packed)) StatusMsg2_t {
            uint16_t bottomMotorSet; // Motor 1 set point
            uint16_t topMotorSet; // Motor 2 set point
            uint16_t voltage; // Current voltage of the battery. 
            uint16_t rssi; // The strength of the radio connection
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
            uint8_t bottomMotor;
            uint8_t topMotor;
            uint8_t empty0; // Unused command field

        };

        struct __attribute__((packed)) ConfigPacket{
            uint8_t version;
            Configs::ConfigState state;
            Configs::ConfigKey configKey;
            uint32_t value;
        };

        // uinion all of the radio messages for type safety
        union Message {
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
        static_assert(sizeof(Message) == sizeof(uint64_t), "Radio messages must be 8 bytes");
        
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

    struct __attribute__((packed)) DataPacket {
        Message message;
        MessageType type;

        // 1. Constructor allowing implicit conversion from '0' (fixes the Circular_Buffer fallback)
        DataPacket(int = 0) 
            : message{0}, type(MessageType::SETUP) {}

        // 2. Multi-argument constructor for initializing packets cleanly
        DataPacket(Message msg, MessageType t) 
            : message(msg), type(t) {}
    };

    // Fixed-point scale factors for packing Gyro floats into int16 status fields.
    constexpr float RADIO_QUAT_SCALE = 32767.0f;  // Quaternion components are unit range [-1, 1]
    constexpr float RADIO_ACCEL_SCALE = 1000.0f;  // m/s^2 -> mm/s^2
    constexpr float RADIO_VEL_SCALE = 1000.0f;    // m/s -> mm/s
    constexpr float RADIO_POS_SCALE = 100.0f;     // m -> cm

    inline int16_t floatToFixed(float value, float scale) {
        float scaled = value * scale;
        if (scaled > 32767.0f) scaled = 32767.0f;
        if (scaled < -32768.0f) scaled = -32768.0f;
        return static_cast<int16_t>(scaled);
    }

    /**
     * Status packet senders. Each takes the telemetry snapshot recorded by the
     * control ISR rather than reading the live subsystems, so every packet in
     * one telemetry frame describes the same instant.
     *
     * Values the control tick cannot see - radio RSSI, battery voltage, GPS -
     * are still read live here, because they are owned by loop() context.
     */
    void sendStatus0(const Drone::Telemetry_t& t);
    void sendStatus1(const Drone::Telemetry_t& t);
    void sendStatus2(const Drone::Telemetry_t& t);
    void sendStatus3(const Drone::Telemetry_t& t);
    void sendStatus4(const Drone::Telemetry_t& t);
    void sendStatus5(const Drone::Telemetry_t& t);
    void sendStatus6(const Drone::Telemetry_t& t);
    

    /**
     * All of the tasks that the radio needs to do during the periodic loop
     */
    void update();
    
    //MARK: ACK

    union ACK {
        uint8_t array[8];
        uint64_t raw;
    };

    inline constexpr ACK ack = {0x69,0x69,0x69,0x69,0x69,0x69,0x69,0x69};
}
