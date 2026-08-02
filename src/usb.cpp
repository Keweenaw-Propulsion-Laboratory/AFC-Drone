/* 
The purpose of this file is to handle all USB communications
between the Drone and a pysically connected Serial terminal
*/

#include "usb.h"
#include "drone.h"
#include "radio.h"
#include "gimbal.h"
#include "motor.h"

#include "Arduino.h"
#include "circular_buffer.h"

static constexpr uint8_t MAX_DATA_LEN = 60; /** Max usb data length */

struct __attribute__((packed)) usb_command_t {
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

struct __attribute__((packed)) usb_telemetry_t {
    // Status 0
    uint16_t loopTimeAvg;
    uint16_t loopTimeMax;
    uint16_t runTime;
    uint8_t rssi;
    uint8_t currentMode;
    // Status 1
    int16_t gimbalPitch;
    int16_t gimbalYaw;
    int16_t topServoSet;
    int16_t bottomServoSet;
    // Status 2
    uint8_t motor1Set;
    uint8_t motor2Set;
    uint16_t voltage;
    // Status 3
    int16_t qR;
    int16_t qI;
    int16_t qJ;
    int16_t qK;
    // Status 4
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    // Status 5
    int16_t velX;
    int16_t velY;
    int16_t velZ;
    // Status 6
    int16_t posX;
    int16_t posY;
    int16_t posZ;
    // Status 7
    float latitude;
    float longitude;
    // 50 / 60 bytes used

};

union __attribute__((packed)) usb_message_t {
    uint8_t raw[MAX_DATA_LEN];
    usb_command_t command;
    usb_telemetry_t telemetry;
    Radio::RadioMessage radio_message;
};


struct __attribute__((packed)) usb_header_t {
    uint16_t packetNum;
    usb_message_types type;
    uint8_t packetLength;
};

struct __attribute__((packed)) usb_packet_t {
    usb_header_t header;
    usb_message_t data;

    usb_packet_t(int = 0) : header{0, (usb_message_types) 0, 0} {}
};

static_assert(sizeof(usb_packet_t) <= 64, "USB packets must be 64 bytes or less");





static Circular_Buffer<usb_packet_t, 16> usb_tx_buffer;
static Circular_Buffer<usb_packet_t, 16> usb_rx_buffer;

static uint16_t usb_global_packet_number = 0;



/**
 * Periodic USB function
 * Reads any new USB packets into the rx buffer
 * Sends out any packets in the tx buffer
 */
void usb_update() {

    if (!Serial) {
        return;
    }

    enum class RxState { READ_HEADER, READ_PAYLOAD };
    static RxState rx_state = RxState::READ_HEADER;
    static usb_packet_t temp_rx_pkt;
    static uint8_t rxBytesRead = 0;

    // Read incoming messages
    // Since packets come through as individual bytes we need to only transition
    // states once we read the required 4 byte header
    while (Serial.available() > 0) {
        if( rx_state == RxState::READ_HEADER ){
            uint8_t* headerPtr = reinterpret_cast<uint8_t*>(&temp_rx_pkt);
            headerPtr[rxBytesRead++] = Serial.read();

            // If all of the header bytes have been read -> transition states
            if (rxBytesRead == sizeof(usb_header_t)) {
                rxBytesRead = 0;
                
                if (temp_rx_pkt.header.packetLength > MAX_DATA_LEN) {
                    temp_rx_pkt.header.packetLength = MAX_DATA_LEN;
                }
                if (temp_rx_pkt.header.packetLength == 0) {
                    usb_rx_buffer.push_back(temp_rx_pkt);
                    rx_state = RxState::READ_HEADER;
                } else {
                    rx_state = RxState::READ_PAYLOAD;
                }
            }
        } else if (rx_state == RxState::READ_PAYLOAD) {

            temp_rx_pkt.data.raw[rxBytesRead++] = Serial.read();

            if (rxBytesRead == temp_rx_pkt.header.packetLength) {
                usb_rx_buffer.push_back(temp_rx_pkt);

                rxBytesRead = 0;
                rx_state = RxState::READ_HEADER;
            }
       }

    }

    // Transmit
    if (usb_tx_buffer.size() > 0) {
        const usb_packet_t& pkt = *usb_tx_buffer.front();

        size_t tx_bytes = sizeof(usb_header_t) + pkt.header.packetLength;
        
        // Check if the outbound buffer has enough room
        if (Serial.availableForWrite() >= (int) tx_bytes) {
            usb_packet_t tx_packet = usb_tx_buffer.pop_front();
            Serial.write(reinterpret_cast<const uint8_t*>(&tx_packet), tx_bytes);
        }
    }


    // If there are full packets in the rx buffer parse them
    if (usb_rx_buffer.size() > 0) {
        usb_packet_t pkt = usb_rx_buffer.pop_front();

        
        switch (pkt.header.type) {

        case usb_message_types::COMMAND :
        
            if (pkt.data.command.flags.targSlot == 0) {
                drone_targ0.gimbalX = pkt.data.command.gimbalX;
                drone_targ0.gimbalY = pkt.data.command.gimbalY;
                drone_targ0.motor0Speed = pkt.data.command.motor0Speed;
                drone_targ0.motor1Speed = pkt.data.command.motor1Speed;
            } else {
                drone_targ1.gimbalX = pkt.data.command.gimbalX;
                drone_targ1.gimbalY = pkt.data.command.gimbalY;
                drone_targ1.motor0Speed = pkt.data.command.motor0Speed;
                drone_targ1.motor1Speed = pkt.data.command.motor1Speed;
            }

            drone_activeSlot = pkt.data.command.flags.activeSlot;

            // Send back an ACK


            break;
        
        default:
            break;
        }
    }
}

static void usb_send(usb_message_t data, usb_message_types type, int length) {
    usb_header_t header {usb_global_packet_number++, usb_message_types::DEBUG_TEXT, (uint8_t) length};
    usb_packet_t packet;

    memcpy(packet.data.raw, data.raw, length );
    memcpy(&packet.header, &header, sizeof(usb_header_t));

    usb_tx_buffer.push_back(packet);


}

/**
 * Send usb debug messages 
 */
void usb_send_text(const char* message, int length) {
    if (message == nullptr || length <= 0 ) {
        return;
    }
    
    int offset = 0;

    while (length > 0) {
        usb_message_t tx_message;
        
        // 1. Determine size for this chunk (up to MAX_DATA_LEN bytes)
        uint8_t chunkSize = (length > MAX_DATA_LEN) ? MAX_DATA_LEN : static_cast<uint8_t>(length);

        memcpy(tx_message.raw, message + offset, chunkSize);

        usb_send(tx_message, usb_message_types::DEBUG_TEXT, chunkSize);

        offset += chunkSize;
        length -= chunkSize;
    }
}

void usb_send_telemetry() {
    usb_message_t tx_message;

    tx_message.telemetry.loopTimeAvg = Drone::rollAvg;
    tx_message.telemetry.loopTimeMax = Drone::worstTime;
    tx_message.telemetry.runTime = millis() / 1000;
    tx_message.telemetry.rssi = radio_lastRssi;
    tx_message.telemetry.currentMode = (uint8_t) Drone::state;
    tx_message.telemetry.gimbalPitch = Gimbal::getPitch();
    tx_message.telemetry.gimbalYaw = Gimbal::getYaw();
    tx_message.telemetry.topServoSet = Gimbal::getTopServo();
    tx_message.telemetry.bottomServoSet = Gimbal::getBottomServo();
    tx_message.telemetry.motor1Set = motor_bottomSetSpeed;
    tx_message.telemetry.motor2Set = motor_topSetSpeed;
    tx_message.telemetry.voltage = 0;
    tx_message.telemetry.qR = 0;
    tx_message.telemetry.qI = 0;
    tx_message.telemetry.qJ = 0;
    tx_message.telemetry.qK = 0;
    tx_message.telemetry.accelX = 0;
    tx_message.telemetry.accelY = 0;
    tx_message.telemetry.accelZ = 0;
    tx_message.telemetry.velX = 0;
    tx_message.telemetry.velY = 0;
    tx_message.telemetry.velZ = 0;
    tx_message.telemetry.posX = 0;
    tx_message.telemetry.posY = 0;
    tx_message.telemetry.posZ = 0;
    tx_message.telemetry.latitude = 0;
    tx_message.telemetry.longitude = 0;

    usb_send(tx_message, usb_message_types::TELEMETRY, sizeof(usb_telemetry_t));

}


