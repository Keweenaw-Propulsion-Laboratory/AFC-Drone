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
static constexpr uint8_t USB_SYNC_0 = 0xA5;
static constexpr uint8_t USB_SYNC_1 = 0x5A;
static constexpr size_t USB_FRAME_OVERHEAD = 2 + 1 + 4 + 2;

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
    radio_Message radio_message;
};

// Payload for USB type RADIO_PACKET.  RadioHead carries its headers outside
// the eight-byte RF payload, so retain them explicitly for the USB decoder.
struct __attribute__((packed)) usb_radio_packet_t {
    usb_radio_direction direction;
    uint8_t packetNum;
    radio_MessageType type;
    radio_Message message;
};


struct __attribute__((packed)) usb_header_t {
    uint16_t packetNum;
    usb_message_types type;
    uint8_t packetLength;
};

static_assert(sizeof(usb_header_t) == 4, "USB header wire size changed");

struct __attribute__((packed)) usb_packet_t {
    usb_header_t header;
    usb_message_t data;

    usb_packet_t(int = 0) : header{0, (usb_message_types) 0, 0} {}
};

static_assert(sizeof(usb_packet_t) <= 64, "USB packets must be 64 bytes or less");
static_assert(sizeof(usb_command_t) == 8, "USB command wire size changed");
static_assert(sizeof(usb_telemetry_t) <= MAX_DATA_LEN, "Telemetry exceeds USB payload limit");
static_assert(sizeof(usb_radio_packet_t) == 11, "USB radio relay wire size changed");





static Circular_Buffer<usb_packet_t, 16> usb_tx_buffer;
static Circular_Buffer<usb_packet_t, 16> usb_rx_buffer;

static uint16_t usb_global_packet_number = 0;

static uint16_t usb_crc16_update(uint16_t crc, uint8_t value) {
    crc ^= static_cast<uint16_t>(value) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                              : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

static uint16_t usb_packet_crc(const usb_packet_t& packet) {
    uint16_t crc = 0xFFFF; // CRC-16/CCITT-FALSE initial value
    const uint8_t* header = reinterpret_cast<const uint8_t*>(&packet.header);
    for (size_t i = 0; i < sizeof(usb_header_t); ++i) {
        crc = usb_crc16_update(crc, header[i]);
    }
    for (uint8_t i = 0; i < packet.header.packetLength; ++i) {
        crc = usb_crc16_update(crc, packet.data.raw[i]);
    }
    return crc;
}

static bool usb_is_valid_rx_header(const usb_header_t& header) {
    return header.type == usb_message_types::COMMAND &&
           header.packetLength == sizeof(usb_command_t);
}



/**
 * Periodic USB function
 * Reads any new USB packets into the rx buffer
 * Sends out any packets in the tx buffer
 */
void usb_update() {

    if (!Serial) {
        return;
    }

    enum class RxState { FIND_SYNC_0, FIND_SYNC_1, READ_VERSION, READ_HEADER, READ_PAYLOAD, READ_CRC_0, READ_CRC_1 };
    static RxState rx_state = RxState::FIND_SYNC_0;
    static usb_packet_t temp_rx_pkt;
    static uint8_t rxBytesRead = 0;
    static uint8_t crc_bytes[2];

    // USB CDC is a byte stream.  Sync, version, length validation, and CRC let
    // this state machine recover after a partial or corrupted frame.
    while (Serial.available() > 0) {
        const uint8_t byte = Serial.read();
        if (rx_state == RxState::FIND_SYNC_0) {
            if (byte == USB_SYNC_0) {
                rx_state = RxState::FIND_SYNC_1;
            }
        } else if (rx_state == RxState::FIND_SYNC_1) {
            if (byte == USB_SYNC_1) {
                rx_state = RxState::READ_VERSION;
            } else if (byte != USB_SYNC_0) {
                rx_state = RxState::FIND_SYNC_0;
            }
        } else if (rx_state == RxState::READ_VERSION) {
            rxBytesRead = 0;
            rx_state = (byte == USB_PROTOCOL_VERSION)
                ? RxState::READ_HEADER : RxState::FIND_SYNC_0;
        } else if (rx_state == RxState::READ_HEADER) {
            uint8_t* headerPtr = reinterpret_cast<uint8_t*>(&temp_rx_pkt);
            headerPtr[rxBytesRead++] = byte;

            if (rxBytesRead == sizeof(usb_header_t)) {
                rxBytesRead = 0;
                if (usb_is_valid_rx_header(temp_rx_pkt.header)) {
                    rx_state = RxState::READ_PAYLOAD;
                } else {
                    rx_state = RxState::FIND_SYNC_0;
                }
            }
        } else if (rx_state == RxState::READ_PAYLOAD) {

            temp_rx_pkt.data.raw[rxBytesRead++] = byte;

            if (rxBytesRead == temp_rx_pkt.header.packetLength) {
                rxBytesRead = 0;
                rx_state = RxState::READ_CRC_0;
            }
       } else if (rx_state == RxState::READ_CRC_0) {
            crc_bytes[0] = byte;
            rx_state = RxState::READ_CRC_1;
       } else { // READ_CRC_1
            crc_bytes[1] = byte;
            const uint16_t received_crc = static_cast<uint16_t>(crc_bytes[0]) |
                                          (static_cast<uint16_t>(crc_bytes[1]) << 8);
            if (received_crc == usb_packet_crc(temp_rx_pkt) && usb_rx_buffer.size() < 16) {
                usb_rx_buffer.push_back(temp_rx_pkt);
            }
            rx_state = RxState::FIND_SYNC_0;
       }

    }

    // Transmit
    if (usb_tx_buffer.size() > 0) {
        // Circular_Buffer::front() returns storage for its optional multi-byte
        // mode, not the normal object queue used here.  peek() returns the
        // actual queued packet and therefore its real payload length.
        const usb_packet_t pkt = usb_tx_buffer.peek();
        const size_t tx_bytes = USB_FRAME_OVERHEAD + pkt.header.packetLength;

        // Check if the outbound buffer has enough room
        if (Serial.availableForWrite() >= (int) tx_bytes) {
            usb_packet_t tx_packet = usb_tx_buffer.pop_front();
            uint8_t frame[USB_FRAME_OVERHEAD + MAX_DATA_LEN];
            frame[0] = USB_SYNC_0;
            frame[1] = USB_SYNC_1;
            frame[2] = USB_PROTOCOL_VERSION;
            memcpy(frame + 3, &tx_packet.header, sizeof(usb_header_t));
            memcpy(frame + 3 + sizeof(usb_header_t), tx_packet.data.raw,
                   tx_packet.header.packetLength);
            const uint16_t crc = usb_packet_crc(tx_packet);
            frame[tx_bytes - 2] = static_cast<uint8_t>(crc);
            frame[tx_bytes - 1] = static_cast<uint8_t>(crc >> 8);
            Serial.write(frame, tx_bytes);
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
    if (length < 0 || length > MAX_DATA_LEN || usb_tx_buffer.size() >= 16) {
        return;
    }
    usb_header_t header {usb_global_packet_number++, type, (uint8_t) length};
    usb_packet_t packet;

    memcpy(packet.data.raw, data.raw, length );
    memcpy(&packet.header, &header, sizeof(usb_header_t));

    usb_tx_buffer.push_back(packet);


}

void usb_radio_relay(const radio_Message& message, radio_MessageType type,
                     uint8_t packetNum, usb_radio_direction direction) {
    usb_message_t tx_message{};
    usb_radio_packet_t relay{direction, packetNum, type, message};
    memcpy(tx_message.raw, &relay, sizeof(relay));
    usb_send(tx_message, usb_message_types::RADIO_PACKET, sizeof(relay));
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
        
        // Determine size for this chunk (up to MAX_DATA_LEN bytes)
        uint8_t chunkSize = (length > MAX_DATA_LEN) ? MAX_DATA_LEN : static_cast<uint8_t>(length);

        memcpy(tx_message.raw, message + offset, chunkSize);

        usb_send(tx_message, usb_message_types::DEBUG_TEXT, chunkSize);

        offset += chunkSize;
        length -= chunkSize;
    }
}

void usb_send_telemetry() {
    usb_message_t tx_message;

    tx_message.telemetry.loopTimeAvg = drone_rollAvg;
    tx_message.telemetry.loopTimeMax = Drone::worstTime;
    tx_message.telemetry.runTime = millis() / 1000;
    tx_message.telemetry.rssi = radio_lastRssi;
    tx_message.telemetry.currentMode = (uint8_t) Drone::state;
    tx_message.telemetry.gimbalPitch = gimbal_pitch;
    tx_message.telemetry.gimbalYaw = gimbal_yaw;
    tx_message.telemetry.topServoSet = gimbal_topServo;
    tx_message.telemetry.bottomServoSet = gimbal_botServo;
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
    tx_message.telemetry.latitude = 38.78152975539397f;
    tx_message.telemetry.longitude = -90.49115790740485f;

    usb_send(tx_message, usb_message_types::TELEMETRY, sizeof(usb_telemetry_t));
}

void usb_radio_relay() {


}
