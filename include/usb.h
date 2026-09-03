#pragma once
#include "Arduino.h"

// These are defined in radio.h.  Forward declarations keep usb.h independent
// of the radio driver while allowing the relay API to use the shared types.
union radio_Message;
enum class radio_MessageType : uint8_t;

enum class usb_message_types : uint8_t {
    RAW = 0, // Explicit value
    DEBUG_TEXT = 1,
    RADIO_PACKET = 2, 
    TELEMETRY = 3,
    COMMAND = 4,
    CONFIG = 5,
};

// Wire format: 0xA5 0x5A, version, packetNum (little-endian), type, length,
// payload, CRC-16/CCITT-FALSE (little-endian). CRC excludes the sync bytes.
static constexpr uint8_t USB_PROTOCOL_VERSION = 1;

enum class usb_radio_direction : uint8_t {
    RECEIVED = 0,
    SENT = 1,
};

void usb_update();

/**
 * Send USB debug messages
 */
void usb_send_text(const char* message, int length);

void usb_send_telemetry();
void usb_radio_relay(const radio_Message& message, radio_MessageType type,
                     uint8_t packetNum, usb_radio_direction direction);
