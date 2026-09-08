#pragma once
#include "Arduino.h"

#include <cstring>

// These types are defined in radio.h. Forward-declaring them keeps usb.h
// independent of the radio driver while letting the relay API use the shared
// types. They must be declared inside namespace Radio - the namespace the
// types actually live in - and at file scope, NOT inside namespace USB.
namespace Radio {
    union Message;
    enum class MessageType : uint8_t;
}

namespace USB {

enum class MessageTypes : uint8_t {
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

enum class RadioDirection : uint8_t {
    RECEIVED = 0,
    SENT = 1,
};

void update();

/**
 * Send USB debug messages
 */
void sendText(const char* message, int length);

/**
 * Send a NUL-terminated debug string.
 *
 * Prefer this over the explicit-length overload for string literals: a
 * hand-counted length that is too short silently truncates the message, and one
 * that is too long reads past the literal.
 */
inline void sendText(const char* message) {
    if (message == nullptr) return;
    sendText(message, static_cast<int>(strlen(message)));
}

void sendTelemetry();
void radioRelay(const Radio::Message& message, Radio::MessageType type,
                     uint8_t packetNum, RadioDirection direction);
}
