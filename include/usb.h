#include "Arduino.h"

enum usb_message_types : uint8_t {
    RAW = 0, // Explicit value
    DEBUG_TEXT = 1,
    RADIO_PACKET, 
    TELEMETRY,
    COMMAND
};

void usb_update();

/**
 * Send USB debug messages
 */
void usb_send_text(const char* message, uint8_t length);

