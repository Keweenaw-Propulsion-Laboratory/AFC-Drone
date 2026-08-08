/* The purpose of this file will be to hold configurable
settings. These settings should be stored to non-volitile
memory in order to persist between system power cycles.
*/

#pragma once
#include <cstdint>

struct __attribute__((packed)) PersistentConfig {
    // Config verification
    uint32_t magic; // = AERE
    uint16_t version; // Current config version. Use for migration
    uint16_t crc; // Checksum

    // Config values
    uint8_t txPowerDbm;
    bool usbRelayEnabled;
    bool radioEnabled;
    int16_t gimbalPitchOffset;
    int16_t gimbalYawOffset;
    int8_t motor1offset;
    int8_t motor2offset;
};

void config_load();
void config_save();
const PersistentConfig& config_get();
PersistentConfig config_mutableGet();
void restoreDefaults();

