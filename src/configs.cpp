#include "configs.h"

#include <EEPROM.h>
#include <cstring>
#include "drone.h"

static constexpr uint32_t CONFIG_MAGIC = 0x41455245; // AERE
static constexpr uint16_t CONFIG_VERSION = 1;
static constexpr int EEPROM_ADDRESS = 0;

PersistentConfig config{};

// /**
//  * The config location in eeprom in bytes
//  */
// enum class ConfigKey : uint8_t {
//     txPowerDbm = 9,
//     usbRelayEnabled = 10,
//     radioEnabled = 11,
//     gimbalXoffset = 12,
//     gimbalYoffset = 14,
//     motor1offset = 15,
//     motor2offset = 16
// };


/**
 * Set the default values for configs here. 
 */
PersistentConfig defaults() {
    return {
        CONFIG_MAGIC, // Magic
        CONFIG_VERSION, // Version
        0, // CRC
        true, // USB enabled
        true, // Radio enabled
        90,89, // Gimbal Offset
        0, 0, // Motor Offset
               
    };
}

static uint16_t checksum(PersistentConfig config) {
    config.crc = 0;
    const auto* bytes = reinterpret_cast<const uint8_t*>(&config);
    uint16_t sum = 0;
    for (size_t i = 0; i < sizeof(config); ++i) {
        sum = static_cast<uint16_t>(sum + bytes[i]);
    }
    return sum;
}

void config_save(PersistentConfig new_config) {
    if (Drone::state == Drone::DroneStates::FLIGHT) {
        return;
    }

    config = new_config;
    
    // Needs to go last in order to verify latest information
    config.crc = checksum(config);

    EEPROM.put(EEPROM_ADDRESS, config);

}

void config_load() {
    PersistentConfig stored{};
    EEPROM.get(EEPROM_ADDRESS, stored);

    if (stored.magic == CONFIG_MAGIC && 
        stored.version == CONFIG_VERSION ) {
        config = stored;
    } else {
        config = defaults();
        config_save();
    }
}

const PersistentConfig& config_get() {
    return config;
}

PersistentConfig config_mutableGet() {
    PersistentConfig temp;

    memcpy(&temp, &config, sizeof(config));
    
    return temp;
}