#include "configs.h"

#include <EEPROM.h>
#include <cstring>
#include "drone.h"

static constexpr uint32_t CONFIG_MAGIC = 0x41455245; // AERE
const uint8_t CONFIG_VERSION = 1;
static constexpr int EEPROM_ADDRESS = 0;

PersistentConfig config{};

static void config_migrate(PersistentConfig &stored);

/**
 * Set the default values for configs here.
 */
PersistentConfig defaults()
{
    return {
        CONFIG_MAGIC,   // Magic
        CONFIG_VERSION, // Version
        0,              // CRC
        false,          // Debug mode
        true,           // USB enabled
        true,           // Radio enabled
        true,           // Skip radio handshake
        20,             // Radio transmit power
        90, // Gimbal Offset
        89, // Gimbal Offset
        0,  // Motor Offset
        0, // Motor Offset

    };
}

static uint16_t checksum(PersistentConfig config)
{
    config.crc = 0;
    const auto *bytes = reinterpret_cast<const uint8_t *>(&config);
    uint16_t sum = 0;
    for (size_t i = 0; i < sizeof(config); ++i)
    {
        sum = static_cast<uint16_t>(sum + bytes[i]);
    }
    return sum;
}

void config_save()
{
    // If drone is inflight do run blocking save to EEPROM.
    if (Drone::state == Drone::DroneStates::FLIGHT)
    {
        return;
    }

    // Needs to go last in order to verify latest information
    config.crc = checksum(config);

    EEPROM.put(EEPROM_ADDRESS, config);
}

void config_load()
{
    PersistentConfig stored{};
    EEPROM.get(EEPROM_ADDRESS, stored);

    // If configs are valid load values
    if (stored.magic == CONFIG_MAGIC &&
        stored.version == CONFIG_VERSION &&
        stored.crc == checksum(stored))
    {
        config = stored;

        // Else if the version is wrong initiate migration
    }
    else if (stored.version != CONFIG_VERSION)
    {
        config_migrate(stored);
    }
    else
    {
        // If values are unrecoverable reset to defaults.
        config = defaults();
        config_save();
    }
}

const PersistentConfig &config_get()
{

    return config;
}

PersistentConfig &config_mutableGet()
{
    return config;
}

void restoreDefaults()
{
    config = defaults();
    config_save();
}

static ConfigResult config_apply(ConfigKey key, int32_t value, bool &changed)
{
    switch (key)
    {
    case ConfigKey::DebugMode:
        if (value != 0 && value != 1 )
            return ConfigResult::INVALID_VALUE;
        changed = config.debugMode != static_cast<bool>(value);
        config.debugMode = static_cast<bool>(value);    
        break;
        
    case ConfigKey::TxPowerDbm:
        if (value < 14 || value > 20)
            return ConfigResult::INVALID_VALUE;
        changed = config.txPowerDbm != static_cast<uint8_t>(value);
        config.txPowerDbm = static_cast<uint8_t>(value);
        break;

    case ConfigKey::UsbRelayEnabled:
        if (value != 0 && value != 1)
            return ConfigResult::INVALID_VALUE;
        changed = config.usbRelayEnabled != static_cast<bool>(value);
        config.usbRelayEnabled = static_cast<bool>(value);
        break;

    case ConfigKey::RadioEnabled:
        if (value != 0 && value != 1)
            return ConfigResult::INVALID_VALUE;
        changed = config.radioEnabled != static_cast<bool>(value);
        config.radioEnabled = static_cast<bool>(value);
        break;

    case ConfigKey::SkipRadioHandshake:
        if (value != 0 && value != 1)
            return ConfigResult::INVALID_VALUE;
        changed = config.skipRadioHandshake != static_cast<bool>(value);
        config.skipRadioHandshake = static_cast<bool>(value);
        break;

    case ConfigKey::GimbalPitchOffset:
        if (value < 60 || value > 120)
            return ConfigResult::INVALID_VALUE;
        changed = config.gimbalPitchOffset != static_cast<int16_t>(value);
        config.gimbalPitchOffset = static_cast<int16_t>(value);
        break;

    case ConfigKey::GimbalYawOffset:
        if (value < 60 || value > 120)
            return ConfigResult::INVALID_VALUE;
        changed = config.gimbalYawOffset != static_cast<int16_t>(value);
        config.gimbalYawOffset = static_cast<int16_t>(value);
        break;

    case ConfigKey::Motor1Offset:
        if (value < -100 || value > 100)
            return ConfigResult::INVALID_VALUE;
        changed = config.motor1offset != static_cast<int8_t>(value);
        config.motor1offset = static_cast<int8_t>(value);
        break;

    case ConfigKey::Motor2Offset:
        if (value < -100 || value > 100)
            return ConfigResult::INVALID_VALUE;
        changed = config.motor2offset != static_cast<int8_t>(value);
        config.motor2offset = static_cast<int8_t>(value);
        break;

    default:
        return ConfigResult::INVALID_KEY;
    }

    return ConfigResult::OK;
}

ConfigResult config_set(ConfigKey key, int32_t value)
{
    if (Drone::state == Drone::DroneStates::FLIGHT)
    {
        return ConfigResult::UNSAFE_STATE;
    }

    bool changed = false;
    const ConfigResult result = config_apply(key, value, changed);

    if (changed)
    {
        config_save();
    }

    return result;
}

void config_set_batch(const ConfigUpdate *updates, ConfigResult *results,
                      uint8_t count)
{
    if (updates == nullptr)
        return;

    if (Drone::state == Drone::DroneStates::FLIGHT)
    {
        for (uint8_t i = 0; i < count; ++i)
        {
            if (results != nullptr)
                results[i] = ConfigResult::UNSAFE_STATE;
        }
        return;
    }

    bool anyChanged = false;
    for (uint8_t i = 0; i < count; ++i)
    {
        bool changed = false;
        const ConfigResult result = config_apply(updates[i].key,
                                                 updates[i].value, changed);
        if (results != nullptr)
            results[i] = result;
        anyChanged = anyChanged || changed;
    }

    if (anyChanged)
        config_save();
}

int32_t config_read(ConfigKey key, ConfigResult &status)
{
    status = ConfigResult::OK;
    switch (key)
    {
    case ConfigKey::DebugMode:
        return config.debugMode;
    case ConfigKey::TxPowerDbm:
        return config.txPowerDbm;

    case ConfigKey::UsbRelayEnabled:
        return config.usbRelayEnabled ? 1 : 0;

    case ConfigKey::RadioEnabled:
        return config.radioEnabled ? 1 : 0;

    case ConfigKey::SkipRadioHandshake:
        return config.skipRadioHandshake ? 1 : 0;

    case ConfigKey::GimbalPitchOffset:
        return config.gimbalPitchOffset;

    case ConfigKey::GimbalYawOffset:
        return config.gimbalYawOffset;

    case ConfigKey::Motor1Offset:
        return config.motor1offset;

    case ConfigKey::Motor2Offset:
        return config.motor2offset;
    }

    // All defined ConfigKey values are handled above. This protects callers
    // from a malformed value received over the radio.

    status = ConfigResult::INVALID_KEY;
    return 0;
}

static void config_migrate(PersistentConfig &stored)
{
    switch (stored.version)
    {

    case (0):
    default:
        // If there is no valid version migration, reset to defaults
        stored = defaults();
        break;
    }

    config = stored;
    // Save migrated configs
    config_save();
}
