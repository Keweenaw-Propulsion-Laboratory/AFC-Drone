#pragma once
#include <cstdint>

extern const uint8_t CONFIG_VERSION;
/**
 * Config structure in non-volitile memory. 
 * 
 * All configs should be loaded upon boot and any changes be saved before or 
 * after fllight. Writing to EEPROM initiates a blocked call which will stall
 * the control loop while it is running.
 * 
 * Any changes to this structure must be added to the migration command
 * and the current version incremented.  
 */
struct __attribute__((packed)) PersistentConfig {
    /* Config Verification 
    This is used when loading from flash to verify that the data is 
    valid and not corrupted.*/

    uint32_t magic; // = AERE
    uint16_t version; // Current config version. Use for migration
    uint16_t crc; // Checksum

    // Config values
    uint8_t txPowerDbm;
    bool usbRelayEnabled;
    bool radioEnabled;
    bool skipRadioHandshake;
    int16_t gimbalPitchOffset;
    int16_t gimbalYawOffset;
    int8_t motor1offset;
    int8_t motor2offset;
};

/** Identifies a single config setting for config_set(). */
enum class ConfigKey : uint16_t {
    DebugMode,
    TxPowerDbm,
    UsbRelayEnabled,
    RadioEnabled,
    SkipRadioHandshake,
    GimbalPitchOffset,
    GimbalYawOffset,
    Motor1Offset,
    Motor2Offset,
};

/**Valid config opperations */
enum class ConfigOp : uint8_t {
    READ = 1,
    SET = 2,
    READ_RESPONSE = 0x81,
    SET_RESPONSE = 0x82,

    ZERO_ALL = 255, // Zeros out all config memory.
};

/** Result of a request to change one persisted setting. */
enum class ConfigResult : uint8_t {
    OK,
    INVALID_VALUE,
    INVALID_KEY,
    UNSAFE_STATE,
    UNKNOWN_VERSION,
    UNKNOWN_OP

};

/**
 * Union of the Config opperation and the result. 
 * 
 * Combines the enum values
 */
union ConfigState {
    ConfigOp operation;
    ConfigResult result;
};

struct ConfigUpdate {
    ConfigKey key;
    int32_t value;
};

/**
 * Load saved configs from the non-volitile flash. 
 */
void config_load();

/**
 * Save configs to the non-voltile flash.
 */
void config_save();

/**
 * Helper command to return a snapshot of the config struct
 */
const PersistentConfig& config_get();

/**
 * Helper command to return the current editable config struct.
 */
PersistentConfig& config_mutableGet();

/**
 * Helper command to set all values back to default.
 */
void restoreDefaults();

/**
 * Validate, apply, and persist one setting.
 *
 * The value is signed so the same interface supports booleans, unsigned
 * settings, and signed calibration offsets. Boolean values must be 0 or 1.
 * Configuration is never written while the vehicle is in FLIGHT.
 */
ConfigResult config_set(ConfigKey key, int32_t value);

/** Apply all valid updates and write EEPROM at most once. */
void config_set_batch(const ConfigUpdate* updates, ConfigResult* results,
                      uint8_t count);

/**
 * Return one setting using the same int32_t representation accepted by
 * config_set(). Boolean settings are returned as 0 or 1.
 * 
 * @param status returns 0 on success. -1 on failure
 */
int32_t config_read(ConfigKey key, ConfigResult& status);
