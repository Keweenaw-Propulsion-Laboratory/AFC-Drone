#pragma once

#include <cstddef>
#include <cstdint>

class HardwareSerial;

/**
 * LG290P rover positioning for the drone.
 *
 * The stationary ground receiver must be configured separately in base mode,
 * using survey-in or a known antenna position. Its RTCM3 corrections must
 * reach the rover through sendCorrections() or a dedicated GNSS UART link.
 *
 * Call update() from loop() to receive fixes at 20 Hz. Position reads use the
 * cached report, so reading faster does not produce new measurements. Keep
 * coordinates as doubles to preserve the receiver's position precision.
 *
 * Uses SparkFun's LG290P GNSS Arduino Library:
 * https://github.com/sparkfun/SparkFun_LG290P_GNSS_Arduino_Library
 */
namespace GPS {

    /**
     * Enable GPS with the final wiring, before the first active setup() call.
     * GPS defaults to disabled: setup succeeds, update does nothing, position
     * reads return false and correction writes return zero.
     *
     * @param serial Dedicated MCU UART; must remain alive for the program lifetime.
     * @param uart GNSS-side UART number (1..3), independent of the MCU port number.
     * @param baud Receiver's current baud rate in bits/s (9600..3000000).
     * @return False for invalid settings or if already configured.
     * @warning Call only during startup, outside interrupt context. Connect MCU
     *          RX to GNSS TX, MCU TX to GNSS RX, and share ground.
     */
    bool configure(HardwareSerial& serial, uint8_t uart, uint32_t baud = 460800);

    /** GGA fix quality. Only RTK_FIXED indicates resolved RTK ambiguities. */
    enum class FixQuality : uint8_t {
        NO_FIX = 0,
        STANDALONE = 1,
        DIFFERENTIAL = 2,
        PRECISE = 3,
        RTK_FIXED = 4,
        RTK_FLOAT = 5,
    };

    /** One position report; coordinates describe the rover antenna. */
    struct Position {
        double latitude = 0.0;       ///< WGS84 degrees, -90 to +90.
        double longitude = 0.0;      ///< WGS84 degrees, -180 to +180.
        double altitudeMeters = 0.0; ///< Height above the WGS84 ellipsoid.
        double northMeters = 0.0;    ///< Offset north of the base antenna.
        double eastMeters = 0.0;     ///< Offset east of the base antenna.
        double upMeters = 0.0;       ///< Offset up in the base's local frame.
        FixQuality fix = FixQuality::NO_FIX;
        uint8_t satellites = 0;      ///< Satellites used in this fix.
        uint32_t ageMs = UINT32_MAX; ///< Time since receipt; UINT32_MAX if none.
        bool relativeValid = false; ///< True when the offsets have a base reference.
    };

    /**
     * Bring up the rover and configure its position stream.
     *
     * Call repeatedly during startup until setupComplete() is true. Completion
     * means GPS is disabled or the receiver is configured; it does not require
     * a satellite or RTK fix. Call configure() first to enable the receiver.
     *
     * @return True while progressing, complete or disabled; false on setup failure.
     * @warning Library commands can block for seconds during connection or
     *          restart. Call only during startup, outside interrupt context.
     */
    bool setup();

    /** @return True when GPS is disabled or the rover configuration is complete. */
    bool setupComplete();

    /**
     * Process available UART bytes and cache complete position reports.
     *
     * Call every pass through loop(). Processes at most 512 bytes per call and
     * does nothing while disabled or before setup completes; it does not wait
     * for a new fix.
     * @warning Call from loop(), never from an interrupt handler.
     */
    void update();

    /**
     * Copy the latest report without polling the receiver or consuming it.
     *
     * @param out Receives the report and its current age, even on a false return.
     * @return True for a valid fix received within the last 500 ms. False before
     *         the first report, on loss of fix, or when the report is stale.
     * @warning Check the return value and require fix == RTK_FIXED when the
     *          application needs RTK precision. Safe to read from a control ISR.
     */
    bool getPosition(Position& out);

    /**
     * Supply the ground base antenna's position for north/east/up offsets.
     *
     * Use the same reference position configured in the ground receiver.
     * Offsets become available on the next valid rover report. This only sets
     * the local reference; it does not configure the ground receiver.
     *
     * @param latitude WGS84 latitude in degrees, -90 to +90.
     * @param longitude WGS84 longitude in degrees, -180 to +180.
     * @param altitudeMeters WGS84 ellipsoid height in meters, not sea-level height.
     * @return False for non-finite coordinates or angles outside their ranges.
     * @warning Call from loop() or startup, outside interrupt context.
     */
    bool setBasePosition(double latitude, double longitude, double altitudeMeters);

    /**
     * Forward raw RTCM3 bytes received from the ground base to the rover.
     *
     * @param data Correction bytes in their original order, including framing.
     * @param length Number of bytes available.
     * @return Bytes queued, possibly fewer than length; retry the remaining bytes
     *         on later loop passes. Returns zero while disabled, before setup
     *         completes, or for null data.
     * @warning Call from loop(), never an ISR. This does not wait for TX space.
     */
    size_t sendCorrections(const uint8_t* data, size_t length);

} // namespace GPS
