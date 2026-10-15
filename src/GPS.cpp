#include "GPS.h"

#include "usb.h"

#include <Arduino.h>
// The upstream NmeaPacket header compares an int index with size_t.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <SparkFun_LG290P_GNSS.h>
#pragma GCC diagnostic pop
#include <cerrno>
#include <cmath>
#include <cstdlib>

#if defined(TEENSYDUINO)
#include <util/atomic.h>
#endif

namespace GPS {

    // Disabled until configure() supplies the final wiring during startup.
    static HardwareSerial* gpsSerial = nullptr;
    static uint8_t receiverUart = 0;
    static uint32_t gpsBaud = 0;
    static constexpr uint16_t FIX_INTERVAL_MS = 50; // 20 Hz rover measurements.
    static constexpr uint32_t MAX_POSITION_AGE_MS = 500;
    static constexpr int MAX_BYTES_PER_UPDATE = 512;
    static constexpr uint16_t FLIGHT_NAV_MODE = 5; // Quectel dynamic flight mode.

    enum class SetupStates : uint8_t {
        DISABLED,
        CONNECT,
        CONFIGURE,
        STREAM,
        SAVE,
        RESTART,
        COMPLETE,
        FAILED,
    };

    static LG290P receiver;
    static SetupStates state = SetupStates::DISABLED;
    static Position latestPosition;
    static uint32_t lastPositionMs = 0;
    static bool reportReceived = false;
    static bool baseConfigured = false;
    static double baseX = 0.0, baseY = 0.0, baseZ = 0.0;
    static double baseSinLat = 0.0, baseCosLat = 1.0;
    static double baseSinLon = 0.0, baseCosLon = 1.0;

    static uint32_t lockPosition() {
#if defined(TEENSYDUINO)
        const uint32_t mask = __get_primask();
#else
        const uint32_t mask = __get_PRIMASK();
#endif
        noInterrupts();
        return mask;
    }

    static void unlockPosition(uint32_t mask) {
        // Preserve the caller's interrupt state, including when read from an ISR.
        if (mask == 0) {
            interrupts();
        }
    }

    static bool parseDecimal(const char* field, double& out) {
        // NMEA decimal fields have no whitespace, exponent or hex notation.
        const char* cursor = field;
        if (*cursor == '-' || *cursor == '+') ++cursor;
        const char* digits = cursor;
        while (*cursor >= '0' && *cursor <= '9') ++cursor;
        if (cursor == digits) return false;
        if (*cursor == '.') {
            digits = ++cursor;
            while (*cursor >= '0' && *cursor <= '9') ++cursor;
            if (cursor == digits) return false;
        }
        if (*cursor != '\0') return false;

        errno = 0;
        char* end = nullptr;
        const double value = std::strtod(field, &end);
        if (*end != '\0' || errno == ERANGE || !std::isfinite(value)) return false;
        out = value;
        return true;
    }

    static bool parseByte(const char* field, uint8_t& out) {
        if (*field == '\0') return false;
        unsigned value = 0;
        for (; *field != '\0'; ++field) {
            if (*field < '0' || *field > '9') return false;
            value = value * 10 + static_cast<unsigned>(*field - '0');
            if (value > UINT8_MAX) return false;
        }
        out = static_cast<uint8_t>(value);
        return true;
    }

    static bool nmeaDegrees(const char* field, unsigned degreeDigits, double limit, double& out) {
        unsigned digitCount = 0;
        while (field[digitCount] >= '0' && field[digitCount] <= '9') ++digitCount;
        if (digitCount != degreeDigits + 2) return false;

        double value;
        if (!parseDecimal(field, value)) return false;
        const double degrees = std::floor(value / 100.0);
        const double minutes = value - degrees * 100.0;
        if (minutes >= 60.0 || degrees > limit || (degrees == limit && minutes != 0.0)) {
            return false;
        }
        out = degrees + minutes / 60.0;
        return true;
    }

    static void onPosition(NmeaPacket& packet) {
        if (!packet.ChecksumValid() || packet.FieldCount() < 15) {
            return;
        }

        Position next;
        uint8_t quality;
        uint8_t satellites;
        double latitude, longitude, height, geoid;
        // Read one GGA epoch directly; ellipsoid height includes geoid separation.
        if (parseByte(packet[6].c_str(), quality) && quality >= 1 && quality <= 5 &&
            parseByte(packet[7].c_str(), satellites) &&
            nmeaDegrees(packet[2].c_str(), 2, 90.0, latitude) &&
            nmeaDegrees(packet[4].c_str(), 3, 180.0, longitude) &&
            (packet[3] == "N" || packet[3] == "S") &&
            (packet[5] == "E" || packet[5] == "W") &&
            parseDecimal(packet[9].c_str(), height) &&
            parseDecimal(packet[11].c_str(), geoid) &&
            packet[10] == "M" && packet[12] == "M" &&
            std::isfinite(height + geoid)) {
            next.fix = static_cast<FixQuality>(quality);
            next.satellites = satellites;
            next.latitude = packet[3] == "S" ? -latitude : latitude;
            next.longitude = packet[5] == "W" ? -longitude : longitude;
            next.altitudeMeters = height + geoid;
        }

        if (next.fix != FixQuality::NO_FIX && baseConfigured) {
            double x, y, z;
            LG290P::geodeticToEcef(next.latitude, next.longitude, next.altitudeMeters, x, y, z);
            const double dx = x - baseX;
            const double dy = y - baseY;
            const double dz = z - baseZ;
            // Rotate the ECEF difference into the base antenna's local frame.
            next.northMeters = -baseSinLat * baseCosLon * dx -
                               baseSinLat * baseSinLon * dy + baseCosLat * dz;
            next.eastMeters = -baseSinLon * dx + baseCosLon * dy;
            next.upMeters = baseCosLat * baseCosLon * dx +
                            baseCosLat * baseSinLon * dy + baseSinLat * dz;
            next.relativeValid = true;
        }

        const uint32_t receivedMs = millis();
        // Publish the whole report together so the control ISR cannot read a
        // double or a position whose fields are only partly updated.
        const uint32_t mask = lockPosition();
        latestPosition = next;
        lastPositionMs = receivedMs;
        reportReceived = true;
        unlockPosition(mask);
    }

    bool configure(HardwareSerial& serial, uint8_t uart, uint32_t baud) {
        if (state != SetupStates::DISABLED || uart < 1 || uart > 3 ||
            baud < 9600 || baud > 3000000) {
            return false;
        }
        gpsSerial = &serial;
        receiverUart = uart;
        gpsBaud = baud;
        state = SetupStates::CONNECT;
        return true;
    }

    bool setup() {
        if (setupComplete()) return true;
        if (state == SetupStates::FAILED) return false;

        bool success = false;
        switch (state) {
            case SetupStates::CONNECT:
                gpsSerial->begin(gpsBaud);
                success = receiver.begin(*gpsSerial, "Drone GPS");
                if (success) state = SetupStates::CONFIGURE;
                break;

            case SetupStates::CONFIGURE:
                // Save and restart once after setting options that require it.
                success = receiver.ensureModeRover() &&
                          receiver.setFixInterval(FIX_INTERVAL_MS, false) &&
                          receiver.setNavMode(FLIGHT_NAV_MODE, false) &&
                          receiver.setPortInputProtocols(receiverUart,
                              LG290P_COM_TYPE_NMEA | LG290P_COM_TYPE_RTCM3) &&
                          receiver.setPortOutputProtocols(receiverUart, LG290P_COM_TYPE_NMEA) &&
                          receiver.setMessageRate("GGA", 1) &&
                          receiver.setMessageRate("PQTMPVT", 0, 1);
                if (success) state = SetupStates::STREAM;
                break;

            case SetupStates::STREAM: {
                // Only GGA is needed for position, RTK quality and satellite count.
                static const char* const unusedMessages[] = {
                    "RMC", "GSV", "GSA", "VTG", "GLL", "ZDA", "GST"
                };
                static size_t messageIndex = 0;
                success = receiver.setMessageRate(unusedMessages[messageIndex], 0);
                if (success && ++messageIndex == sizeof(unusedMessages) / sizeof(unusedMessages[0])) {
                    state = SetupStates::SAVE;
                }
                break;
            }

            case SetupStates::SAVE:
                success = receiver.save();
                if (success) state = SetupStates::RESTART;
                break;

            case SetupStates::RESTART:
                success = receiver.reset() && receiver.nmeaSubscribe("GGA", onPosition);
                if (success) state = SetupStates::COMPLETE;
                break;

            default:
                break;
        }

        if (!success) {
            USB::sendText("DRONE: GPS setup failed. Check wiring, baud rate and receiver firmware");
            state = SetupStates::FAILED;
        }
        return success;
    }

    bool setupComplete() {
        return state == SetupStates::DISABLED || state == SetupStates::COMPLETE;
    }

    void update() {
        if (state != SetupStates::COMPLETE || gpsSerial == nullptr) return;

        // The byte overload bounds the work; the library's no-argument update()
        // drains the UART without a limit.
        for (int count = 0; count < MAX_BYTES_PER_UPDATE && gpsSerial->available() > 0; ++count) {
            const int incoming = gpsSerial->read();
            if (incoming < 0) break;
            receiver.update(static_cast<uint8_t>(incoming));
        }
    }

    bool getPosition(Position& out) {
        const uint32_t mask = lockPosition();
        out = latestPosition;
        const uint32_t receivedMs = lastPositionMs;
        const bool received = reportReceived;
        unlockPosition(mask);

        out.ageMs = received ? static_cast<uint32_t>(millis() - receivedMs) : UINT32_MAX;
        const bool valid = received && out.fix != FixQuality::NO_FIX &&
                           out.ageMs <= MAX_POSITION_AGE_MS;
        if (!valid) out.relativeValid = false;
        return valid;
    }

    bool setBasePosition(double latitude, double longitude, double altitudeMeters) {
        if (!std::isfinite(latitude) || !std::isfinite(longitude) ||
            !std::isfinite(altitudeMeters) ||
            std::fabs(latitude) > 90.0 || std::fabs(longitude) > 180.0) {
            return false;
        }

        LG290P::geodeticToEcef(latitude, longitude, altitudeMeters, baseX, baseY, baseZ);
        baseSinLat = std::sin(latitude * DEG_TO_RAD);
        baseCosLat = std::cos(latitude * DEG_TO_RAD);
        baseSinLon = std::sin(longitude * DEG_TO_RAD);
        baseCosLon = std::cos(longitude * DEG_TO_RAD);
        baseConfigured = true;

        const uint32_t mask = lockPosition();
        latestPosition.relativeValid = false;
        unlockPosition(mask);
        return true;
    }

    size_t sendCorrections(const uint8_t* data, size_t length) {
        if (state != SetupStates::COMPLETE || gpsSerial == nullptr || data == nullptr || length == 0) return 0;

        const int available = gpsSerial->availableForWrite();
        if (available <= 0) return 0;
        const size_t capacity = static_cast<size_t>(available);
        const size_t count = length < capacity ? length : capacity;
        return gpsSerial->write(data, count);
    }

} // namespace GPS
