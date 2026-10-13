#include "GPS.h"

#include "usb.h"

#include <Arduino.h>
// The upstream NmeaPacket header compares an int index with size_t.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <SparkFun_LG290P_GNSS.h>
#pragma GCC diagnostic pop
#include <cmath>
#include <cstdlib>

#if defined(TEENSYDUINO)
#include <util/atomic.h>
#endif

namespace GPS {

    // TODO: Teensy UART/pins are not finalized. Set to &Serial1, &Serial2, etc.
    // Reserve that UART for GPS; connect Teensy RX to GNSS TX and TX to RX.
    static HardwareSerial* gpsSerial = nullptr;
    // GNSS-side UART number (1, 2 or 3), independent of the Teensy Serial number.
    static constexpr int LG290P_UART = 1; // TODO: Confirm the breakout connector.
    static constexpr uint32_t GPS_BAUD = 460800; // LG290P default baud rate.
    static constexpr uint16_t FIX_INTERVAL_MS = 50; // 20 Hz rover measurements.
    static constexpr uint32_t MAX_POSITION_AGE_MS = 500;
    static constexpr int MAX_BYTES_PER_UPDATE = 512;
    static constexpr uint16_t FLIGHT_NAV_MODE = 5; // Quectel dynamic flight mode.

    enum class SetupStates : uint8_t {
        CONNECT,
        CONFIGURE,
        STREAM,
        SAVE,
        RESTART,
        COMPLETE,
        FAILED,
    };

    static LG290P receiver;
    static SetupStates state = SetupStates::CONNECT;
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

    static double nmeaDegrees(const char* field) {
        const double value = std::strtod(field, nullptr);
        const double degrees = std::floor(value / 100.0);
        return degrees + (value - degrees * 100.0) / 60.0;
    }

    static void onPosition(NmeaPacket& packet) {
        if (!packet.ChecksumValid() || packet.FieldCount() < 15) {
            return;
        }

        Position next;
        const int quality = std::atoi(packet[6].c_str());
        if (quality >= 1 && quality <= 5) {
            next.fix = static_cast<FixQuality>(quality);
            next.satellites = static_cast<uint8_t>(std::atoi(packet[7].c_str()));

            // Read one GGA epoch directly. Library getters can enable additional
            // reports, and GGA sea-level height needs its geoid separation added.
            next.latitude = nmeaDegrees(packet[2].c_str());
            next.longitude = nmeaDegrees(packet[4].c_str());
            if (packet[3] == "S") next.latitude = -next.latitude;
            if (packet[5] == "W") next.longitude = -next.longitude;
            next.altitudeMeters = std::strtod(packet[9].c_str(), nullptr) +
                                  std::strtod(packet[11].c_str(), nullptr);

            if (packet[2].empty() || packet[4].empty() ||
                packet[9].empty() || packet[11].empty() ||
                (packet[3] != "N" && packet[3] != "S") ||
                (packet[5] != "E" && packet[5] != "W") ||
                !std::isfinite(next.latitude) || !std::isfinite(next.longitude) ||
                !std::isfinite(next.altitudeMeters) ||
                std::fabs(next.latitude) > 90.0 || std::fabs(next.longitude) > 180.0) {
                next.fix = FixQuality::NO_FIX;
            }
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

    bool setup() {
        if (state == SetupStates::COMPLETE) return true;
        if (state == SetupStates::FAILED) return false;
        if (gpsSerial == nullptr) {
            USB::sendText("DRONE: GPS UART not configured. Set gpsSerial in GPS.cpp");
            state = SetupStates::FAILED;
            return false;
        }

        bool success = false;
        switch (state) {
            case SetupStates::CONNECT:
                gpsSerial->begin(GPS_BAUD);
                success = receiver.begin(*gpsSerial, "Drone GPS");
                if (success) state = SetupStates::CONFIGURE;
                break;

            case SetupStates::CONFIGURE:
                // Save and restart once after setting options that require it.
                success = receiver.ensureModeRover() &&
                          receiver.setFixInterval(FIX_INTERVAL_MS, false) &&
                          receiver.setNavMode(FLIGHT_NAV_MODE, false) &&
                          receiver.setPortInputProtocols(LG290P_UART,
                              LG290P_COM_TYPE_NMEA | LG290P_COM_TYPE_RTCM3) &&
                          receiver.setPortOutputProtocols(LG290P_UART, LG290P_COM_TYPE_NMEA) &&
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

    bool setupComplete() {return state == SetupStates::COMPLETE;}

    void update() {
        if (!setupComplete()) return;

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
        if (!setupComplete() || data == nullptr || length == 0) return 0;

        const int available = gpsSerial->availableForWrite();
        if (available <= 0) return 0;
        const size_t capacity = static_cast<size_t>(available);
        const size_t count = length < capacity ? length : capacity;
        return gpsSerial->write(data, count);
    }

} // namespace GPS
