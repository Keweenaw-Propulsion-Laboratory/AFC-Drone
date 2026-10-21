#pragma once

#include <cstddef>
#include <cstdint>

#include "drone.h"

/**
 * Flight Data Logger
 *
 * Records binary flight data to the Teensy 4.1's built-in SD card, from boot
 * until power-off. Producers - including the 1 kHz control ISR - copy records
 * into a RAM ring buffer; loop() drains the ring to the card in whole 512-byte
 * sectors, and only while the card is not busy. An SD card can stall for
 * hundreds of milliseconds mid-write, and the ring is what absorbs that.
 *
 * Built to lose as little as possible when things go wrong:
 *
 * - Every record carries a sequence number and a CRC-32. The decoder detects
 *   gaps exactly, and resyncs past a torn or corrupt sector.
 * - A write or sync error does not end the log, or even the file. The writer
 *   closes the file, backs off, reopens it and carries on at its end while
 *   the ring keeps buffering. Only if that keeps failing does it move on to a
 *   new segment file (LOG008.BIN after LOG007.BIN) of the same session.
 *   Remounting the card blocks for an unknown time, so it is only attempted
 *   outside the armed/flight states.
 * - The file is synced every 250 ms, and on the next boot setup() scans the
 *   preallocated space past the last sync and recovers any records a power cut
 *   left there. In practice a power cut costs what was still in RAM.
 * - Pulling the card out is noticed within about 0.3 s, without blocking.
 *   Records keep buffering in RAM - about 2.5 s worth before new ones are
 *   dropped - and the same card reinserted on the ground continues the same
 *   file. A different card gets a new segment file of the same session.
 * - A full card does not end the log. When free space runs low the oldest
 *   LOGnnn.BIN is deleted, and file names wrap from LOG999 back to LOG000.
 *
 * The dashboard can start and stop the log, and drop named marks into it, over
 * USB or the radio; see command(). State changes and other events are noted in
 * the log as TEXT records alongside the telemetry.
 *
 * Files are LOG000.BIN - LOG999.BIN in the card root, and decode with
 * scripts/decode_log.py. Format the card exFAT with the SD Association
 * formatter; FAT32 works but stalls for longer.
 *
 * On the nucleo_l152re bench target every function is a harmless no-op.
 */
namespace Logger {

// MARK: File format
//
// These structs are the wire format scripts/decode_log.py parses. Changing any
// of them, or Drone::Telemetry_t, means bumping FORMAT_VERSION and updating the
// decoder in the same commit. The static_asserts in logger.cpp catch a silent
// layout change.

static constexpr uint32_t FILE_MAGIC     = 0x4C434641; // "AFCL" little-endian
static constexpr uint16_t FORMAT_VERSION = 1;
static constexpr uint32_t HEADER_BYTES   = 512; // FileHeader is padded to one sector
static constexpr uint8_t  RECORD_SYNC    = 0xA5;

enum class RecordType : uint8_t {
    TELEMETRY = 1, // Payload is a raw Drone::Telemetry_t
    STATS     = 2, // Payload is a StatsRecord, once a second
    TEXT      = 3, // Payload is up to 64 bytes of ASCII, not NUL-terminated
    END       = 4, // No payload. Written by stop(); a file ending without it was cut off
};

/** First bytes of every log file. Zero-padded to HEADER_BYTES. */
struct __attribute__((packed)) FileHeader {
    uint32_t magic;             // FILE_MAGIC
    uint16_t version;           // FORMAT_VERSION
    uint16_t headerBytes;       // HEADER_BYTES; the first record starts here
    uint32_t sessionId;         // Shared by every segment of one boot; seeds every record CRC
    uint16_t segment;           // 0 for the first file of a session, then 1, 2, ...
    uint16_t recordHeaderBytes; // sizeof(RecordHeader)
    uint16_t telemetryBytes;    // sizeof(Drone::Telemetry_t)
    uint16_t reserved;
    uint32_t preallocSectors;   // Sectors reserved for the file; 0 if preallocation failed
};

/**
 * Precedes every record's payload.
 *
 * The CRC is zlib-compatible CRC-32 over this header up to the crc field,
 * then the payload, starting from the file's sessionId - in Python,
 * zlib.crc32(header[:12] + payload, sessionId). Seeding it with the session
 * means a stale record left in reused card space by an earlier boot fails the
 * check rather than being mistaken for current data.
 */
struct __attribute__((packed)) RecordHeader {
    uint8_t  sync;   // RECORD_SYNC
    uint8_t  type;   // RecordType
    uint16_t length; // Payload bytes following this header
    uint32_t seq;    // Per-session, +1 per record, INCLUDING dropped ones
    uint32_t timeUs; // micros() when the record was made; wraps every ~71 min
    uint32_t crc;
};

/** Logger health, recorded once a second. */
struct __attribute__((packed)) StatsRecord {
    uint32_t dropped;        // Records refused because the ring was full, since boot
    uint32_t highWaterBytes; // Most bytes ever waiting in the ring, since boot
    uint32_t maxStallUs;     // Longest wait on a busy card with data queued, this interval
    uint32_t maxSyncUs;      // Longest file.sync(), this interval
    uint16_t writeErrors;    // Write/sync failures since boot
    uint16_t segment;        // Segment being written
};

// MARK: Status
//
// State and Result values go on the wire unchanged in the USB LOG_STATUS
// message (see usb.cpp), so existing values must not be renumbered.

enum class State : uint8_t {
    NO_CARD     = 0, // No card mounted; update() watches the slot for one
    IDLE        = 1, // Mounted, no file open (after stop())
    LOGGING     = 2, // Normal operation
    REOPEN_WAIT = 3, // A write failed; buffering while waiting to retry
    CARD_OUT    = 4, // The card was pulled mid-log; buffering until it is back
};

/** Outcome of start() and stop(). */
enum class Result : uint8_t {
    OK          = 0,
    IN_FLIGHT   = 1, // Refused: it would block the loop while armed or flying
    NO_CARD     = 2, // No card mounted
    WRONG_STATE = 3, // Already logging (start) or already stopped (stop)
    CARD_ERROR  = 4, // The card failed the file operation
    UNKNOWN_ACTION = 5, // command() was given an Action it does not know
};

/** Operator commands. Wire values, as Result. */
enum class Action : uint8_t {
    START = 1,
    STOP  = 2,
    MARK  = 3, // Write a named marker into the log; works in flight
};

/** Longest label a mark keeps. The radio can carry only 7 characters of one. */
static constexpr size_t MAX_MARK_LABEL = 48;

/** A snapshot of the logger for the dashboard. See getStatus(). */
struct Status {
    State state;
    bool cardUnmountable;  // A card is in the slot but would not mount; format it
    bool cardRemoved;      // The card was pulled out while mounted; reinsert it
    bool slowFilesystem;   // The mounted card is not exFAT
    bool fileOpen;         // A log file is open now
    bool haveFile;         // fileIndex/fileBytes describe the open or last-closed file
    uint16_t fileIndex;    // n of LOGnnn.BIN
    uint32_t fileBytes;    // Written to the card so far, header sector included
    uint64_t cardBytes;    // Card capacity; 0 with no card
    uint64_t freeBytes;    // Space a log can still grow into; 0 with no card
    uint32_t bytesPerSec;  // Log data accepted over the last second; 0 when stopped
    uint32_t secondsRemaining; // freeBytes at the last measured (else nominal) rate
    uint32_t dropped;      // As getDropped()
    uint32_t writeErrors;  // As getWriteErrors()
    uint32_t highWaterBytes; // As getHighWater()
    uint32_t bufferBytes;  // Ring size, to put highWaterBytes in proportion
    uint8_t commandCount;  // +1 per command(); wraps
    Result commandResult;  // Outcome of the latest command()
    uint16_t marks;        // Marks written since boot
    uint16_t filesRecycled; // Old logs deleted to make room, since boot
};

// MARK: Interface

/**
 * Mount the card, recover the tail of the previous log if a power cut left
 * one, and start logging to a new file.
 *
 * Call once during BOOT. A missing or unreadable card is not a vehicle fault:
 * this reports it over USB and returns true. update() then watches the slot
 * and mounts a card inserted later, on the ground; a card that fails to mount
 * is retried only after it is reinserted.
 *
 * @return Always true. Whether a card is in use is setupComplete().
 * @warning Blocks while the card initializes, the previous file is scanned,
 *          and the new file is preallocated - typically tens of milliseconds.
 *          Call it before the control timer starts.
 */
bool setup();

/** @return True while a card is mounted, whether or not a file is open. */
bool setupComplete();

/**
 * Move queued data to the card. Call every pass from loop().
 *
 * Writes at most four sectors (2 KB) per call, and never waits on a busy card.
 * Also syncs the file every 250 ms, records a StatsRecord once a second, and
 * runs error recovery.
 *
 * @warning file.sync() blocks for its duration - normally a few milliseconds.
 *          Watch maxSyncUs in the stats. Opening a new segment after an error
 *          also blocks, for up to tens of milliseconds.
 */
void update();

/**
 * Begin logging to a new file after stop(). setup() calls this itself, so it
 * is only needed to resume.
 *
 * Refused while armed or flying: creating and preallocating the file blocks
 * for tens of milliseconds.
 */
Result start();

/**
 * Write an END record, flush everything queued, release the unused
 * preallocation, and close the file. Use before a planned power-off.
 *
 * Refused while armed or flying, because it blocks until the flush completes,
 * which includes any card stall. Stopping during error recovery discards
 * what is queued, since there is no file to flush it into.
 */
Result stop();

/**
 * Write "MARK n: label" into the log as a TEXT record. Cheap and safe in
 * flight, unlike start() and stop().
 *
 * @param length Label bytes; anything past MAX_MARK_LABEL is dropped.
 */
Result mark(const char* label, size_t length);

/**
 * Carry out an operator command from the dashboard, and record it for
 * getStatus() so the dashboard can see which command a result belongs to.
 *
 * @param label Mark label for Action::MARK, otherwise ignored.
 */
Result command(Action action, const char* label, size_t labelLength);

/** @return True while records are being accepted and written. */
bool isLogging();

/**
 * Queue one telemetry snapshot.
 *
 * Safe to call from the control ISR: this copies about 100 bytes and computes
 * a CRC with interrupts briefly masked, roughly 1-2 us. No SD access.
 */
void record(const Drone::Telemetry_t& t);

/**
 * Queue a short text event - a state change, a fault, an operator command.
 * Truncated to 64 characters. Safe from any context.
 */
void note(const char* text);

/** @return Records refused because the ring was full, since boot. */
uint32_t getDropped();

/**
 * @return The most bytes that have ever waited in the ring. If this gets
 *         near the ring size (256 KB), the ring needs to be bigger.
 */
uint32_t getHighWater();

/** @return Write and sync failures since boot. */
uint32_t getWriteErrors();

/**
 * Fill out a status snapshot. Cheap: card space is measured once per mount
 * and tracked from there, so this does no card I/O. Call from loop() context.
 */
void getStatus(Status& out);

} // namespace Logger
