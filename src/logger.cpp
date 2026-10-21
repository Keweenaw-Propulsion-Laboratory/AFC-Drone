#include "logger.h"

#include <Arduino.h>
#include <cstddef>
#include <cstring>
#include <new>

#include "usb.h"

// MARK: Wire format checks

static_assert(sizeof(Logger::FileHeader) == 24, "Log file header size changed");
static_assert(sizeof(Logger::FileHeader) <= Logger::HEADER_BYTES, "Log file header overflows its sector");
static_assert(sizeof(Logger::RecordHeader) == 16, "Log record header size changed");
static_assert(offsetof(Logger::RecordHeader, crc) == 12, "Log record CRC must be the last header field");
static_assert(sizeof(Logger::StatsRecord) == 20, "Log stats record size changed");

// Telemetry_t is logged as raw memory, padding included, because the control
// ISR cannot afford to repack it field by field. scripts/decode_log.py mirrors
// this exact layout; these asserts make a struct edit a build failure rather
// than a log that decodes into plausible garbage.
static_assert(sizeof(Drone::Telemetry_t) == 88, "Telemetry_t changed: bump FORMAT_VERSION, update decode_log.py");
static_assert(offsetof(Drone::Telemetry_t, state) == 16, "Telemetry_t layout changed");
static_assert(offsetof(Drone::Telemetry_t, gimbalPitch) == 20, "Telemetry_t layout changed");
static_assert(offsetof(Drone::Telemetry_t, qR) == 36, "Telemetry_t layout changed");
static_assert(offsetof(Drone::Telemetry_t, posZ) == 84, "Telemetry_t layout changed");

// SdFat ships with Teensyduino but not with the STM32 core, and the Nucleo has
// no card slot. The bench build gets the no-op stubs at the bottom.
#if defined(TEENSYDUINO)

#include <SdFat.h>
#include "SdCard/SdioTeensy.h" // USDHC1 register names, for driving the card directly

namespace Logger {

// Timing traces for hardware tests, on the debug text channel. LOGGER_TEST_HOOKS
// has no build environment of its own; for a bench build that has it, run
//   PLATFORMIO_BUILD_FLAGS=-DLOGGER_TEST_HOOKS pio run -e teensy41 -t upload
// and never fly that firmware.
#if defined(LOGGER_TEST_HOOKS)
#define LOGGER_TRACE(...) do { char traceText[96]; snprintf(traceText, sizeof(traceText), __VA_ARGS__); USB::sendText(traceText); } while (0)
#else
// Still type-checks the arguments, so timing variables kept for the trace do
// not trip -Wunused; the dead branch compiles to nothing.
#define LOGGER_TRACE(...) do { if (false) { char traceText[1]; snprintf(traceText, 1, __VA_ARGS__); } } while (0)
#endif

// MARK: Tuning

// SD cards are written a sector at a time. Writes of whole sectors at aligned
// file offsets go straight to the card; anything else makes SdFat
// read-modify-write through its cache, which is slower and stalls more.
static constexpr uint32_t SECTOR_BYTES = 512;
static_assert(HEADER_BYTES == SECTOR_BYTES, "Records must start sector-aligned");

// RAM between the producers and the card. At 1 kHz * 104 B telemetry records,
// 256 KB rides out about 2.5 s of the card not accepting writes - a long stall,
// or an error and the reopen after it. Must be a power of two (for masking) and
// whole sectors (so a sector never wraps the end of the buffer).
static constexpr uint32_t BUFFER_BYTES = 256 * 1024;
static constexpr uint32_t BUFFER_MASK = BUFFER_BYTES - 1;
static_assert((BUFFER_BYTES & BUFFER_MASK) == 0, "BUFFER_BYTES must be a power of two");
static_assert(BUFFER_BYTES % SECTOR_BYTES == 0, "BUFFER_BYTES must be whole sectors");

// Reserved up front so the card never allocates clusters or rewrites the FAT
// mid-flight: 256 MB is ~40 min at 104 KB/s. A session that outgrows it keeps
// writing, just with the allocation stalls preallocation avoids. Also bounds
// how far the boot-time recovery scan can look.
static constexpr uint32_t PREALLOCATE_SECTORS = (256UL * 1024 * 1024) / SECTOR_BYTES;

// Cap on sectors per loop() pass, so draining a backlog after a stall cannot
// starve radio/USB servicing. 4 sectors is 2 KB, a few tens of microseconds.
static constexpr int MAX_SECTORS_PER_PASS = 4;

// How often the file length is committed to the directory. This bounds what a
// power cut can leave past the recorded end, which recoverTail() then has to
// find. Each sync is a blocking write of its own; see maxSyncUs in the stats.
static constexpr uint32_t SYNC_INTERVAL_MS = 250;
static constexpr uint32_t STATS_INTERVAL_MS = 1000;

// After a write error the log is reopened and continued in the same file.
// If that keeps failing with nothing written in between - a bad patch of card,
// say - it moves on to a fresh file instead.
static constexpr uint8_t MAX_SAME_FILE_ATTEMPTS = 3;

// Backoff between attempts to continue after a write error.
static constexpr uint32_t RETRY_MIN_MS = 100;
static constexpr uint32_t RETRY_MAX_MS = 2000;

// The Teensy 4.1 slot has no card-detect switch, but an SD card holds DAT3 up
// through its internal pull-up until it is initialized. Released from the SDIO
// controller and given a pull-down, the pin reads high only with a card in -
// the same check PJRC's SD library uses for mediaPresent(). Reading it costs
// nothing, unlike sd.begin() on an empty slot, which blocks for about 2 s.
static constexpr uint8_t SD_DAT3_PIN = 46;
static constexpr uint32_t CARD_DETECT_INTERVAL_MS = 250;

// How often a mounted card may be asked whether it is still there. The
// question is a CMD13 status request: a few microseconds when the card
// answers, and an immediate command timeout when it has been pulled. Without
// it a pulled card just looks permanently busy.
//
// It must not be asked in the middle of normal writing. SdFat's command
// helper clears every pending controller flag, including the transfer-complete
// flag its FIFO write path is still waiting to consume; losing that flag makes
// the driver think the card is busy forever, and logging stops dead. So while
// logging, the card is only asked once it has been busy for longer than a
// healthy card's write stall - by then no transfer-complete flag is pending.
// While stopped there is no transfer in progress, so asking is always safe.
static constexpr uint32_t CARD_PROBE_INTERVAL_MS = 100;
static constexpr uint32_t STALL_PROBE_US = 250000;

// Bringing a card back after it lost contact. A card that lost power has to
// run its power-up initialisation again, which the SD spec allows up to 1 s
// for; SdFat's begin() waits that out in one blocking loop. Instead the card
// is walked through it here a step per loop pass (see serviceCardWake()), and
// SdFat's begin() is called only once the card reports ready, when it takes
// about 10 ms.
static constexpr uint32_t WAKE_CLOCK_KHZ = 400;       // Identification-mode clock limit
static constexpr uint32_t WAKE_POLL_INTERVAL_MS = 10;  // Between ACMD41 polls
static constexpr uint32_t WAKE_TIMEOUT_MS = 1500;      // Spec maximum is 1 s
static constexpr uint32_t WAKE_COMMAND_TIMEOUT_US = 2000;

// Window for the measured data rate in getStatus().
static constexpr uint32_t RATE_INTERVAL_MS = 1000;

// What one telemetry record per 1 kHz control tick costs. getStatus() uses it
// for the time-remaining estimate until a rate has actually been measured.
static constexpr uint32_t NOMINAL_BYTES_PER_SEC =
    (sizeof(RecordHeader) + sizeof(Drone::Telemetry_t)) * 1000;

// LOG000.BIN - LOG999.BIN. Names are used in a circle: after LOG999 comes
// LOG000 again, which by then is the oldest log on the card and is deleted.
static constexpr unsigned MAX_FILES = 1000;

// When the card's free space falls below this, the oldest log is deleted to
// make room. Two preallocations' worth: the next file's reservation, plus
// room for the open one to outgrow its own.
static constexpr uint64_t FREE_LOW_WATER_BYTES =
    2ULL * PREALLOCATE_SECTORS * SECTOR_BYTES;
static constexpr uint32_t RECLAIM_INTERVAL_MS = 1000;
static constexpr uint16_t MAX_PAYLOAD_BYTES = 128;
static constexpr uint16_t MAX_TEXT_BYTES = 64;
static_assert(sizeof(Drone::Telemetry_t) <= MAX_PAYLOAD_BYTES, "Telemetry record too big");

// MARK: CRC-32

/**
 * Table for the reflected CRC-32 (polynomial 0xEDB88320) that zlib and
 * Python's zlib.crc32() compute.
 *
 * Built at compile time but deliberately not const: a const table is placed
 * in flash, and flash cache misses inside the control ISR cost far more than
 * the 1 KB of tightly coupled RAM this occupies instead.
 */
struct Crc32Table {
    uint32_t entry[256];
    constexpr Crc32Table() : entry() {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int bit = 0; bit < 8; bit++) {
                c = (c & 1) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
            }
            entry[i] = c;
        }
    }
};
static Crc32Table crcTable;

/**
 * Fold bytes into a running CRC. Start from ~seed and invert the result, which
 * makes the whole computation equal to zlib.crc32(data, seed).
 */
static uint32_t crcUpdate(uint32_t crc, const void* data, uint32_t length) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (uint32_t i = 0; i < length; i++) {
        crc = crcTable.entry[(crc ^ bytes[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc;
}

static uint32_t recordCrc(uint32_t session, const void* header, const void* payload, uint16_t length) {
    uint32_t crc = crcUpdate(~session, header, offsetof(RecordHeader, crc));
    return ~crcUpdate(crc, payload, length);
}

// MARK: State

static SdFs sd;
static FsFile file;
static State writer = State::NO_CARD;

// Producers only enqueue while this is set. Cleared by stop(), and stays set
// through REOPEN_WAIT so an error does not lose what arrives during recovery.
static volatile bool accepting = false;

// RAM2 (DMAMEM) is 512 KB and otherwise mostly unused, which keeps this out of
// the tightly coupled RAM the control loop runs from. The card is fed by the
// CPU through the SDIO FIFO, not DMA, so cache coherence is not a concern.
DMAMEM static uint8_t ring[BUFFER_BYTES];

// Free-running byte counters. head - tail is the number of bytes queued, which
// stays correct across the 32-bit wraparound because it is unsigned. head and
// nextSeq change only inside push()'s critical section; tail only in loop()
// context. The writer moves tail a whole sector at a time, so
// (tail & BUFFER_MASK) is always sector-aligned and every sector is contiguous
// in ring[].
static volatile uint32_t head = 0;
static volatile uint32_t tail = 0;
static volatile uint32_t nextSeq = 0;
static volatile uint32_t dropped = 0;

static uint32_t sessionId = 0;
static uint16_t currentSegment = 0;
static uint16_t nextSegment = 0;
static unsigned nextFileIndex = 0;
static char fileName[24] = "";

// Health, reported in StatsRecord
static uint32_t highWater = 0;
static uint32_t intervalMaxStallUs = 0;
static uint32_t intervalMaxSyncUs = 0;
static uint16_t writeErrors = 0;
static bool stalled = false;
static uint32_t stallStartUs = 0;

static uint32_t lastSyncMs = 0;
static uint32_t lastStatsMs = 0;
static uint32_t retryStartMs = 0;
static uint32_t retryDelayMs = RETRY_MIN_MS;
static uint8_t sameFileAttempts = 0;
static uint32_t lastDetectMs = 0;

// Consecutive card-detect reads that saw a card, saturating at 2. A mount is
// tried once, on the step to 2, so the contacts of a card still being pushed
// in have one interval to settle. A card that then fails to mount is left
// alone until it is pulled and reinserted.
static uint8_t presentReads = 0;

// Card space, for getStatus(). Counting free clusters means reading the whole
// FAT or allocation bitmap - far too slow to repeat - so it is measured once
// per mount and then tracked from what this module itself allocates.
static uint32_t clusterBytes = 0;
static uint64_t cardBytes = 0;
static uint64_t freeAtMount = 0;    // Free space once the previous log was repaired
static uint64_t usedSinceMount = 0; // Allocation of every file closed since then
static bool slowFilesystem = false; // Mounted card is not exFAT
static bool cardUnmountable = false; // A card is in the slot but will not mount
static bool cardRemoved = false; // The mounted card was pulled; cleared by the next mount
static uint32_t lastProbeMs = 0;

// Set whenever the card's multi-sector write has just ended (a sync, a file
// opened): the card is then asked whether it is still there before the next
// write. See drain().
static bool probeBeforeWrite = false;

// The ID of the mounted card, to tell whether a card coming back is the same
// one - and so whether the file and space bookkeeping still describe it.
static cid_t cardCid{};
static bool haveCardCid = false;

// Waking a card that came back while logging; see serviceCardWake()
enum class Wake : uint8_t {
    OFF,     // Not waking a card
    CMD0,    // Next: reset it to idle
    CMD8,    // Next: check it is a v2 card and agrees on voltage
    ACMD41,  // Polling until it finishes power-up
};
static Wake wake = Wake::OFF;
static uint32_t wakeStartMs = 0;
static uint32_t lastWakeStepMs = 0;
static uint8_t wakeCmd8Tries = 0;
// A card still settling after an aborted write can miss the first CMD8, as
// SdFat's own begin() allows for; it is reset and asked again.
static constexpr uint8_t WAKE_CMD8_TRIES = 3;
// A different card came back in flight. It is mounted, but rescanning and
// re-measuring it would block, so it is taken into use on the ground.
static bool newCardWaiting = false;

// The open file, or the last one closed while haveFile stays set.
static bool fileOpen = false;
static bool haveFile = false;
static uint16_t fileIndex = 0;
static uint32_t fileBytes = 0;     // Header plus every record written to the card
static uint32_t fileReserved = 0;  // Preallocated bytes

// Which LOGnnn.BIN names exist on the card, one bit per index. Built when the
// card mounts and kept current from there, so finding the oldest log never
// needs a directory scan.
static uint8_t logPresent[(MAX_FILES + 7) / 8];
static unsigned newestIndex = 0; // Most recently created log; valid while any exist
static uint16_t filesRecycled = 0;
static uint32_t lastReclaimMs = 0;

// Operator commands from the dashboard, echoed in the status report
static uint8_t commandCount = 0;
static Result commandResult = Result::OK;
static uint16_t markCount = 0;

// Accepted log bytes per second, over the last RATE_INTERVAL_MS
static uint32_t rateSampleMs = 0;
static uint32_t rateSampleHead = 0;
static uint32_t bytesPerSec = 0;
static uint32_t lastActiveBytesPerSec = 0;

// Sector scratch for the file header and recovery. Only used from loop() and
// setup() context, so one shared buffer is enough.
static uint8_t scratch[2 * SECTOR_BYTES];

// MARK: Producer

/**
 * Copy one record into the ring, or count it as dropped.
 *
 * Safe from both the control ISR and loop(). The critical section is needed
 * because there is more than one producer: a loop()-context note() could be
 * preempted by the ISR halfway through advancing head. PRIMASK is saved and
 * restored rather than unconditionally re-enabled, so calling this from
 * inside the ISR does not turn interrupts back on early.
 */
static void push(RecordType type, const void* payload, uint16_t length) {
    RecordHeader header;
    header.sync = RECORD_SYNC;
    header.type = static_cast<uint8_t>(type);
    header.length = length;
    header.timeUs = micros();

    const uint32_t total = sizeof(header) + length;

    uint32_t primask;
    __asm__ volatile("mrs %0, primask" : "=r"(primask));
    __disable_irq();

    if (accepting) {
        // A dropped record still consumes a sequence number, so the decoded
        // log shows the loss as a gap at exactly the right place.
        header.seq = nextSeq;
        nextSeq = nextSeq + 1;

        if (BUFFER_BYTES - (head - tail) < total) {
            // Full: keep what is already queued so the log stays contiguous.
            dropped = dropped + 1;
        } else {
            header.crc = recordCrc(sessionId, &header, payload, length);

            uint32_t pos = head & BUFFER_MASK;
            auto copy = [&pos](const void* src, uint32_t n) {
                const uint32_t first = min(n, BUFFER_BYTES - pos);
                memcpy(&ring[pos], src, first);
                memcpy(&ring[0], static_cast<const uint8_t*>(src) + first, n - first);
                pos = (pos + n) & BUFFER_MASK;
            };
            copy(&header, sizeof(header));
            copy(payload, length);
            head = head + total;
        }
    }

    if ((primask & 1) == 0) __enable_irq();
}

// MARK: Card and files

/** True in the states where a long blocking call could trip the comm watchdog. */
static bool inFlight() {
    switch (Drone::getState()) {
    case Drone::States::READY_ARMED:
    case Drone::States::MAN_FLIGHT:
    case Drone::States::AUTO_FLIGHT:
        return true;
    default:
        return false;
    }
}

static bool mount() {
    // FIFO_SDIO is the fast mode for the Teensy 4.1's built-in slot: 4-bit
    // bus, CPU-fed FIFO. DMA_SDIO exists but is slower in practice.
    if (!sd.begin(SdioConfig(FIFO_SDIO))) return false;

    cardUnmountable = false;
    cardRemoved = false;
    slowFilesystem = sd.fatType() != FAT_TYPE_EXFAT;
    if (slowFilesystem) {
        USB::sendText("Logger: card is not exFAT; reformat for shorter write stalls");
    }
    clusterBytes = sd.bytesPerCluster();
    cardBytes = static_cast<uint64_t>(sd.clusterCount()) * clusterBytes;
    haveCardCid = sd.card()->readCID(&cardCid);
    probeBeforeWrite = false;
    return true;
}

/** Space a file of this many bytes occupies on the card. */
static uint64_t allocated(uint64_t bytes) {
    if (clusterBytes == 0) return bytes;
    return (bytes + clusterBytes - 1) / clusterBytes * clusterBytes;
}

/**
 * Take the free-space baseline the status report counts down from.
 *
 * Blocks while it reads the card's whole FAT or allocation bitmap: a fraction
 * of a second on a 32 GB card. Only called right after a mount, which only
 * happens on the ground.
 */
static void measureFreeSpace() {
    freeAtMount = static_cast<uint64_t>(sd.freeClusterCount()) * clusterBytes;
    usedSinceMount = 0;
}

/** The open file was closed, keeping `bytes` of card space. */
static void releaseFile(uint64_t bytes) {
    usedSinceMount += allocated(bytes);
    fileOpen = false;
    fileReserved = 0;
}

/**
 * Parse a log file name. Only names this module writes count, so a user's
 * LOG1.BIN or LOG0001.TXT is never mistaken for one.
 */
static bool parseLogName(const char* name, unsigned& index) {
    if (strncmp(name, "LOG", 3) != 0 || strcmp(name + 6, ".BIN") != 0) return false;
    index = 0;
    for (int i = 3; i < 6; i++) {
        if (name[i] < '0' || name[i] > '9') return false;
        index = index * 10 + static_cast<unsigned>(name[i] - '0');
    }
    return true;
}

static bool logExists(unsigned index) {
    return logPresent[index / 8] & (1u << (index % 8));
}

static void setLogExists(unsigned index, bool exists) {
    if (exists) logPresent[index / 8] |= static_cast<uint8_t>(1u << (index % 8));
    else logPresent[index / 8] &= static_cast<uint8_t>(~(1u << (index % 8)));
}

/**
 * Record which logs exist, and find the newest - to check it for a cut-off
 * tail and to number the next file after it. Walks the root directory once
 * rather than probing 1000 names with exists().
 *
 * Names wrap from LOG999 back to LOG000, so the highest number is not always
 * the newest. The logs on a card form a run of consecutive numbers (modulo
 * 1000) that ends at the newest file, and the largest stretch of unused
 * numbers is what lies ahead of it. The newest log is the one just before
 * that stretch.
 *
 * @return True if any log exists; newest is then its index.
 */
static bool scanLogs(unsigned& newest) {
    memset(logPresent, 0, sizeof(logPresent));

    FsFile root;
    FsFile entry;
    unsigned count = 0;
    unsigned highest = 0;
    if (!root.open("/")) return false;
    while (entry.openNext(&root, O_RDONLY)) {
        char name[24];
        unsigned index;
        if (entry.isFile() && entry.getName(name, sizeof(name)) > 0 &&
            parseLogName(name, index) && !logExists(index)) {
            setLogExists(index, true);
            if (count == 0 || index > highest) highest = index;
            count++;
        }
        entry.close();
    }
    root.close();

    if (count == 0) return false;
    newest = highest; // Also the answer when all 1000 names are taken
    if (count == MAX_FILES) return true;

    // Walk once around the circle from an existing log, measuring each run of
    // unused numbers and remembering the log before the longest.
    unsigned bestGap = 0;
    unsigned gap = 0;
    unsigned lastPresent = highest;
    for (unsigned step = 1; step <= MAX_FILES; step++) {
        const unsigned index = (highest + step) % MAX_FILES;
        if (!logExists(index)) {
            gap++;
            continue;
        }
        if (gap > bestGap) {
            bestGap = gap;
            newest = lastPresent;
        }
        gap = 0;
        lastPresent = index;
    }
    return true;
}

/** The oldest log other than the open one, or false if there is none. */
static bool findOldestLog(unsigned& oldest) {
    for (unsigned step = 1; step <= MAX_FILES; step++) {
        const unsigned index = (newestIndex + step) % MAX_FILES;
        if (!logExists(index)) continue;
        if (fileOpen && index == fileIndex) continue;
        oldest = index;
        return true;
    }
    return false;
}

/**
 * Delete one log to make room, and credit its space back.
 *
 * Blocks while the filesystem frees the file's clusters: a few milliseconds
 * on exFAT, which clears a run of bitmap bits, and up to tens of milliseconds
 * on FAT32, which rewrites the file's FAT chain.
 */
static bool deleteLog(unsigned index) {
    char name[24];
    snprintf(name, sizeof(name), "LOG%03u.BIN", index);

    FsFile old;
    uint64_t size = 0;
    if (old.open(name, O_RDONLY)) {
        size = old.fileSize();
        old.close();
    }
    if (!sd.remove(name)) return false;

    setLogExists(index, false);
    freeAtMount += allocated(size);
    filesRecycled++;

    char msg[MAX_TEXT_BYTES];
    snprintf(msg, sizeof(msg), "Logger: card full, deleted oldest log %s", name);
    USB::sendText(msg);
    note(msg);
    return true;
}

/** Card space not yet allocated to any file, the open one's reservation included. */
static uint64_t filesystemFree() {
    const uint64_t open = fileOpen ? max(allocated(fileBytes), allocated(fileReserved)) : 0;
    const uint64_t used = usedSinceMount + open;
    return freeAtMount > used ? freeAtMount - used : 0;
}

/**
 * Keep the card from filling: when free space runs low, delete the oldest log.
 * One file per RECLAIM_INTERVAL_MS at most, so a card that needs several
 * deletions spreads their blocking out rather than stacking it in one pass.
 *
 * Runs in flight as well. A deletion costs one blocking call every few tens
 * of minutes of logging, where refusing would end the log when the card fills.
 */
static void reclaimIfLow(uint32_t now) {
    if (now - lastReclaimMs < RECLAIM_INTERVAL_MS) return;
    lastReclaimMs = now;
    if (filesystemFree() >= FREE_LOW_WATER_BYTES) return;

    unsigned oldest;
    if (findOldestLog(oldest)) deleteLog(oldest);
}

/**
 * Create the next LOGnnn.BIN, preallocate it, and write its header sector.
 *
 * The header is written straight to the file as a full sector, not through the
 * ring, so it lands first even when this is a reopen with data still queued.
 * It is synced immediately so that the file and its preallocation are on the
 * card - recoverTail() needs both - before any record is.
 */
static bool openSegment() {
    const unsigned index = nextFileIndex % MAX_FILES;
    snprintf(fileName, sizeof(fileName), "LOG%03u.BIN", index);
    nextFileIndex = (index + 1) % MAX_FILES; // Skip this name next time, even if the rest fails

    // All 1000 names have been used and the circle is back to the oldest log.
    if (logExists(index) && !deleteLog(index)) return false;

    if (!file.open(fileName, O_RDWR | O_CREAT | O_EXCL)) return false;
    setLogExists(index, true);
    newestIndex = index;

    uint32_t preallocSectors = PREALLOCATE_SECTORS;
    if (!file.preAllocate(static_cast<uint64_t>(PREALLOCATE_SECTORS) * SECTOR_BYTES)) {
        // Not fatal - writes still work, they allocate as they go. Recovery
        // after a power cut is off for this file, since the space past its
        // end is not reserved for it.
        USB::sendText("Logger: preallocate failed, expect longer write stalls");
        preallocSectors = 0;
    }

    FileHeader fh{};
    fh.magic = FILE_MAGIC;
    fh.version = FORMAT_VERSION;
    fh.headerBytes = HEADER_BYTES;
    fh.sessionId = sessionId;
    fh.segment = nextSegment;
    fh.recordHeaderBytes = sizeof(RecordHeader);
    fh.telemetryBytes = sizeof(Drone::Telemetry_t);
    fh.preallocSectors = preallocSectors;

    memset(scratch, 0, SECTOR_BYTES);
    memcpy(scratch, &fh, sizeof(fh));
    if (file.write(scratch, SECTOR_BYTES) != SECTOR_BYTES || !file.sync()) {
        file.close();
        usedSinceMount += allocated(static_cast<uint64_t>(preallocSectors) * SECTOR_BYTES);
        return false;
    }

    fileOpen = true;
    haveFile = true;
    fileIndex = static_cast<uint16_t>(index);
    fileBytes = SECTOR_BYTES;
    fileReserved = preallocSectors * SECTOR_BYTES;

    currentSegment = nextSegment;
    nextSegment++;
    lastSyncMs = millis();
    probeBeforeWrite = true;
    return true;
}

/**
 * Count the records that start in the first sector of a two-sector window and
 * continue the sequence. The second sector is there for a record that starts
 * near the end of the first and runs over into it.
 *
 * Scans byte by byte for RECORD_SYNC and accepts a candidate only if its CRC
 * checks out against the session, so torn sectors, record fragments, and stale
 * data from earlier sessions are all skipped.
 */
static uint32_t scanSector(const uint8_t* window, uint32_t session,
                           bool& haveLast, uint32_t& lastSeq, bool& sawEnd) {
    uint32_t continuing = 0;
    uint32_t offset = 0;
    while (offset < SECTOR_BYTES) {
        RecordHeader rh;
        if (window[offset] != RECORD_SYNC) { offset++; continue; }
        memcpy(&rh, window + offset, sizeof(rh));
        if (rh.length > MAX_PAYLOAD_BYTES) { offset++; continue; }

        const uint32_t storedCrc = rh.crc;
        if (recordCrc(session, &rh, window + offset + sizeof(rh), rh.length) != storedCrc) {
            offset++;
            continue;
        }

        if (!haveLast || rh.seq > lastSeq) {
            continuing++;
            lastSeq = rh.seq;
            haveLast = true;
            if (rh.type == static_cast<uint8_t>(RecordType::END)) sawEnd = true;
        }
        offset += sizeof(rh) + rh.length;
    }
    return continuing;
}

/**
 * Extend a log over records that reached the card past its recorded end.
 *
 * Between syncs, sectors reach the card but the directory still records the
 * older, shorter length, so a normal read stops short of them. They are still
 * sitting in the file's preallocated (contiguous) space, though. This reads on
 * past the recorded end, sector by sector, and keeps going while each sector
 * holds records that continue the sequence. Each such sector is then written
 * back over itself through the file, which extends the recorded length to
 * cover it.
 *
 * Writes over identical data and stops at the first sector that does not
 * continue, so running it on a healthy file is safe.
 *
 * @param f    Open for writing and positioned at its recorded end, `size`.
 * @return     Sectors recovered. `f` is left positioned after the last one.
 */
static uint32_t extendTail(FsFile& f, const FileHeader& fh, uint64_t size) {
    const uint32_t firstSector = f.firstSector();
    const uint32_t validSectors = static_cast<uint32_t>(size / SECTOR_BYTES);
    if (firstSector == 0 || fh.preallocSectors == 0 || validSectors >= fh.preallocSectors) return 0;

    // Start two sectors before the recorded end to learn the last sequence
    // number already in the file.
    uint32_t sector = validSectors > 2 ? validSectors - 2 : 1;
    bool haveLast = false;
    uint32_t lastSeq = 0;
    bool sawEnd = false;
    uint32_t recovered = 0;

    if (!sd.card()->readSector(firstSector + sector, scratch)) return 0;

    for (; sector < fh.preallocSectors; sector++) {
        uint8_t* next = scratch + SECTOR_BYTES;
        if (sector + 1 >= fh.preallocSectors ||
            !sd.card()->readSector(firstSector + sector + 1, next)) {
            memset(next, 0, SECTOR_BYTES);
        }

        const uint32_t continuing = scanSector(scratch, fh.sessionId, haveLast, lastSeq, sawEnd);

        if (sector >= validSectors) {
            if (continuing == 0) break;
            if (f.write(scratch, SECTOR_BYTES) != SECTOR_BYTES) break;
            recovered++;
        }
        if (sawEnd) break;

        memcpy(scratch, next, SECTOR_BYTES);
    }
    return recovered;
}

/** Read a log's header, positioned at the start of the file. */
static bool readFileHeader(FsFile& f, FileHeader& fh) {
    return f.read(&fh, sizeof(fh)) == static_cast<int>(sizeof(fh)) &&
           fh.magic == FILE_MAGIC && fh.version == FORMAT_VERSION &&
           fh.headerBytes == HEADER_BYTES;
}

/**
 * Repair the newest log from an earlier boot: recover what a power cut left
 * past its last sync, then release the rest of its preallocation.
 *
 * A file closed by stop() ends in an END record and has no preallocation left,
 * so this finds nothing to do.
 */
static void recoverTail(const char* name) {
    FsFile old;
    if (!old.open(name, O_RDWR)) return;

    FileHeader fh;
    const uint64_t size = old.fileSize();
    if (!readFileHeader(old, fh) || fh.preallocSectors == 0 ||
        size < SECTOR_BYTES || size % SECTOR_BYTES != 0 ||
        size / SECTOR_BYTES >= fh.preallocSectors || !old.seekSet(size)) {
        // Not ours, not preallocated, or closed cleanly by stop() - whose final
        // partial-sector flush is the only way the size stops being whole
        // sectors.
        old.close();
        return;
    }

    const uint32_t recovered = extendTail(old, fh, size);

    // Releases the rest of the preallocation, whether or not anything was
    // recovered, and commits the new length.
    old.truncate();
    old.close();

    if (recovered > 0) {
        char msg[96];
        snprintf(msg, sizeof(msg), "Logger: recovered %lu sectors after power loss in %s",
                 static_cast<unsigned long>(recovered), name);
        USB::sendText(msg);
    }
}

/**
 * Reopen this session's log and carry on writing at its end, so one flight
 * stays in one file across a write error or a card that was pulled and put
 * back.
 *
 * Its preallocation is still in place - only stop() and an earlier boot's
 * recovery release it - so writing continues into reserved space just as
 * before. Records that reached the card after the last sync are recovered
 * first. Anything the RAM buffer still holds is written after them; where the
 * two overlap, the decoder drops the duplicates by sequence number.
 *
 * The header must carry this boot's session ID, so a different card - or a
 * file from another boot that happens to share the name - is never appended
 * to.
 *
 * @param credit Where the card-space bookkeeping currently counts this file,
 *        so it can be moved back to being counted as the open file.
 * @param rewindRing Instead of recovering the sectors written since the last
 *        sync from the card - a rewrite of each, and a sync, tens of
 *        milliseconds - wind the ring's tail back over them, so the normal
 *        paced writer sends them again. They are still in the ring unless new
 *        records have since overwritten them; if they have, the stretch is
 *        lost (the sequence numbers show it) rather than blocking. Only for
 *        the same card coming back, where fileBytes still describes this file.
 */
enum class Credit : uint8_t {
    NONE,     // Still counted as the open file: closed by onCardRemoved()
    BASELINE, // A remount since: the free-space baseline includes it
    RELEASED, // releaseFile() added it to usedSinceMount
};

static bool reopenLog(unsigned index, Credit credit, bool rewindRing = false) {
    const uint32_t openStartUs = micros();
    snprintf(fileName, sizeof(fileName), "LOG%03u.BIN", index);
    if (!file.open(fileName, O_RDWR)) return false;

    FileHeader fh;
    const uint64_t size = file.fileSize();
    if (!readFileHeader(file, fh) || fh.sessionId != sessionId ||
        size < SECTOR_BYTES || size % SECTOR_BYTES != 0 || !file.seekSet(size)) {
        file.close(); // Nothing written, so this does no card I/O
        return false;
    }

    if (rewindRing) {
        // fileBytes is how far into the file the ring had written; the card
        // has it only up to its recorded size. tail is sector-aligned and
        // both are whole sectors, so the rewound tail is too.
        const uint32_t behind = fileBytes > size ? static_cast<uint32_t>(fileBytes - size) : 0;
        bool rewound = false;
        __disable_irq(); // Against push() moving head between the check and the move
        if (behind <= BUFFER_BYTES && head - (tail - behind) <= BUFFER_BYTES) {
            tail = tail - behind;
            rewound = true;
        }
        __enable_irq();
        LOGGER_TRACE("Logger trace: reopen: open+header %lu us, ring %s %lu bytes",
                     static_cast<unsigned long>(micros() - openStartUs),
                     rewound ? "rewound" : "could not rewind", static_cast<unsigned long>(behind));
        if (!rewound) note("SD CARD RESUME GAP: records written before the dropout were lost");
    } else {
        const uint32_t extendStartUs = micros();
        const uint32_t recovered = extendTail(file, fh, size);
        const uint32_t syncStartUs = micros();
        if (!file.sync()) {
            file.close();
            return false;
        }
        LOGGER_TRACE("Logger trace: reopen: open+header %lu us, extend %lu sectors %lu us, sync %lu us",
                     static_cast<unsigned long>(extendStartUs - openStartUs), static_cast<unsigned long>(recovered),
                     static_cast<unsigned long>(syncStartUs - extendStartUs),
                     static_cast<unsigned long>(micros() - syncStartUs));
    }

    // The open file is counted on its own in getStatus() and filesystemFree(),
    // so take its space back out of whichever total already holds it.
    const uint64_t reserved = static_cast<uint64_t>(fh.preallocSectors) * SECTOR_BYTES;
    const uint64_t space = allocated(max(static_cast<uint64_t>(file.curPosition()), reserved));
    if (credit == Credit::BASELINE) freeAtMount += space;
    else if (credit == Credit::RELEASED) usedSinceMount -= min(usedSinceMount, space);

    fileOpen = true;
    haveFile = true;
    fileIndex = static_cast<uint16_t>(index);
    fileBytes = static_cast<uint32_t>(file.curPosition());
    fileReserved = static_cast<uint32_t>(reserved);
    lastSyncMs = millis();
    probeBeforeWrite = true;
    return true;
}

/** Card just mounted: repair the previous log, then start a new one. */
static void onMounted() {
    // A card that comes back after being pulled mid-log continues that log;
    // see onCardRemoved(). Anything else starts a fresh one.
    const bool resume = writer == State::CARD_OUT;
    if (!resume) writer = State::IDLE;

    unsigned newest = 0;
    if (scanLogs(newest)) {
        // The log being resumed is repaired by reopenLog() instead, which
        // keeps its preallocation; recoverTail() would release it.
        if (!(resume && newest == fileIndex)) {
            char name[24];
            snprintf(name, sizeof(name), "LOG%03u.BIN", newest);
            recoverTail(name);
        }
        newestIndex = newest;
        nextFileIndex = (newest + 1) % MAX_FILES;
    } else {
        newestIndex = MAX_FILES - 1; // So the oldest-first search starts at LOG000
        nextFileIndex = 0;
    }
    measureFreeSpace();

    if (resume) {
        // Back into the same file when it is this card, otherwise into a new
        // segment of the same session; either way everything buffered while
        // the card was out is still queued ahead of new records.
        const bool sameFile = haveFile && logExists(fileIndex) && reopenLog(fileIndex, Credit::BASELINE);
        if (sameFile || openSegment()) {
            writer = State::LOGGING;
            stalled = false;
            sameFileAttempts = 0;
            char msg[MAX_TEXT_BYTES];
            snprintf(msg, sizeof(msg), sameFile ? "Logger: card back, continuing %s"
                                                : "Logger: card back, log continues in %s", fileName);
            USB::sendText(msg);
            note(msg);
        } else {
            writer = State::REOPEN_WAIT;
            retryStartMs = millis();
            retryDelayMs = RETRY_MIN_MS;
        }
        return;
    }

    if (start() != Result::OK) {
        USB::sendText("Logger: could not create a log file");
    } else {
        USB::sendText("Logger: logging started");
    }
}

/** Hand DAT3 back to GPIO, after a failed sd.begin() left it muxed to SDIO. */
static bool readCardDetect() {
    pinMode(SD_DAT3_PIN, INPUT_PULLDOWN);
    delayMicroseconds(5); // Let the pull-down discharge the line
    return digitalReadFast(SD_DAT3_PIN);
}

static void clearStaleTransfer();

/** Mount a card inserted after boot. Only blocks when one is actually there. */
static void serviceCardDetect(uint32_t now) {
    if (now - lastDetectMs < CARD_DETECT_INTERVAL_MS) return;
    lastDetectMs = now;

    if (!digitalReadFast(SD_DAT3_PIN)) {
        presentReads = 0;
        cardUnmountable = false;
        return;
    }
    if (presentReads >= 2) return; // Already tried this insertion
    // Mounting, scanning and preallocating block for tens of milliseconds, so
    // hold off until the vehicle is on the ground.
    if (presentReads == 1 && inFlight()) return;
    if (++presentReads < 2) return;

    const uint32_t mountStartUs = micros();
    if (cardRemoved) clearStaleTransfer();
    const bool mounted = mount();
    LOGGER_TRACE("Logger trace: mount %s in %lu us", mounted ? "ok" : "failed",
                 static_cast<unsigned long>(micros() - mountStartUs));
    if (mounted) {
        const uint32_t onMountedStartUs = micros();
        onMounted();
        LOGGER_TRACE("Logger trace: mount to logging %lu us total, onMounted %lu us",
                     static_cast<unsigned long>(micros() - mountStartUs),
                     static_cast<unsigned long>(micros() - onMountedStartUs));
    } else {
        char msg[80];
        snprintf(msg, sizeof(msg), "Logger: inserted card would not mount (err 0x%02X); format it",
                 sd.sdErrorCode());
        USB::sendText(msg);
        readCardDetect();
        cardUnmountable = true;
    }
}

// MARK: Card removal

/**
 * Make SdFat's driver forget a transfer that will never finish.
 *
 * A card pulled mid-write leaves the driver's "transfer in progress" flag set,
 * and sd.begin() does not reset it, so the first read after a remount waits
 * out the driver's 1 s timeout on it. Forcing a data-timeout error through the
 * controller's FORCE_EVENT register ends that wait at once: syncDevice() sees
 * the error, drops the flag, and returns. It then tries to stop the write with
 * CMD12, which a card that is not there simply does not answer.
 */
/** Reset the SD controller and re-enable the status flags SdFat relies on. */
static void resetController() {
    SDHC_MIX_CTRL |= 0x80000000;
    SDHC_SYSCTL |= SDHC_SYSCTL_RSTA | SDHC_SYSCTL_SDCLKFS(0x80);
    const uint32_t startUs = micros();
    while ((SDHC_SYSCTL & SDHC_SYSCTL_RSTA) && micros() - startUs < 1000) {}
    SDHC_IRQSTATEN = SDHC_IRQSTATEN_DMAESEN | SDHC_IRQSTATEN_AC12ESEN | SDHC_IRQSTATEN_DEBESEN |
                     SDHC_IRQSTATEN_DCESEN | SDHC_IRQSTATEN_DTOESEN | SDHC_IRQSTATEN_CIESEN |
                     SDHC_IRQSTATEN_CEBESEN | SDHC_IRQSTATEN_CCESEN | SDHC_IRQSTATEN_CTOESEN |
                     SDHC_IRQSTATEN_DINTSEN | SDHC_IRQSTATEN_TCSEN | SDHC_IRQSTATEN_CCSEN;
}

static void clearStaleTransfer() {
    // Reset first: a card that vanished mid-command can leave the command line
    // held busy, and the CMD12 syncDevice() may send would wait out SdFat's
    // 1 s timeout for it.
    resetController();
    USDHC1_FORCE_EVENT = SDHC_IRQSTAT_DTOE;
    sd.card()->syncDevice();
    // And after: CMD12 is an abort command, which leaves the controller
    // holding the command line until its command and data paths are reset.
    resetController();
    SDHC_IRQSTAT = SDHC_IRQSTAT; // Nothing is in flight now; start clean
}

/** True if the mounted card answers a status request. */
static bool cardResponds() {
    return sd.card()->status() != 0;
}

/** cardResponds(), at most every CARD_PROBE_INTERVAL_MS; true means it did not. */
static bool cardMissing(uint32_t now) {
    if (now - lastProbeMs < CARD_PROBE_INTERVAL_MS) return false;
    lastProbeMs = now;
    return !cardResponds();
}

/**
 * The card was pulled out while mounted. Drop the file without touching the
 * card and go back to watching the slot.
 *
 * If it was logging, it carries on buffering in RAM (state CARD_OUT) while
 * serviceCardWake() waits for the card to come back and wakes it without
 * blocking, in flight too; the same card then continues the same file. The
 * ring holds about 2.5 s at the full telemetry rate; past that, new records
 * are dropped and counted, and the gap shows in the sequence numbers.
 *
 * file.close() is deliberately not called: it would try to write the
 * directory entry, and each of its requests waits out SdFat's 1 s timeout
 * before failing - long enough to trip the comm watchdog if this happens in
 * flight. Constructing a fresh FsFile over it forgets the open file instead.
 * FsFile owns no heap memory, so nothing leaks, and the volume state it
 * pointed into is rebuilt by the next sd.begin(). What the file had reached
 * at its last sync is on the card, and the next mount's recoverTail() picks
 * up any later sectors that made it there.
 */
static void onCardRemoved() {
    new (&file) FsFile();
    fileOpen = false;
    fileReserved = 0;
    stalled = false;

    writer = accepting ? State::CARD_OUT : State::NO_CARD;
    cardRemoved = true;
    wake = Wake::OFF;
    newCardWaiting = false;
    probeBeforeWrite = false;
    readCardDetect(); // Hand DAT3 back from the SDIO controller for slot polling
    presentReads = 0;
    lastDetectMs = millis();

    if (writer == State::CARD_OUT) {
        note("SD CARD REMOVED");
        USB::sendText("Logger: SD card lost; buffering until it is back");
    } else {
        USB::sendText("Logger: SD card removed");
    }
}

// MARK: Card wake

/** Clock the controller at no more than kHz, as SdFat's setSdclk() does. */
static void setWakeClock(uint32_t kHz) {
    const uint32_t base = (528000000U * 3) / ((CCM_ANALOG_PFD_528 & 0x3F) / 6) /
                          (((CCM_CSCDR1 >> 11) & 0x7) + 1);
    uint32_t sdclkfs = 1;
    uint32_t dvs = 1;
    while (base / (sdclkfs * 16) > kHz * 1000 && sdclkfs < 0x100) sdclkfs <<= 1;
    while (base / (sdclkfs * dvs) > kHz * 1000 && dvs < 16) dvs++;

    const uint32_t sysctl = SDHC_SYSCTL & ~(SDHC_SYSCTL_DVS_MASK | SDHC_SYSCTL_SDCLKFS_MASK);
    SDHC_SYSCTL = sysctl | SDHC_SYSCTL_DVS(dvs - 1) | SDHC_SYSCTL_SDCLKFS(sdclkfs >> 1);
    const uint32_t startUs = micros();
    while (!(SDHC_PRSSTAT & SDHC_PRSSTAT_SDSTB) && micros() - startUs < 1000) {}
}

/**
 * Send one command and wait for it - well under a millisecond at the 400 kHz
 * wake clock. No transfer is ever in flight while waking, so clearing the
 * controller's status flags here cannot lose one SdFat is waiting for.
 *
 * @return False if the card did not answer (or for CMD0, never).
 */
static uint32_t lastWakeStatus = 0; // INT_STATUS after the last wake command, for traces

static bool wakeCommand(uint8_t index, uint32_t responseFlags, uint32_t arg, uint32_t& response) {
    constexpr uint32_t COMMAND_ERRORS = SDHC_IRQSTAT_CTOE | SDHC_IRQSTAT_CCE |
                                        SDHC_IRQSTAT_CEBE | SDHC_IRQSTAT_CIE;
    const uint32_t startUs = micros();
    while (SDHC_PRSSTAT & SDHC_PRSSTAT_CIHB) {
        if (micros() - startUs > WAKE_COMMAND_TIMEOUT_US) return false;
    }
    SDHC_IRQSTAT = SDHC_IRQSTAT;
    SDHC_CMDARG = arg;
    SDHC_XFERTYP = SDHC_XFERTYP_CMDINX(index) | responseFlags;
    uint32_t status;
    while (!((status = SDHC_IRQSTAT) & (SDHC_IRQSTAT_CC | COMMAND_ERRORS))) {
        if (micros() - startUs > WAKE_COMMAND_TIMEOUT_US) return false;
    }
    SDHC_IRQSTAT = status;
    lastWakeStatus = status;
    response = SDHC_CMDRSP0;
    return !(status & COMMAND_ERRORS);
}

// Response types, as SdFat's CMD_RESP_* for these commands
static constexpr uint32_t RESP_NONE = SDHC_XFERTYP_RSPTYP(0);
static constexpr uint32_t RESP_R1_R7 = SDHC_XFERTYP_CICEN | SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(2);
static constexpr uint32_t RESP_R3 = SDHC_XFERTYP_RSPTYP(2);

/**
 * Put the controller in the state SdFat's initSDHC() leaves it in for card
 * identification: reset, 400 kHz, status flags enabled, DAT3 back on the bus,
 * and the 74+ clocks a card wants before its first command started. A pulled
 * card can leave it mid-command with the command line held busy, and only a
 * reset (in clearStaleTransfer()) clears that. Microseconds, apart from the clocks, which run on their
 * own while the next pass comes round.
 */
static void resetForWake() {
    clearStaleTransfer(); // Also resets the controller
    setWakeClock(WAKE_CLOCK_KHZ);

    // DAT3 back to USDHC1, with SdFat's pad setting and its pull-up. Slot
    // polling left a pull-down on it, and a card that sees DAT3 low when it
    // receives CMD0 switches to SPI mode - which only a power cycle undoes.
    IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_05 = IOMUXC_SW_PAD_CTL_PAD_PKE | IOMUXC_SW_PAD_CTL_PAD_DSE(7) |
                                          IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_PUE |
                                          IOMUXC_SW_PAD_CTL_PAD_PUS(1);
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_05 = 0;

    SDHC_SYSCTL |= SDHC_SYSCTL_INITA;
}

/** Give up on waking this card; watch the slot for it again. */
static void stopWake(const char* why) {
    LOGGER_TRACE("Logger trace: wake stopped, status %08lX sysctl %08lX prsstat %08lX proctl %08lX",
                 static_cast<unsigned long>(lastWakeStatus), static_cast<unsigned long>(SDHC_SYSCTL),
                 static_cast<unsigned long>(SDHC_PRSSTAT), static_cast<unsigned long>(SDHC_PROCTL));
    wake = Wake::OFF;
    readCardDetect(); // Back to GPIO for slot polling
    presentReads = 0;
    lastDetectMs = millis();
    char msg[80];
    snprintf(msg, sizeof(msg), "Logger: card came back but %s; will retry", why);
    USB::sendText(msg);
}

/**
 * The card has finished powering up. Hand it to SdFat, which now initialises
 * it in about 10 ms, and if it is the card that left, carry straight on with
 * the same file without rescanning it.
 */
static void finishWake() {
    wake = Wake::OFF;
    const cid_t oldCid = cardCid;
    const bool hadCid = haveCardCid;

    const uint32_t startUs = micros();
    if (!mount()) {
        stopWake("would not mount");
        return;
    }
    const uint32_t mountUs = micros() - startUs;
    const bool sameCard = hadCid && haveCardCid && memcmp(&oldCid, &cardCid, sizeof(cardCid)) == 0;

    if (!sameCard) {
        if (inFlight()) {
            // A different card: its logs and free space must be read before
            // writing to it, which blocks. That waits for the ground.
            newCardWaiting = true;
            USB::sendText("Logger: a different card came back; it is used once landed");
            return;
        }
        onMounted();
        return;
    }

    // Same card, same filesystem: the list of logs and the space bookkeeping
    // still hold, so neither has to be read again.
    if (reopenLog(fileIndex, Credit::NONE, true) || openSegment()) {
        writer = State::LOGGING;
        stalled = false;
        sameFileAttempts = 0;
        char msg[MAX_TEXT_BYTES];
        snprintf(msg, sizeof(msg), "Logger: card back, continuing %s", fileName);
        USB::sendText(msg);
        note(msg);
    } else {
        writer = State::REOPEN_WAIT;
        retryStartMs = millis();
        retryDelayMs = RETRY_MIN_MS;
    }
    LOGGER_TRACE("Logger trace: wake took %lu ms; mount %lu us, resume %lu us",
                 static_cast<unsigned long>(millis() - wakeStartMs),
                 static_cast<unsigned long>(mountUs),
                 static_cast<unsigned long>(micros() - startUs - mountUs));
}

/**
 * Bring back a card that left while logging, without blocking - in flight
 * included. Runs from update() while the logger is CARD_OUT.
 *
 * Watches the slot until a card is seen on two reads in a row, then walks it
 * through power-up one short step per pass: CMD0 to reset it, CMD8 to agree
 * the voltage, and ACMD41 every WAKE_POLL_INTERVAL_MS until it reports ready.
 * Every step is a single command at the 400 kHz identification clock, well
 * under a millisecond. Only then is it handed to SdFat (finishWake()).
 */
static void serviceCardWake(uint32_t now) {
    if (newCardWaiting) {
        if (!inFlight()) {
            newCardWaiting = false;
            onMounted();
        }
        return;
    }

    uint32_t response = 0;
    switch (wake) {
    case Wake::OFF:
        if (now - lastDetectMs < CARD_DETECT_INTERVAL_MS) return;
        lastDetectMs = now;
        if (!digitalReadFast(SD_DAT3_PIN)) {
            presentReads = 0;
            return;
        }
        if (++presentReads < 2) return; // One interval for the contacts to settle

        // Hand the pins back to the controller and slow it to the clock a card
        // being identified needs, then send it the 74+ clocks it wants first.
        resetForWake();
        wakeStartMs = now;
        lastWakeStepMs = now;
        wakeCmd8Tries = 0;
        wake = Wake::CMD0;
        return;

    case Wake::CMD0:
        if (SDHC_SYSCTL & SDHC_SYSCTL_INITA) return; // Still clocking
        if (now - lastWakeStepMs < (wakeCmd8Tries > 0 ? WAKE_POLL_INTERVAL_MS : 0)) return;
        lastWakeStepMs = now;
        wakeCommand(0, RESP_NONE, 0, response);
        wake = Wake::CMD8;
        return;

    case Wake::CMD8:
        if (!wakeCommand(8, RESP_R1_R7, 0x1AA, response) || response != 0x1AA) {
            if (++wakeCmd8Tries < WAKE_CMD8_TRIES) {
                wake = Wake::CMD0;
                return;
            }
            // Version 1 cards need SdFat's controller-reset workaround, which
            // its blocking begin() has; take that path on the ground.
            stopWake("did not answer CMD8");
            return;
        }
        wake = Wake::ACMD41;
        return;

    case Wake::ACMD41:
        if (now - lastWakeStepMs < WAKE_POLL_INTERVAL_MS) return;
        lastWakeStepMs = now;
        if (now - wakeStartMs > WAKE_TIMEOUT_MS) {
            stopWake("never finished powering up");
            return;
        }
        if (!wakeCommand(55, RESP_R1_R7, 0, response) ||
            !wakeCommand(41, RESP_R3, 0x40300000, response)) {
            stopWake("stopped answering");
            return;
        }
        if (response & 0x80000000) finishWake(); // Power-up done
        return;
    }
}

// MARK: Writer

/** Close the broken file and keep buffering until update() can continue. */
static void fail(const char* operation) {
    // A pulled card is the usual reason for a write to fail, and closing the
    // file on one would block; see onCardRemoved().
    if (!cardResponds()) {
        onCardRemoved();
        return;
    }

    writeErrors++;
    file.close();
    // Closed without truncate(), so the file keeps all of its preallocation
    releaseFile(max(static_cast<uint64_t>(fileBytes), static_cast<uint64_t>(fileReserved)));
    writer = State::REOPEN_WAIT;
    stalled = false;
    retryStartMs = millis();
    retryDelayMs = RETRY_MIN_MS;

    char msg[80];
    snprintf(msg, sizeof(msg), "Logger: %s failed on %s (sd err 0x%02X), retrying",
             operation, fileName, sd.sdErrorCode());
    USB::sendText(msg);
}

/** Write whole queued sectors while the card will take them. */
static void drain() {
    for (int i = 0; i < MAX_SECTORS_PER_PASS; i++) {
        // head is a single aligned 32-bit load, so the ISR cannot tear it; at
        // worst it is one record stale, which only delays that record a pass.
        const uint32_t queued = head - tail;
        if (queued > highWater) highWater = queued;

        if (queued < SECTOR_BYTES) {
            stalled = false;
            return;
        }


        // First write since the card's multi-sector write ended: nothing is in
        // flight, so it is safe to ask whether the card is still there - and
        // necessary, because SdFat starts the new write by polling the card's
        // status for up to 1 s, which on a card that has gone is all of it.
        if (probeBeforeWrite) {
            if (!cardResponds()) {
                onCardRemoved();
                return;
            }
            probeBeforeWrite = false;
        }

        // The card is still programming the last sector. Writing now would
        // block until it finishes - the stall the ring exists to absorb.
        if (file.isBusy()) {
            if (!stalled) {
                stalled = true;
                stallStartUs = micros();
            } else if (micros() - stallStartUs >= STALL_PROBE_US && cardMissing(millis())) {
                // Busy for longer than any healthy write stall: it may be gone
                onCardRemoved();
            }
            return;
        }
        if (stalled) {
            const uint32_t stallUs = micros() - stallStartUs;
            if (stallUs > intervalMaxStallUs) intervalMaxStallUs = stallUs;
            stalled = false;
        }

        if (file.write(&ring[tail & BUFFER_MASK], SECTOR_BYTES) != SECTOR_BYTES) {
            // tail is not advanced, so the next segment starts with this same
            // sector. If part of it did land here, the decoder drops the
            // duplicates by sequence number.
            fail("write");
            return;
        }
        tail = tail + SECTOR_BYTES;
        fileBytes += SECTOR_BYTES;
        sameFileAttempts = 0;
    }
}

static void sampleRate(uint32_t now) {
    const uint32_t elapsed = now - rateSampleMs;
    if (elapsed < RATE_INTERVAL_MS) return;

    const uint32_t sampleHead = head;
    bytesPerSec = static_cast<uint32_t>(
        static_cast<uint64_t>(sampleHead - rateSampleHead) * 1000 / elapsed);
    if (bytesPerSec > 0) lastActiveBytesPerSec = bytesPerSec;
    rateSampleHead = sampleHead;
    rateSampleMs = now;
}

static void syncIfDue(uint32_t now) {
    if (now - lastSyncMs < SYNC_INTERVAL_MS || file.isBusy()) return;

    const uint32_t startUs = micros();
    // End the card's multi-sector write first, so asking it a question cannot
    // clear a transfer-complete flag the driver still wants, then make sure it
    // is there before the sync's directory write polls a missing card for 1 s.
    if (!sd.card()->syncDevice() || !cardResponds()) {
        fail("sync");
        return;
    }
    if (!file.sync()) {
        fail("sync");
        return;
    }
    const uint32_t syncUs = micros() - startUs;
    if (syncUs > intervalMaxSyncUs) intervalMaxSyncUs = syncUs;
    lastSyncMs = now;
    probeBeforeWrite = true;
}

static void recordStatsIfDue(uint32_t now) {
    if (now - lastStatsMs < STATS_INTERVAL_MS) return;

    StatsRecord stats;
    stats.dropped = dropped;
    stats.highWaterBytes = highWater;
    stats.maxStallUs = intervalMaxStallUs;
    stats.maxSyncUs = intervalMaxSyncUs;
    stats.writeErrors = writeErrors;
    stats.segment = currentSegment;
    push(RecordType::STATS, &stats, sizeof(stats));

    intervalMaxStallUs = 0;
    intervalMaxSyncUs = 0;
    lastStatsMs = now;
}

/**
 * After an error: reopen the same file on the same mount, then a new segment,
 * both of which are cheap. Only if neither works, and only on the ground,
 * remount the card - sd.begin() can block long enough to trip the comm
 * watchdog.
 */
static void serviceReopen(uint32_t now) {
    if (now - retryStartMs < retryDelayMs) return;
    // Opening a file or remounting with no card there blocks for seconds
    if (!cardResponds()) {
        onCardRemoved();
        return;
    }

    // The same file first, so a passing glitch leaves one flight in one file.
    // A new file only if that keeps failing, and a remount only on the ground.
    bool sameFile = false;
    if (haveFile && logExists(fileIndex) && sameFileAttempts < MAX_SAME_FILE_ATTEMPTS) {
        sameFileAttempts++;
        sameFile = reopenLog(fileIndex, Credit::RELEASED);
    }
    bool resumed = sameFile || openSegment();
    if (!resumed && !inFlight()) {
        if (mount()) {
            measureFreeSpace();
            sameFile = haveFile && logExists(fileIndex) && reopenLog(fileIndex, Credit::BASELINE);
            resumed = sameFile || openSegment();
        }
    }

    if (resumed) {
        writer = State::LOGGING;
        char msg[MAX_TEXT_BYTES];
        snprintf(msg, sizeof(msg), sameFile ? "Logger: write error recovered, continuing %s"
                                            : "Logger: resumed in new file %s", fileName);
        USB::sendText(msg);
        note(msg);
        return;
    }

    retryStartMs = now;
    retryDelayMs = min(retryDelayMs * 2, RETRY_MAX_MS);
}

// MARK: Public interface

bool setup() {
    // Identifies this boot's files and seeds every record CRC, so it only has
    // to differ between sessions. The RTC seconds plus the cycle counter -
    // whose value here depends on how long the earlier startup stages took -
    // is plenty for that.
    sessionId = rtc_get() ^ (ARM_DWT_CYCCNT * 2654435761u);

    // Check the slot first: sd.begin() on an empty one wastes about 2 s.
    lastDetectMs = millis();
    if (!readCardDetect()) {
        USB::sendText("Logger: no SD card; insert one on the ground to start logging");
        presentReads = 0;
        writer = State::NO_CARD;
        return true; // A missing card is not a flight fault
    }

    if (!mount()) {
        char msg[80];
        snprintf(msg, sizeof(msg), "Logger: SD card would not mount (err 0x%02X); format it",
                 sd.sdErrorCode());
        USB::sendText(msg);
        readCardDetect();
        cardUnmountable = true;
        presentReads = 2; // Not retried until it is reinserted
        writer = State::NO_CARD;
        return true;
    }

    onMounted();
    return true;
}

bool setupComplete() { return writer != State::NO_CARD && writer != State::CARD_OUT; }

void update() {
    const uint32_t now = millis();
    sampleRate(now);

    switch (writer) {
    case State::NO_CARD:
        serviceCardDetect(now);
        return;

    case State::CARD_OUT: {
        recordStatsIfDue(now); // Into the ring, so the log shows the buffer filling
        const uint32_t passStartUs = micros();
        serviceCardWake(now);
        // The figure that matters in flight: the longest the loop was held
        // up at once, against the comm watchdog's 100 ms.
        static uint32_t longestPassUs = 0;
        longestPassUs = max(longestPassUs, micros() - passStartUs);
        if (writer != State::CARD_OUT) {
            LOGGER_TRACE("Logger trace: longest single pass while the card was out: %lu us",
                         static_cast<unsigned long>(longestPassUs));
            longestPassUs = 0;
        }
        return;
    }

    case State::IDLE:
        if (cardMissing(now)) onCardRemoved();
        return;

    case State::REOPEN_WAIT:
        recordStatsIfDue(now);
        if (cardMissing(now)) {
            onCardRemoved();
            return;
        }
        reclaimIfLow(now); // A full card is one reason the reopen keeps failing
        serviceReopen(now);
        return;

    case State::LOGGING:
        recordStatsIfDue(now);
        drain();
        if (writer == State::LOGGING) syncIfDue(now);
        if (writer == State::LOGGING) reclaimIfLow(now);
        return;
    }
}

Result start() {
    if (writer == State::NO_CARD || writer == State::CARD_OUT) return Result::NO_CARD;
    if (writer != State::IDLE) return Result::WRONG_STATE;
    if (inFlight()) return Result::IN_FLIGHT;
    if (!openSegment()) return Result::CARD_ERROR;

    // The ring is empty here - stop() flushed it - but its final partial
    // flush left tail off a sector boundary. Hold off the ISR, which may be
    // calling record(), while both counters move.
    __disable_irq();
    head = 0;
    tail = 0;
    rateSampleHead = 0;
    accepting = true;
    __enable_irq();

    stalled = false;
    lastStatsMs = millis();
    rateSampleMs = lastStatsMs;
    writer = State::LOGGING;
    return Result::OK;
}

/**
 * Zero the ring's loss and peak counters, so the next log starts its own
 * count. Only once accepting is clear: push() no longer touches dropped, and
 * drain() has nothing left to raise highWater with.
 */
static void resetBufferStats() {
    dropped = 0;
    highWater = 0;
}

Result stop() {
    if (writer == State::NO_CARD) return Result::NO_CARD;
    if (writer == State::IDLE) return Result::WRONG_STATE;
    if (inFlight()) return Result::IN_FLIGHT;

    if (writer == State::REOPEN_WAIT || writer == State::CARD_OUT) {
        // No file to flush into. What is queued is lost; say how much.
        accepting = false;
        char msg[64];
        snprintf(msg, sizeof(msg), "Logger: stopped while recovering, %lu bytes discarded",
                 static_cast<unsigned long>(head - tail));
        USB::sendText(msg);
        resetBufferStats();
        writer = writer == State::CARD_OUT ? State::NO_CARD : State::IDLE;
        return Result::OK;
    }

    push(RecordType::END, nullptr, 0);
    accepting = false; // Nothing new enters the ring from here

    while (head != tail) {
        const uint32_t pos = tail & BUFFER_MASK;
        const uint32_t n = min(head - tail, BUFFER_BYTES - pos);
        if (file.write(&ring[pos], n) != n) {
            fail("final flush");
            writer = State::IDLE;
            return Result::CARD_ERROR;
        }
        tail = tail + n;
        fileBytes += n;
    }

    file.truncate(); // Release the unused preallocation past the current position
    file.close();
    releaseFile(fileBytes);
    resetBufferStats();
    writer = State::IDLE;
    return Result::OK;
}

bool isLogging() { return writer == State::LOGGING; }

void record(const Drone::Telemetry_t& t) {
    push(RecordType::TELEMETRY, &t, sizeof(t));
}

void note(const char* text) {
    if (text == nullptr) return;
    push(RecordType::TEXT, text, static_cast<uint16_t>(strnlen(text, MAX_TEXT_BYTES)));
}

uint32_t getDropped() { return dropped; }
uint32_t getHighWater() { return highWater; }
uint32_t getWriteErrors() { return writeErrors; }

#if defined(LOGGER_TEST_HOOKS)
/**
 * Put the card back in its idle state with CMD0, which is what a brief loss
 * of contact does to it - minus the loss of power. It stops answering
 * everything the logger sends, so the removal path runs exactly as for a
 * real dropout, and the card has to be initialised again to come back.
 */
static Result testResetCard(bool midWrite) {
    if (writer == State::NO_CARD || writer == State::CARD_OUT) return Result::NO_CARD;
    // Normally the open transfer is finished first. midWrite leaves it open,
    // as a real pull does: the write never completes, and the driver is left
    // believing it is still in progress - see clearStaleTransfer().
    if (!midWrite && fileOpen && !file.sync()) return Result::CARD_ERROR;

    const uint32_t startUs = micros();
    while ((SDHC_PRSSTAT & SDHC_PRSSTAT_CIHB) && micros() - startUs < 1000) {}
    SDHC_CMDARG = 0;
    SDHC_XFERTYP = SDHC_XFERTYP_CMDINX(0); // CMD0, no response
    while (!(SDHC_IRQSTAT & (SDHC_IRQSTAT_CC | SDHC_IRQSTAT_CTOE | SDHC_IRQSTAT_CCE | SDHC_IRQSTAT_CEBE | SDHC_IRQSTAT_CIE)) && micros() - startUs < 2000) {}
    // Only the command's own flags: mid-write, the transfer's are not ours
    SDHC_IRQSTAT = SDHC_IRQSTAT_CC | SDHC_IRQSTAT_CTOE | SDHC_IRQSTAT_CCE | SDHC_IRQSTAT_CEBE | SDHC_IRQSTAT_CIE;
    if (!midWrite) probeBeforeWrite = true; // As after any sync: the next write starts a new transfer
    USB::sendText(midWrite ? "Logger test: sent the card CMD0 mid-write" : "Logger test: sent the card CMD0");
    return Result::OK;
}
#endif

#if defined(LOGGER_TEST_HOOKS)
/**
 * Read the open log back from the card and check it record by record: every
 * CRC, and every sequence number following the last. Blocks for as long as
 * reading the whole file takes, so ground tests only.
 */
static Result testVerifyLog() {
    if (!fileOpen) return Result::WRONG_STATE;
    if (!sd.card()->syncDevice() || !file.sync()) return Result::CARD_ERROR;
    probeBeforeWrite = true;

    FsFile check;
    if (!check.open(fileName, O_RDONLY) || !check.seekSet(HEADER_BYTES)) return Result::CARD_ERROR;

    static uint8_t buf[8192];
    uint32_t have = 0;
    uint32_t records = 0, gaps = 0, missing = 0, duplicates = 0, skipped = 0;
    uint32_t firstSeq = 0, lastSeq = 0;
    bool haveLast = false;
    const uint32_t startMs = millis();

    for (;;) {
        const int got = check.read(buf + have, sizeof(buf) - have);
        if (got > 0) have += static_cast<uint32_t>(got);
        uint32_t pos = 0;
        while (pos + sizeof(RecordHeader) <= have) {
            RecordHeader rh;
            if (buf[pos] != RECORD_SYNC) { pos++; skipped++; continue; }
            memcpy(&rh, buf + pos, sizeof(rh));
            if (rh.length > MAX_PAYLOAD_BYTES) { pos++; skipped++; continue; }
            if (pos + sizeof(rh) + rh.length > have) break; // Rest arrives with the next read
            if (recordCrc(sessionId, &rh, buf + pos + sizeof(rh), rh.length) != rh.crc) {
                pos++;
                skipped++;
                continue;
            }
            if (!haveLast) {
                firstSeq = rh.seq;
            } else if (rh.seq <= lastSeq) {
                duplicates++;
            } else if (rh.seq != lastSeq + 1) {
                gaps++;
                missing += rh.seq - lastSeq - 1;
            }
            if (!haveLast || rh.seq > lastSeq) lastSeq = rh.seq;
            haveLast = true;
            records++;
            pos += sizeof(rh) + rh.length;
        }
        memmove(buf, buf + pos, have - pos);
        have -= pos;
        if (got <= 0) break;
    }
    check.close();

    char msg[160];
    snprintf(msg, sizeof(msg),
             "Logger test: verify %s: %lu records, seq %lu-%lu, %lu gaps (%lu missing), %lu duplicates, "
             "%lu bytes skipped, %lu trailing, %lu ms",
             fileName, static_cast<unsigned long>(records), static_cast<unsigned long>(firstSeq),
             static_cast<unsigned long>(lastSeq), static_cast<unsigned long>(gaps),
             static_cast<unsigned long>(missing), static_cast<unsigned long>(duplicates),
             static_cast<unsigned long>(skipped), static_cast<unsigned long>(have),
             static_cast<unsigned long>(millis() - startMs));
    USB::sendText(msg);
    return Result::OK;
}
#endif

Result mark(const char* label, size_t length) {
    if (writer == State::NO_CARD) return Result::NO_CARD;
    if (!accepting) return Result::WRONG_STATE;

    markCount++;
    char text[MAX_TEXT_BYTES + 1];
    snprintf(text, sizeof(text), "MARK %u: %.*s", static_cast<unsigned>(markCount),
             static_cast<int>(min(length, MAX_MARK_LABEL)), label != nullptr ? label : "");
    note(text);
    return Result::OK;
}

Result command(Action action, const char* label, size_t labelLength) {
    Result result = Result::UNKNOWN_ACTION;
    switch (action) {
    case Action::START:
        result = start();
        if (result == Result::OK) note("LOG START by operator");
        break;
    case Action::STOP:
        // Noted first: stop() closes the file behind its END record.
        if (writer == State::LOGGING && !inFlight()) note("LOG STOP by operator");
        result = stop();
        break;
    case Action::MARK:
        result = mark(label, labelLength);
        break;
#if defined(LOGGER_TEST_HOOKS)
    case Action::TEST_CARD_RESET:
        result = testResetCard(false);
        break;
    case Action::TEST_CARD_RESET_MID_WRITE:
        result = testResetCard(true);
        break;
    case Action::TEST_VERIFY_LOG:
        result = testVerifyLog();
        break;
#endif
    }

    commandCount++;
    commandResult = result;
    return result;
}

void getStatus(Status& out) {
    out = Status{};
    out.state = writer;
    out.cardUnmountable = cardUnmountable;
    out.cardRemoved = (writer == State::NO_CARD || writer == State::CARD_OUT) && cardRemoved;
    out.slowFilesystem = writer != State::NO_CARD && writer != State::CARD_OUT && slowFilesystem;
    out.fileOpen = fileOpen;
    out.haveFile = haveFile;
    out.fileIndex = fileIndex;
    out.fileBytes = fileBytes;
    out.bytesPerSec = bytesPerSec;
    out.dropped = dropped;
    out.writeErrors = writeErrors;
    out.highWaterBytes = highWater;
    out.bufferBytes = BUFFER_BYTES;
    out.commandCount = commandCount;
    out.commandResult = commandResult;
    out.marks = markCount;
    out.filesRecycled = filesRecycled;
    if (writer == State::NO_CARD || writer == State::CARD_OUT) return;

    // What a log can still grow into: free space, plus the open file's
    // preallocation it has not written yet, since that is reserved for it.
    out.cardBytes = cardBytes;
    const uint64_t used = usedSinceMount + (fileOpen ? allocated(fileBytes) : 0);
    out.freeBytes = freeAtMount > used ? freeAtMount - used : 0;

    const uint32_t rate = lastActiveBytesPerSec > 0 ? lastActiveBytesPerSec : NOMINAL_BYTES_PER_SEC;
    const uint64_t seconds = out.freeBytes / rate;
    out.secondsRemaining = seconds > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(seconds);
}

} // namespace Logger

#else // Bench target: no SD hardware

namespace Logger {

bool setup() { return true; }
bool setupComplete() { return false; }
void update() {}
Result start() { return Result::NO_CARD; }
Result stop() { return Result::NO_CARD; }
bool isLogging() { return false; }
void record(const Drone::Telemetry_t&) {}
void note(const char*) {}
uint32_t getDropped() { return 0; }
uint32_t getHighWater() { return 0; }
uint32_t getWriteErrors() { return 0; }
Result mark(const char*, size_t) { return Result::NO_CARD; }
Result command(Action, const char*, size_t) { return Result::NO_CARD; }
void getStatus(Status& out) { out = Status{}; }

} // namespace Logger

#endif
