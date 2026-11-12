#!/usr/bin/env python3
"""
Decode AFC-Drone flight logs (LOGnnn.BIN) into CSV.

    python3 scripts/decode_log.py /media/sd/LOG*.BIN -o decoded/

Files are grouped by session (one session per boot) and their segments joined
in order. Records are deduplicated by sequence number - a segment opened after
a write error may repeat the last sector of the one before - and every gap in
the sequence is reported, so lost data is never silent.

Writes, per session:
    session_<id>_telemetry.csv
    session_<id>_stats.csv
    session_<id>_text.csv
    session_<id>_imu.csv        BNO08x reports with sensor sample times
    session_<id>_timesync.csv   base-station clock-sync exchanges (t2, t3)

The format is defined in include/logger.h. Keep the structs below in step with
it and with Drone::Telemetry_t in include/drone.h; logger.cpp static_asserts
the layout this script assumes.
"""

import argparse
import csv
import struct
import sys
import zlib
from collections import defaultdict
from pathlib import Path

FILE_MAGIC = 0x4C434641
FORMAT_VERSION = 1
RECORD_SYNC = 0xA5
MAX_PAYLOAD_BYTES = 128

FILE_HEADER = struct.Struct("<IHHIHHHHI")
RECORD_HEADER = struct.Struct("<BBHIII")
STATS = struct.Struct("<IIIIHH")

TYPE_TELEMETRY = 1
TYPE_STATS = 2
TYPE_TEXT = 3
TYPE_END = 4
TYPE_IMU = 5
TYPE_TIMESYNC = 6

# Logger::ImuRecord and Logger::TimeSyncRecord.
IMU = struct.Struct("<BBBBII4f")
TIMESYNC = struct.Struct("<B3xII")
IMU_FLAG_INT_TIMED = 1
IMU_SENSOR_NAMES = {0x02: "gyro", 0x04: "linear_accel", 0x08: "game_rotation"}

# Drone::Telemetry_t as laid out in memory, padding included: 3 pad bytes
# after the 1-byte state so the floats that follow are 4-byte aligned.
TELEMETRY = struct.Struct("<4H2IB3x2f2h2H4f9f")
TELEMETRY_FIELDS = [
    "loopTimeLast", "loopTimeAvg", "loopTimeMax", "loopTimeMin",
    "missedTicks", "runtimeSec", "state",
    "gimbalPitch", "gimbalYaw", "topServoSet", "bottomServoSet",
    "topMotorSet", "bottomMotorSet",
    "qR", "qI", "qJ", "qK",
    "accelX", "accelY", "accelZ",
    "velX", "velY", "velZ",
    "posX", "posY", "posZ",
]
assert TELEMETRY.size == 88

STATE_NAMES = {
    0: "BOOT", 1: "RADIO_SETUP", 2: "SENSOR_SETUP", 3: "CONTROL_SETUP",
    4: "SAFE", 10: "READY_ARMED", 11: "MAN_FLIGHT", 12: "AUTO_FLIGHT",
    255: "FAULT_ERROR",
}


def read_header(data, path):
    if len(data) < FILE_HEADER.size:
        raise ValueError(f"{path}: too short for a log header")
    (magic, version, header_bytes, session, segment, record_header_bytes,
     telemetry_bytes, _reserved, _prealloc) = FILE_HEADER.unpack_from(data)
    if magic != FILE_MAGIC:
        raise ValueError(f"{path}: not an AFC-Drone log")
    if version != FORMAT_VERSION:
        raise ValueError(f"{path}: format version {version}, this decoder reads {FORMAT_VERSION}")
    if record_header_bytes != RECORD_HEADER.size or telemetry_bytes != TELEMETRY.size:
        raise ValueError(f"{path}: record layout does not match this decoder")
    return session, segment, header_bytes


def scan_records(data, start, session):
    """
    Yield (seq, type, timeUs, payload) for every record whose CRC checks out.

    Anything that does not check out - a torn sector, the tail of a record
    whose start was lost, zeroed space past the end - is skipped one byte at a
    time until the next valid record. Returns the number of bytes skipped.
    """
    offset = start
    skipped = 0
    end = len(data)
    while offset + RECORD_HEADER.size <= end:
        if data[offset] != RECORD_SYNC:
            offset += 1
            skipped += 1
            continue
        _sync, rtype, length, seq, time_us, crc = RECORD_HEADER.unpack_from(data, offset)
        body = offset + RECORD_HEADER.size
        if length > MAX_PAYLOAD_BYTES or body + length > end:
            offset += 1
            skipped += 1
            continue
        payload = data[body:body + length]
        if zlib.crc32(data[offset:offset + 12] + payload, session) != crc:
            offset += 1
            skipped += 1
            continue
        yield seq, rtype, time_us, payload
        offset = body + length
    return skipped


def unwrap_near(value_us, record_us, wraps):
    """
    Unwrap a 32-bit micros() value taken close to (within ~35 min of) the
    record's own timeUs, which has already been unwrapped by `wraps`.
    """
    delta = (value_us - record_us + (1 << 31)) % (1 << 32) - (1 << 31)
    return record_us + (wraps << 32) + delta


def decode_session(session, segments, out_dir):
    records = {}
    skipped_total = 0
    ended = False

    for segment, path, data, header_bytes in sorted(segments):
        gen = scan_records(data, header_bytes, session)
        while True:
            try:
                seq, rtype, time_us, payload = next(gen)
            except StopIteration as stop:
                skipped_total += stop.value or 0
                break
            records.setdefault(seq, (rtype, time_us, payload))
            if rtype == TYPE_END:
                ended = True

    if not records:
        print(f"session {session:08x}: no valid records")
        return

    seqs = sorted(records)
    gaps = []
    for prev, cur in zip(seqs, seqs[1:]):
        if cur != prev + 1:
            gaps.append((prev + 1, cur - 1))
    missing = sum(b - a + 1 for a, b in gaps)

    stem = out_dir / f"session_{session:08x}"
    files = {
        TYPE_TELEMETRY: open(f"{stem}_telemetry.csv", "w", newline=""),
        TYPE_STATS: open(f"{stem}_stats.csv", "w", newline=""),
        TYPE_TEXT: open(f"{stem}_text.csv", "w", newline=""),
        TYPE_IMU: open(f"{stem}_imu.csv", "w", newline=""),
        TYPE_TIMESYNC: open(f"{stem}_timesync.csv", "w", newline=""),
    }
    writers = {t: csv.writer(f) for t, f in files.items()}
    writers[TYPE_TELEMETRY].writerow(["seq", "timeS"] + TELEMETRY_FIELDS + ["stateName"])
    writers[TYPE_STATS].writerow(["seq", "timeS", "dropped", "highWaterBytes",
                                  "maxStallUs", "maxSyncUs", "writeErrors", "segment"])
    writers[TYPE_TEXT].writerow(["seq", "timeS", "text"])
    # sampleUs/intUs/rxUs/txUs are raw 32-bit drone micros(); the *S columns
    # are the same instants unwrapped, in seconds.
    writers[TYPE_IMU].writerow(["seq", "timeS", "sensor", "sensorId", "sequence", "status",
                                "intTimed", "sampleUs", "intUs", "sampleS",
                                "v0", "v1", "v2", "v3"])
    writers[TYPE_TIMESYNC].writerow(["seq", "timeS", "pingSeq", "rxUs", "txUs", "rxS", "txS"])

    # micros() wraps every 2^32 us (~71.6 min). Records are in time order, so
    # a large backwards step is a wrap, not a real jump.
    wraps = 0
    last_us = None
    counts = defaultdict(int)
    events = []  # Marks and state changes, echoed to the console
    for seq in seqs:
        rtype, time_us, payload = records[seq]
        if last_us is not None and time_us < last_us and last_us - time_us > (1 << 31):
            wraps += 1
        last_us = time_us
        time_s = f"{(time_us + (wraps << 32)) / 1e6:.6f}"
        counts[rtype] += 1

        if rtype == TYPE_TELEMETRY and len(payload) == TELEMETRY.size:
            values = TELEMETRY.unpack(payload)
            state = values[TELEMETRY_FIELDS.index("state")]
            writers[rtype].writerow([seq, time_s, *values, STATE_NAMES.get(state, state)])
        elif rtype == TYPE_STATS and len(payload) == STATS.size:
            writers[rtype].writerow([seq, time_s, *STATS.unpack(payload)])
        elif rtype == TYPE_IMU and len(payload) == IMU.size:
            sid, sequence, status, flags, sample_us, int_us, *v = IMU.unpack(payload)
            writers[rtype].writerow([seq, time_s, IMU_SENSOR_NAMES.get(sid, sid), sid, sequence,
                                     status, int(bool(flags & IMU_FLAG_INT_TIMED)),
                                     sample_us, int_us,
                                     f"{unwrap_near(sample_us, time_us, wraps) / 1e6:.6f}", *v])
        elif rtype == TYPE_TIMESYNC and len(payload) == TIMESYNC.size:
            ping_seq, rx_us, tx_us = TIMESYNC.unpack(payload)
            writers[rtype].writerow([seq, time_s, ping_seq, rx_us, tx_us,
                                     f"{unwrap_near(rx_us, time_us, wraps) / 1e6:.6f}",
                                     f"{unwrap_near(tx_us, time_us, wraps) / 1e6:.6f}"])
        elif rtype == TYPE_TEXT:
            text = payload.decode("ascii", "replace")
            writers[rtype].writerow([seq, time_s, text])
            if text.startswith(("MARK ", "STATE ", "COMM WATCHDOG", "LOG ", "SD CARD", "Logger: ")):
                events.append((time_s, text))

    for f in files.values():
        f.close()

    names = ", ".join(p.name for _, p, _, _ in sorted(segments))
    print(f"session {session:08x} ({names})")
    print(f"  {counts[TYPE_TELEMETRY]} telemetry, {counts[TYPE_STATS]} stats, "
          f"{counts[TYPE_TEXT]} text, {counts[TYPE_IMU]} imu, "
          f"{counts[TYPE_TIMESYNC]} timesync records; seq {seqs[0]}-{seqs[-1]}")
    print(f"  {missing} records missing in {len(gaps)} gaps; {skipped_total} bytes skipped")
    for a, b in gaps[:20]:
        print(f"    missing seq {a}-{b} ({b - a + 1})")
    if len(gaps) > 20:
        print(f"    ... {len(gaps) - 20} more")
    if events:
        print("  events:")
        for time_s, text in events[:50]:
            print(f"    {float(time_s):10.3f} s  {text}")
        if len(events) > 50:
            print(f"    ... {len(events) - 50} more in {stem.name}_text.csv")
    print("  ended cleanly" if ended else "  no END record: power was cut or the logger never stopped")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip())
    parser.add_argument("files", nargs="+", type=Path, help="LOGnnn.BIN files")
    parser.add_argument("-o", "--out", type=Path, default=Path("."), help="output directory")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    sessions = defaultdict(list)
    for path in args.files:
        data = path.read_bytes()
        try:
            session, segment, header_bytes = read_header(data, path)
        except ValueError as err:
            print(f"skipping {err}", file=sys.stderr)
            continue
        sessions[session].append((segment, path, data, header_bytes))

    for session, segments in sessions.items():
        decode_session(session, segments, args.out)


if __name__ == "__main__":
    main()
