#!/usr/bin/env python3
"""
Put pendulum-test magnet passes on the drone's clock and compare them with the gyro.

    python3 scripts/decode_log.py LOG012.BIN -o decoded/
    python3 scripts/pendulum_sync.py \\
        --base   test_logs/pendulum_20261111_140000 \\
        --drone  decoded/session_1a2b3c4d \\
        -o pendulum_results.csv

--base is the stem the dashboard server's --pendulum-log wrote
(<stem>_timesync.csv, <stem>_passes.csv). --drone is the stem decode_log.py
wrote (<stem>_imu.csv, <stem>_timesync.csv).

Clock sync
----------
Each exchange gives t1 (base sends ping), t2 (drone receives it), t3 (drone
sends pong), t4 (base receives pong), every one stamped in an RFM69 interrupt.
The midpoints (t1+t4)/2 on the base clock and (t2+t3)/2 on the drone clock are
the same instant to within the link's asymmetry, which is tiny because both
ends use the same radio. Exchanges whose round trip is unusually long (an
interrupt held off, a busy loop) are dropped. Then, for each pass, a straight
line drone = a * base + b is fitted to the exchanges within --window seconds
of it. A local fit follows the Uno's ceramic resonator, which drifts with
temperature as well as being off by up to ~0.5 %.

The residual of that fit is printed per pass: it is the measured sync error.

Gyro comparison
---------------
The swing axis is the principal axis of the calibrated gyro readings. At the
bottom of the swing the angular rate about it peaks; a sinusoid is fitted to
|rate| around each pass to find that peak, and its time is compared with the
magnet centroid.

Times are micros() counters that wrap every 2^32 us. They are unwrapped
relative to the first exchange, which is valid for runs under ~35 minutes.
"""

import argparse
import csv
import statistics
import sys
from pathlib import Path

WRAP = 1 << 32
HALF = 1 << 31
NO_PREV = 0xFFFF


def rel(value, anchor):
    """value - anchor for 32-bit counters, assuming they are within 2^31 us."""
    return (int(value) - anchor + HALF) % WRAP - HALF


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def load_exchanges(base_stem, drone_stem):
    """Return [(t1, t2, t3, t4)] in raw 32-bit counts, oldest first."""
    rows = read_csv(f"{base_stem}_timesync.csv")
    t2_by_seq = {}
    for r in rows:
        t2_by_seq[int(r["seq"])] = int(r["t2Us"])

    # t3 from the pongs themselves: each carries the previous pong's t3 - t2.
    t3_by_seq = {}
    for r in rows:
        seq, prev_low, delta = int(r["seq"]), int(r["prevSeq"]), int(r["prevTxMinusRxUs"])
        back = (seq - prev_low) & 0xFF
        prev = seq - back
        if delta != NO_PREV and back != 0 and prev in t2_by_seq:
            t3_by_seq[prev] = (t2_by_seq[prev] + delta) % WRAP

    # And from the drone's SD log, which has t3 even for the last pong.
    drone_path = Path(f"{drone_stem}_timesync.csv") if drone_stem else None
    from_sd = 0
    if drone_path and drone_path.exists():
        by_rx = {(int(r["pingSeq"]), int(r["rxUs"])): int(r["txUs"]) for r in read_csv(drone_path)}
        for seq, t2 in t2_by_seq.items():
            t3 = by_rx.get((seq & 0xFF, t2))
            if t3 is not None and seq not in t3_by_seq:
                t3_by_seq[seq] = t3
                from_sd += 1

    out = []
    for r in rows:
        seq = int(r["seq"])
        if seq in t3_by_seq:
            out.append((int(r["t1Us"]), int(r["t2Us"]), t3_by_seq[seq], int(r["t4Us"])))
    print(f"clock sync: {len(rows)} pongs logged, {len(out)} complete exchanges "
          f"({from_sd} completed from the drone's SD log)")
    return out


def to_points(exchanges, base_anchor, drone_anchor):
    """(base_mid, drone_mid, round_trip) in us, relative to the anchors."""
    pts = []
    for t1, t2, t3, t4 in exchanges:
        r1, r4 = rel(t1, base_anchor), rel(t4, base_anchor)
        r2, r3 = rel(t2, drone_anchor), rel(t3, drone_anchor)
        rtt = (r4 - r1) - (r3 - r2)
        pts.append(((r1 + r4) / 2, (r2 + r3) / 2, rtt))
    return pts


def reject_slow(points):
    rtts = [p[2] for p in points]
    med = statistics.median(rtts)
    mad = statistics.median(abs(r - med) for r in rtts) or 1.0
    limit = med + max(6 * mad, 50.0)
    kept = [p for p in points if 0 < p[2] <= limit]
    print(f"round trip: median {med:.0f} us, MAD {mad:.0f} us; "
          f"dropped {len(points) - len(kept)} exchanges over {limit:.0f} us")
    return kept


def fit_line(points):
    """Least squares drone = a * base + b. Returns (a, b, rms residual)."""
    n = len(points)
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points)
    a = sxy / sxx if sxx else 1.0
    b = my - a * mx
    rms = (sum((p[1] - (a * p[0] + b)) ** 2 for p in points) / n) ** 0.5
    return a, b, rms


def base_to_drone(t_base, points, window_us, min_points=6):
    """Map one base time onto the drone clock with a local line fit."""
    near = [p for p in points if abs(p[0] - t_base) <= window_us]
    if len(near) < min_points:
        near = sorted(points, key=lambda p: abs(p[0] - t_base))[:min_points]
    a, b, rms = fit_line(near)
    return a * t_base + b, rms, len(near)


def load_gyro(drone_stem, drone_anchor):
    """[(t_us, (x, y, z))] for calibrated gyro samples, and how many were INT-timed."""
    rows = read_csv(f"{drone_stem}_imu.csv")
    out, timed = [], 0
    for r in rows:
        if r["sensor"] != "gyro":
            continue
        out.append((rel(int(r["sampleUs"]), drone_anchor), (float(r["v0"]), float(r["v1"]), float(r["v2"]))))
        timed += r["intTimed"] == "1"
    out.sort()
    return out, timed


def principal_axis(vectors):
    """Dominant direction of a set of 3-vectors (power iteration on the scatter matrix)."""
    c = [[sum(v[i] * v[j] for v in vectors) for j in range(3)] for i in range(3)]
    axis = [1.0, 1.0, 1.0]
    for _ in range(100):
        nxt = [sum(c[i][j] * axis[j] for j in range(3)) for i in range(3)]
        norm = sum(x * x for x in nxt) ** 0.5 or 1.0
        axis = [x / norm for x in nxt]
    return axis


def gyro_peak(samples, center_us, half_period_us):
    """
    Time of peak |rate| near center_us.

    Over most of a half swing the rate keeps one sign and follows a sinusoid,
    so |rate| = c0 + c1 cos(w dt) + c2 sin(w dt) is fitted by least squares
    over +-0.4 of a half period, with w = pi / half period. Using the whole
    hump rather than its flat top makes the peak time far less sensitive to
    gyro noise than fitting a parabola to the top alone.
    """
    import bisect
    import math
    span = 0.4 * half_period_us
    times = [s[0] for s in samples]
    lo = bisect.bisect_left(times, center_us - span)
    hi = bisect.bisect_right(times, center_us + span)
    near = samples[lo:hi]
    if len(near) < 10:
        return None
    w = math.pi / half_period_us
    s = [[0.0] * 3 for _ in range(3)]
    r = [0.0] * 3
    for t, y in near:
        x = w * (t - center_us)
        basis = (1.0, math.cos(x), math.sin(x))
        y = abs(y)
        for i in range(3):
            r[i] += basis[i] * y
            for j in range(3):
                s[i][j] += basis[i] * basis[j]
    c = solve3(s, r)
    if c is None:
        return None
    amplitude = math.hypot(c[1], c[2])
    if amplitude <= 0:
        return None
    shift = math.atan2(c[2], c[1]) / w
    if abs(shift) > span / 2:
        return None  # Peak not inside the window: no clean swing here
    return center_us + shift, c[0] + amplitude


def solve3(m, v):
    m = [row[:] + [v[i]] for i, row in enumerate(m)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda k: abs(m[k][col]))
        if abs(m[piv][col]) < 1e-12:
            return None
        m[col], m[piv] = m[piv], m[col]
        for k in range(3):
            if k != col:
                f = m[k][col] / m[col][col]
                for j in range(col, 4):
                    m[k][j] -= f * m[col][j]
    return [m[i][3] / m[i][i] for i in range(3)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip())
    parser.add_argument("--base", required=True, help="stem of the server's pendulum_*_timesync/_passes CSVs")
    parser.add_argument("--drone", required=True, help="stem of decode_log.py's session_* CSVs")
    parser.add_argument("--window", type=float, default=10.0,
                        help="seconds of sync exchanges either side of a pass to fit (default 10)")
    parser.add_argument("-o", "--out", type=Path, default=Path("pendulum_results.csv"))
    args = parser.parse_args()

    exchanges = load_exchanges(args.base, args.drone)
    if len(exchanges) < 6:
        sys.exit("not enough complete clock-sync exchanges to fit")
    base_anchor, drone_anchor = exchanges[0][0], exchanges[0][1]

    points = reject_slow(to_points(exchanges, base_anchor, drone_anchor))
    a, b, rms = fit_line(points)
    span_s = (points[-1][0] - points[0][0]) / 1e6
    print(f"whole-run line: base clock runs {(1 / a - 1) * 1e6:+.0f} ppm against the drone; "
          f"rms residual {rms:.1f} us over {span_s:.0f} s")

    gyro, timed = load_gyro(args.drone, drone_anchor)
    if gyro:
        print(f"gyro: {len(gyro)} samples, {timed} timed from the INT edge")
    axis = principal_axis([g[1] for g in gyro]) if gyro else None
    rate = [(t, sum(v[i] * axis[i] for i in range(3))) for t, v in gyro] if gyro else []

    passes = read_csv(f"{args.base}_passes.csv")
    centroids = [rel(int(p["centroidUs"]), base_anchor) for p in passes]
    if len(centroids) > 1:
        half_period = statistics.median(b2 - b1 for b1, b2 in zip(centroids, centroids[1:]))
    else:
        half_period = 1e6

    results, diffs, residuals = [], [], []
    for p, c in zip(passes, centroids):
        drone_us, res, n = base_to_drone(c, points, args.window * 1e6)
        residuals.append(res)
        row = {
            "index": p["index"],
            "baseCentroidUs": p["centroidUs"],
            "droneCentroidS": f"{(drone_us) / 1e6:.6f}",
            "syncPoints": n,
            "syncResidualUs": f"{res:.1f}",
            "gyroPeakS": "",
            "gyroPeakRadS": "",
            "gyroMinusMagMs": "",
        }
        peak = gyro_peak(rate, drone_us, half_period) if rate else None
        if peak:
            t_peak, height = peak
            row["gyroPeakS"] = f"{t_peak / 1e6:.6f}"
            row["gyroPeakRadS"] = f"{height:.4f}"
            row["gyroMinusMagMs"] = f"{(t_peak - drone_us) / 1000:.3f}"
            diffs.append((t_peak - drone_us) / 1000)
        results.append(row)

    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()) if results else ["index"])
        w.writeheader()
        w.writerows(results)

    print(f"passes: {len(passes)}; local sync residual median {statistics.median(residuals):.1f} us, "
          f"worst {max(residuals):.1f} us" if residuals else "passes: none")
    if diffs:
        sd = statistics.stdev(diffs) if len(diffs) > 1 else 0.0
        print(f"gyro peak - magnet centroid: mean {statistics.mean(diffs):+.3f} ms, "
              f"std {sd:.3f} ms over {len(diffs)} passes")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
