# Pendulum Timing Test

This test checks the BNO085 gyro's timing. The drone swings on a pendulum and records gyro data to its SD card. Each time it passes the bottom of the swing, a magnet on the drone goes over a Hall-effect sensor on the base station, and the base station records the time. Afterwards a script puts both sets of times on the same clock and compares them for every swing.

Expect the clocks to agree to within tens of microseconds. The test only needs to run for a few minutes.

## What you need

| Item | Notes |
|---|---|
| Drone (Teensy 4.1 + BNO085 + RFM69) | Flashed from the `test/Gyro_Pendulum` branch, with a microSD card fitted (exFAT preferred) |
| BNO085 `INT` wire | From the BNO085 `INT` pin to Teensy **pin 22**. Not in the standard wiring diagram. |
| Small neodymium magnet | Fixed to the drone at the point that passes closest to the sensor |
| Base station: Arduino Uno + RFM69 | Flashed from `AFCDrone_BaseStation` on `feat/gimbal-test-stand` |
| SparkFun Qwiic TMAG5273 | Hall-effect sensor, mounted under the bottom of the swing |
| 3.3 V I2C level shifter | The TMAG5273 is 3.3 V only and the Uno is 5 V |
| Laptop | Running the dashboard server from `AFCDrone_BaseController` on `feat/Watchdog_and_Battery` |
| microSD card reader | To get the log off the drone afterwards |

## 1. Wiring

### Drone

Connect the BNO085 `INT` pin to Teensy **pin 22**. The firmware uses it to time each gyro sample exactly. Without it the test still runs, but gyro times can be up to 2 ms late, which defeats the point of the test.

### Base station

The RFM69 wiring is unchanged (see the `AFCDrone_BaseStation` README). Add the TMAG5273:

| TMAG5273 | Level shifter | Arduino Uno |
|---|---|---|
| `SDA` | LV side ↔ HV side | `A4` |
| `SCL` | LV side ↔ HV side | `A5` |
| `3V3` | `LV` reference | `3.3V` |
| `GND` | `GND` | `GND` |
| | `HV` reference | `5V` |

## 2. Mechanical setup

1. Hang the drone so it swings in one plane.
2. Fix the magnet to the drone where it passes closest to the sensor at the bottom of the swing.
3. Mount the TMAG5273 directly below the drone's resting position, a few centimetres from the magnet's path. Point its **Z axis** (perpendicular to the board) at the magnet. If the sensor has to sit another way, change `MAG_AXIS` at the top of the base station's `src/main.cpp` to the axis that points at the magnet.
4. Make sure nothing else magnetic or steel moves near the sensor during the test.

## 3. Flash the firmware

From each repo, with the board connected over USB:

```bash
# Drone (AFC-Drone, branch test/Gyro_Pendulum)
pio run -e teensy41 -t upload

# Base station (AFCDrone_BaseStation, branch feat/gimbal-test-stand)
pio run -t upload
```

## 4. Start the base station and logging

1. **Keep the magnet away from the sensor**, then plug the Uno into the laptop. For its first few hundred samples it learns the resting field, and a magnet sitting on the sensor would throw that off.
2. Start the dashboard server with pendulum logging turned on:

   ```bash
   cd AFCDrone_BaseController
   ./start-dashboard.sh --serial-port /dev/ttyACM0 --pendulum-log test_logs
   ```

   On Windows use `start-dashboard.bat` and the Uno's `COM` port.
3. Open the dashboard at http://localhost:8000 and check the debug terminal for:
   - `TMAG5273 at 0x22, +-80 mT, axis Z`: the sensor was found. If it says `TMAG5273 not found`, check the I2C wiring and the level shifter.
   - `MAG <field> uT base <baseline> uT <rate> Hz passes <n>`, once a second. The rate should be around 4000–5000 Hz.
4. The server prints `Pendulum log: test_logs/pendulum_<date>_<time>_*.csv`. That is the base-station log for this run.

## 5. Start the drone

1. Power the drone. It starts SD logging on its own at boot. The dashboard's SD log panel should show **Logging**. If it shows **No card**, fix the card before going on.
2. Wait for `RF link up` in the debug terminal. Clock sync starts automatically once the drone is linked.
3. Leave the drone in **SAFE**. The test doesn't need it armed, and the motors must not run.

## 6. Tune the pass detector

1. Watch the `MAG` line while you swing the drone gently by hand over the sensor. Note how far the field moves from `base` at the bottom of the swing.
2. Set `PASS_ENTER_UT` in the base station's `src/main.cpp` to about **half** of that change, and `PASS_EXIT_UT` to about half of `PASS_ENTER_UT`. The defaults are 2000 µT and 1000 µT.
3. If you changed them, re-flash the Uno (keeping the magnet away while it boots) and reconnect the dashboard. Reconnecting starts a new set of log files.
4. Check that `passes <n>` goes up by exactly one for every pass over the sensor.

## 7. Run the test

1. Use the dashboard's SD log panel to drop a **mark** (for example `run1`) so the run is easy to find in the drone's log.
2. Release the drone from a consistent angle and let it swing freely for **2–5 minutes**.
3. Watch for:
   - `passes` going up once per swing past the bottom
   - the drone staying linked (no `RF link lost`)
   - no `MAG pass too long` messages. That message means the drone came to rest over the sensor, or the baseline drifted.
4. Stop the swing, then press **Stop Log** in the SD log panel so the log file is closed cleanly.
5. Stop the dashboard server (Ctrl+C) and power down the drone.

## 8. Analyze the results

1. Copy `LOGnnn.BIN` from the drone's SD card. The newest file number is the latest run.
2. Decode it:

   ```bash
   cd AFC-Drone
   python3 scripts/decode_log.py /path/to/LOGnnn.BIN -o decoded/
   ```

   This prints the session ID and writes `decoded/session_<id>_imu.csv`, `_timesync.csv` and the usual telemetry files. In `_imu.csv`, check that the `intTimed` column is `1` for the gyro rows. A `0` means the INT wire isn't working.
3. Combine the two logs:

   ```bash
   python3 scripts/pendulum_sync.py \
       --base  ../AFCDrone_BaseController/test_logs/pendulum_<date>_<time> \
       --drone decoded/session_<id> \
       -o pendulum_results.csv
   ```

   Pass the file names without the `_timesync.csv` / `_passes.csv` / `_imu.csv` endings.

### Reading the output

The script prints a summary like this:

```text
clock sync: 1073 pongs logged, 1073 complete exchanges (1 completed from the drone's SD log)
round trip: median 2226 us, MAD 10 us; dropped 73 exchanges over 2286 us
whole-run line: base clock runs +3021 ppm against the drone; rms residual 219.3 us over 240 s
gyro: 96001 samples, 96001 timed from the INT edge
passes: 265; local sync residual median 4.1 us, worst 8.9 us
gyro peak - magnet centroid: mean -0.002 ms, std 0.065 ms over 265 passes
```

| Line | What to look for |
|---|---|
| `clock sync` | Several hundred complete exchanges for a few-minute run (five per second) |
| `round trip` | A small spread (MAD of tens of µs). A few percent of exchanges dropped is normal. |
| `whole-run line` | The Uno's clock error. A few thousand ppm is normal for an Uno. The rms residual here is large on purpose: it shows why a single fixed offset isn't used. |
| `local sync residual` | **The sync error.** It should be tens of µs or less, well under the 1 ms target. |
| `gyro peak - magnet centroid` | **The result**: how far the gyro's bottom-of-swing time is from the magnet's, as a mean and spread over all passes |

`pendulum_results.csv` has one row per pass:

| Column | Meaning |
|---|---|
| `index` | Pass number from the base station |
| `baseCentroidUs` | Pass time on the Uno's clock |
| `droneCentroidS` | The same moment on the drone's clock, in seconds from the first sync |
| `syncPoints` / `syncResidualUs` | How many sync exchanges the local clock fit used, and how well they fit |
| `gyroPeakS` | When the gyro's swing rate peaked (the bottom, according to the gyro) |
| `gyroPeakRadS` | Peak angular rate, rad/s |
| `gyroMinusMagMs` | Gyro time minus magnet time, in ms |

## Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| `TMAG5273 not found` | I2C wiring, missing level shifter, or no 3.3 V to the sensor |
| `MAG` rate well below 4000 Hz | I2C bus problem: long wires or missing pull-ups on the 3.3 V side |
| `passes` doesn't count, or counts twice per swing | `PASS_ENTER_UT` is too high (no count) or too low (double count); see step 6. The axis may also be wrong; check `MAG_AXIS`. |
| `MAG pass too long - relearning baseline` | The drone stopped over the sensor, or the magnet was near the sensor when the Uno booted |
| No `_timesync.csv` rows | The drone isn't linked (`RF link up` never appeared), or the server wasn't started with `--pendulum-log` |
| `not enough complete clock-sync exchanges to fit` | The run was too short, or the radio link kept dropping. Move the antennas closer. |
| `intTimed` is `0` in `_imu.csv` | BNO085 `INT` isn't reaching Teensy pin 22 |
| Gyro columns empty in `pendulum_results.csv` | The drone's log doesn't cover that pass. Check that logging was on, and that you decoded the right `LOGnnn.BIN`. |
| Large `local sync residual` (hundreds of µs) | Too few exchanges near that pass, which usually means radio dropouts. Try `--window 20`. |

## How it works

- **Clock sync.** Five times a second the base station sends a time ping and the drone answers. Both ends stamp the exact moment their radio chip reports a packet sent or received, using an interrupt. Ping and reply take the same path in opposite directions, so the radio's own delays cancel out. Protocol details are in the [Dashboard Protocol Reference](dashboard-protocol.md#rfm69-clock-sync).
- **Fitting the clocks afterwards.** The Uno's clock can be off by up to about 0.5% and drifts with temperature, so it isn't corrected live. `pendulum_sync.py` fits a straight line between the two clocks over the 10 s of sync data around each pass.
- **Magnet pass time.** The base station takes the time-weighted centre of the magnetic-field bump. For a field symmetric about the bottom of the swing, that is the bottom, whatever the threshold.
- **Gyro sample times.** The BNO085 reports how old each sample was when its `INT` line went low. The drone stamps that falling edge, so every gyro sample gets the time it was actually measured. The bottom of the swing is where the angular rate peaks; the script fits a sine curve around each pass to find that peak.
- **Limit.** Times are 32-bit microsecond counters, unwrapped relative to the first sync exchange, so keep a single run under about 35 minutes.
