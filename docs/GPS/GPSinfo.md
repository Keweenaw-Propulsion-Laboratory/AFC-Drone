## GPS Module Use Case

The GPS module provides positioning for the drone's **LG290P rover receiver**.
It configures the receiver for 20 Hz measurements, reads its position reports,
and keeps the latest latitude, longitude, ellipsoid height, fix quality, and
satellite count available to flight software. When the ground base antenna's
position is supplied, it also calculates north/east/up offsets from that base.

The implementation lives in [`src/GPS.cpp`](../../src/GPS.cpp), with its public
interface in [`include/GPS.h`](../../include/GPS.h). It uses SparkFun's LG290P
GNSS Arduino Library; [`platformio.ini`](../../platformio.ini) currently pins
version **v3.1.1**.

`GPS::update()` is intended to run every pass through `loop()` and cache complete
reports. `GPS::getPosition()` copies the cache without requesting another
measurement, so a faster control loop can read the same report repeatedly.
Coordinates stay as `double` values to preserve the receiver's position precision.

> [!IMPORTANT]
> **GPS currently defaults to disabled and is not integrated into flight software.**
> Nothing calls `GPS::configure()`, GPS setup is commented out in
> [`src/drone.cpp`](../../src/drone.cpp), and
> [`src/main.cpp`](../../src/main.cpp) does not call `GPS::update()`.
> While disabled, `setup()` and `setupComplete()` return true, `update()` does
> nothing, `getPosition()` returns false, and `sendCorrections()` returns zero.
> USB latitude and longitude are still fixed test values, not GPS measurements.

## Hardware and Receiver Roles

The drone carries the **rover** receiver. A separate, stationary ground receiver
must be configured as the **base**, using either survey-in or a known antenna
position. This module configures the rover only; ground receiver configuration
and delivery of its corrections belong to the surrounding system.

The MCU connection uses a dedicated hardware UART:

```text
Flight computer                 LG290P rover
MCU RX <----------------------- GNSS TX
MCU TX -----------------------> GNSS RX
GND --------------------------- GND
```

| Setting | Requirement |
| :--- | :--- |
| MCU port | A dedicated `HardwareSerial` object that remains alive for the program's lifetime |
| Receiver port | GNSS-side UART number **1, 2, or 3**; independent of the MCU serial port number |
| Baud rate | The receiver's current baud rate, from **9600 to 3000000 bits/s** |
| Default baud argument | **460800 bits/s** |
| Incoming protocols on the selected GNSS port | NMEA and RTCM3 |
| Outgoing protocol on the selected GNSS port | NMEA |

`GPS::configure(serial, uart, baud)` records the final wiring and enables setup.
It rejects an invalid port, an out-of-range baud rate, or a second configuration
after the first successful call. It does not open the MCU UART until `setup()`
runs, and it does not change the receiver's baud rate or automatically discover it.
The supplied rate must already match the receiver.

RTCM3 corrections can reach the rover through `GPS::sendCorrections()` on this
connection, or through a separate GNSS UART link configured outside this module.
The software does not assign MCU UART pins or select a final serial port for the
application.

## Rover Startup

Call `GPS::configure()` during startup, then keep calling `GPS::setup()` until
`GPS::setupComplete()` is true. A true return from `setup()` means that the current
step succeeded, setup is already complete, or GPS is disabled; it does not by
itself mean that all startup steps have finished.

| Stage | What the module does |
| :--- | :--- |
| `DISABLED` | Accepts startup without accessing hardware until `configure()` enables GPS |
| `CONNECT` | Opens the MCU UART at the configured baud rate and connects through the LG290P library |
| `CONFIGURE` | Ensures rover mode, requests a **50 ms fix interval**, selects flight navigation mode **5**, configures port protocols, enables GGA every fix, and disables PQTMPVT output |
| `STREAM` | Disables one unused message per setup call: `RMC`, `GSV`, `GSA`, `VTG`, `GLL`, `ZDA`, and `GST` |
| `SAVE` | Saves the receiver configuration |
| `RESTART` | Resets the receiver and subscribes to GGA position reports |
| `COMPLETE` | Allows position processing and correction forwarding |
| `FAILED` | Returns false on later setup calls; no automatic retry is implemented |

**Setup completion is receiver configuration, not a position lock.** It does not
wait for satellites, an ordinary fix, or an RTK fixed solution. Flight software
must assess the reports separately before using the position.

Although startup advances through stages, LG290P library commands can block for
seconds while connecting or restarting. Run setup during startup, outside an
interrupt handler.

If any active setup step fails, the module enters `FAILED` and reports this text
through `USB::sendText()`:

```text
DRONE: GPS setup failed. Check wiring, baud rate and receiver firmware
```

There is no public reset, retry, or disable function. Once setup has failed,
calling `setup()` again keeps returning false, and `configure()` cannot replace
the configuration. The module also does not change the drone's flight state;
startup code must decide how to handle the failure.

## Position Reports

`GPS::Position` describes the **rover antenna**, using one GGA report for all
measurement fields. It does not compensate for the distance between the antenna
and the drone's center of mass.

| Field | Units / values | Meaning |
| :--- | :--- | :--- |
| `latitude` | `double`, degrees, -90 to +90 | WGS84 latitude; north is positive, south is negative |
| `longitude` | `double`, degrees, -180 to +180 | WGS84 longitude; east is positive, west is negative |
| `altitudeMeters` | `double`, meters | Height above the WGS84 ellipsoid |
| `northMeters` | `double`, meters | Northward offset from the configured base antenna |
| `eastMeters` | `double`, meters | Eastward offset from the configured base antenna |
| `upMeters` | `double`, meters | Upward offset in the base antenna's local frame |
| `fix` | `GPS::FixQuality` | GGA fix classification |
| `satellites` | `uint8_t` | Number of satellites used in the fix |
| `ageMs` | `uint32_t`, milliseconds | Time since the MCU received the cached report; `UINT32_MAX` before any report |
| `relativeValid` | `bool` | Whether the offsets have a base reference and are usable with this report |

Before a report is received, coordinate and offset fields are zero, `fix` is
`NO_FIX`, `satellites` is zero, and `relativeValid` is false. Zero coordinates
alone do not identify a missing position; check the result of `getPosition()`.

### Fix Quality

| GGA value | `GPS::FixQuality` | Meaning |
| :--- | :--- | :--- |
| 0 | `NO_FIX` | No usable fix; also used when report fields fail validation |
| 1 | `STANDALONE` | Standalone satellite fix |
| 2 | `DIFFERENTIAL` | Differential fix |
| 3 | `PRECISE` | Precise fix classification |
| 4 | `RTK_FIXED` | RTK solution with resolved ambiguities |
| 5 | `RTK_FLOAT` | RTK solution with unresolved ambiguities |

`getPosition()` accepts a fresh fix in any supported class from **1 through 5**.
For an application that requires RTK precision, also require
`position.fix == GPS::FixQuality::RTK_FIXED`. Neither the name `PRECISE` nor a
true return from `getPosition()` establishes an RTK fixed solution.

### Report Validation and Height

The GGA callback ignores packets with an invalid checksum or fewer than 15
fields. For a packet that passes those checks, it validates the fix quality,
satellite count, coordinates, hemisphere indicators, altitude, geoid separation,
and meter unit markers before accepting the position.

Latitude and longitude are converted from NMEA degrees and minutes to decimal
degrees. Minutes must be below 60, and the resulting angles must stay within
their latitude/longitude limits. Numeric parsing rejects missing or malformed
values, non-finite values, overflow, whitespace, and exponent or hexadecimal
notation. Satellite counts must fit in a `uint8_t`; there is no additional
minimum-satellite requirement in this module.

GGA supplies height above mean sea level and geoid separation separately. The
module combines them as:

```text
altitudeMeters = GGA altitude + GGA geoid separation
```

The result is **ellipsoid height**, which is also the height convention required
by `setBasePosition()`. Comparing it directly with a sea-level altitude mixes
two different references.

A checksum-valid packet with enough fields but unusable fix data replaces the
cache with a default `NO_FIX` report and updates its receipt time. A packet that
fails the initial checksum/field-count checks leaves the previous cache intact;
that older report eventually becomes stale.

### Update Rate and Freshness

The requested receiver interval is **50 ms**, or **20 reports per second**.
`GPS::update()` processes at most **512 UART bytes per call** and stops sooner
when no bytes are available. It does not wait for a complete report or a new
fix, and it does nothing until setup is complete.

Call it every pass through `loop()` so reports do not accumulate in the UART
buffer. The 512-byte limit bounds byte processing per call, not execution time.
Reading the cached position at the 1 kHz control rate still provides only the
measurements that have arrived from the receiver.

`GPS::getPosition(out)` always copies the cached report and calculates its current
age. It returns true only when a report has been received, its fix is not
`NO_FIX`, and its age is **500 ms or less**. At more than 500 ms it returns false
and clears `out.relativeValid`, while leaving the copied measurement fields
available. Always check the return value before treating those fields as current.

`ageMs` measures time since **local receipt**, not the satellite measurement's
timestamp. It does not account for receiver or UART backlog, and the public
report does not expose a GNSS timestamp, an accuracy estimate, or correction age.

## Position Relative to the Ground Base

Call `GPS::setBasePosition(latitude, longitude, altitudeMeters)` with the same
WGS84 reference position used by the ground receiver. Latitude and longitude
must be within their normal ranges and all three inputs must be finite.
Invalid input returns false without changing the existing reference.

The module converts the base and rover positions to Earth-centered,
Earth-fixed (ECEF) coordinates, subtracts the base position, then rotates the
difference into the base antenna's local north/east/up frame:

| Offset | Positive direction | Reference |
| :--- | :--- | :--- |
| `northMeters` | North | Ground base antenna |
| `eastMeters` | East | Ground base antenna |
| `upMeters` | Up | Ground base antenna's local tangent frame |

After setting or changing the base, the cached `relativeValid` flag is cleared.
Offsets become valid on the **next valid rover report**, rather than being
recomputed immediately from the old cache. Without a base reference, absolute
coordinates can still be valid, but relative offsets remain unavailable.

**A base reference does not establish an RTK fix.** It enables the coordinate
calculation even for a standalone fix. `relativeValid` describes availability of
the offsets; `fix` describes the receiver's solution quality. Require a true
`getPosition()` result, `relativeValid`, and `RTK_FIXED` together when the caller
needs current relative positioning from an RTK fixed solution.

`setBasePosition()` only sets the software's local reference. It does not
configure the ground receiver, start survey-in, or deliver corrections. The
module has no public method to clear a base reference once set.

## RTK Correction Forwarding

`GPS::sendCorrections(data, length)` forwards raw RTCM3 bytes from the ground base
to the rover's UART. Preserve their original order and framing. The caller must
provide the transport from the base to the drone; this module does not receive
corrections from the radio or USB by itself.

The function checks the UART's available transmit space and writes only as many
bytes as fit. Its return value is the number queued, which can be less than
`length`. Keep the unqueued remainder and retry it on later loop passes before
sending newer bytes. The module does not retain that remainder for the caller.

It returns zero if GPS is disabled, setup is incomplete or failed, the data
pointer is null, the length is zero, or there is no transmit space. It does not
wait for space, decode or validate RTCM3 frames, or confirm that the receiver
used the corrections. Bytes queued are not an acknowledgement of an RTK solution;
inspect subsequent position reports for fix quality.

## Public Interface and Call Context

| Function | Purpose | Intended call context |
| :--- | :--- | :--- |
| `configure(serial, uart, baud)` | Enables GPS with the final serial connection | Startup only, before active GPS setup |
| `setup()` | Advances receiver initialization; false means setup failure | Repeated calls during startup |
| `setupComplete()` | Checks whether GPS is disabled or receiver setup has finished | Startup completion checks |
| `update()` | Processes available receive bytes and publishes reports | Every pass through `loop()` |
| `getPosition(out)` | Copies the latest report and checks freshness/fix validity | `loop()` or a control interrupt |
| `setBasePosition(lat, lon, height)` | Supplies the base antenna reference | Startup or `loop()` |
| `sendCorrections(data, length)` | Queues as many correction bytes as currently fit | `loop()` |

Position publication and copying briefly mask interrupts so the control ISR
cannot read partly updated fields or a torn `double`. The code preserves the
caller's prior interrupt state when leaving that protected section.
`getPosition()` performs no serial I/O and is the supported position read from
a control ISR. Receiver setup, UART processing, base changes, and correction
writes belong outside interrupt handlers.

## Integration

| Where | Current state | Work needed to use GPS |
| :--- | :--- | :--- |
| Startup wiring selection | No call to `GPS::configure()` | Supply the dedicated MCU serial port, GNSS-side UART number, and matching baud rate before GPS setup |
| [`src/drone.cpp`](../../src/drone.cpp) | GPS setup is commented out in `SENSOR_SETUP`; completion checks only the gyro | Include the GPS header, call `GPS::setup()` repeatedly, handle failure, and include `GPS::setupComplete()` in stage completion |
| [`src/main.cpp`](../../src/main.cpp) | No GPS servicing in `loop()` | Include the GPS header and call `GPS::update()` every pass |
| Ground base and correction transport | No calls to `setBasePosition()` or `sendCorrections()` | Configure the base separately, supply its reference position when offsets are needed, and deliver corrections through a transport or separate GNSS link |
| Flight control | No calls to `GPS::getPosition()` | Read and check the cached position wherever navigation needs it |
| [`src/usb.cpp`](../../src/usb.cpp) | Latitude and longitude are hard-coded test coordinates | Populate live measurements and define how validity, age, and fix quality reach the ground station |
| [`include/radio.h`](../../include/radio.h), [`src/radio.cpp`](../../src/radio.cpp) | An unused `StatusMsg7_t` contains float latitude/longitude; no GPS message type, union member, or sender is connected | Define and implement the GPS telemetry path |

The existing `posX`, `posY`, and `posZ` telemetry comes from gyro dead reckoning,
not from the GPS north/east/up offsets. This module does not fuse those estimates,
calculate velocity or heading, navigate to waypoints, or command the motors and
gimbal. It supplies position data for other code to use.

Both the existing USB coordinate fields and the unused radio coordinate struct
use `float`, while `GPS::Position` uses `double`. Any telemetry integration that
needs to preserve the module's coordinate precision must account for that
conversion and define the coordinate frame and units for relative positions.

Runtime loss of fix or stale data is reported through `getPosition()` returning
false. The module does not automatically reconnect, restart the receiver, alert
the ground station, or trigger a flight failsafe. Those responses, along with
any stricter age or fix-quality requirements, belong to the consuming software.
