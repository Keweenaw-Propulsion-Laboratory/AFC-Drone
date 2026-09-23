# AFC Drone Dashboard Protocol Reference

This document describes the protocol currently implemented by the flight
firmware. It is intended for a dashboard client that connects over USB and,
optionally, observes or communicates through the RFM69 radio.

## Implementation status

| Interface | Direction | Endpoint | Status |
| --- | --- | --- | --- |
| USB | Dashboard → drone | `RAW` identify | Implemented; the drone answers the identify query so a client can tell a direct-wired drone from a base-station relay. |
| USB | Dashboard → drone | `COMMAND` | Implemented; no acknowledgement is sent. |
| USB | Dashboard → drone | `CONFIG` `SET` / `READ` | Implemented. |
| USB | Dashboard → drone | `HEARTBEAT` | Implemented; feeds the link watchdog and carries the requested state. |
| USB | Drone → dashboard | `TELEMETRY` | Implemented at approximately 10 Hz; `voltage` now carries a live smoothed pack reading. Record is 64 bytes as of the watchdog field. |
| USB | Drone → dashboard | `DEBUG_TEXT` | Implemented. |
| USB | Drone → dashboard | `RADIO_PACKET` relay | Implemented for sent and received RFM69 packets. |
| RFM69 | Ground station → drone | `COMMAND` | Implemented. |
| RFM69 | Ground station → drone | `HEARTBEAT` | Implemented; feeds the link watchdog and carries the requested state. |
| RFM69 | Drone → ground station | `STATUS0`–`STATUS6` | Implemented at approximately 10 Hz; types 3–6 (quaternion, acceleration, velocity, position) now carry live Gyro data. |
| RFM69 | Ground station → drone | `CONFIG` | Partially implemented; see [Radio configuration limitations](#radio-configuration-limitations). |

All multi-byte integers and IEEE-754 `float` values are little-endian. All
structures are packed: do not insert alignment padding in the dashboard
decoder.

## USB transport

USB uses a byte stream with this frame layout:

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 1 | Sync `0xA5` |
| 1 | 1 | Sync `0x5A` |
| 2 | 1 | Protocol version (`1`) |
| 3 | 2 | Packet sequence number, `uint16` little-endian |
| 5 | 1 | Message type |
| 6 | 1 | Payload length, `0`–`64` |
| 7 | `length` | Payload |
| 7 + `length` | 2 | CRC-16/CCITT-FALSE, little-endian |

The CRC covers the four-byte packet header (sequence number, type, length)
followed by the payload. It does **not** cover the two sync bytes or the
protocol-version byte. CRC parameters are polynomial `0x1021`, initial value
`0xFFFF`, no reflection, and no final XOR.

The drone emits its own incrementing sequence number. A dashboard should use
it to detect dropped USB frames, but it must not assume inbound sequence
numbers are validated; the firmware currently does not enforce them.

### USB message types

| Value | Name | Dashboard direction | Notes |
| ---: | --- | --- | --- |
| 0 | `RAW` | Both | Device identify handshake, 1 payload byte each way. See [`RAW` identify handshake](#raw-identify-handshake-type-0). |
| 1 | `DEBUG_TEXT` | Drone → dashboard | UTF-8/ASCII text bytes; no terminating NUL. |
| 2 | `RADIO_PACKET` | Drone → dashboard | RFM69 packet mirror. |
| 3 | `TELEMETRY` | Drone → dashboard | 64-byte combined telemetry record. |
| 4 | `COMMAND` | Dashboard → drone | Exactly 8 payload bytes. |
| 5 | `CONFIG` | Both | Configuration requests and responses. |
| 10 | `HEARTBEAT` | Dashboard → drone | Exactly 8 payload bytes. See [Link watchdog and heartbeat](#link-watchdog-and-heartbeat). |

The USB receive parser only accepts inbound `COMMAND`, `CONFIG`, `HEARTBEAT`,
and `RAW` frames; other type values are discarded before their payload is read,
and an accepted `RAW` frame is then dropped by the dispatcher. A `HEARTBEAT`
frame whose payload length is not exactly 8 is rejected by the header check and
never reaches the watchdog, so a malformed heartbeat will let the link time
out rather than reporting an error.

### `COMMAND` request (type 4)

Payload length must be exactly 8 bytes.

| Offset | Type | Field | Meaning |
| ---: | --- | --- | --- |
| 0 | `uint8` | flags | Bit 0: target slot to update. Bit 1: active target slot. Bits 2–7: zero/reserved. |
| 1 | `int16` | `gimbalX` | Normalized gimbal X command. The flight loop divides by `1638.0`, yielding approximately ±20°. |
| 3 | `int16` | `gimbalY` | Normalized gimbal Y command. |
| 5 | `uint8` | `motor0Speed` | Motor command, nominal range 0–255. |
| 6 | `uint8` | `motor1Speed` | Motor command, nominal range 0–255. |
| 7 | `uint8` | reserved | Send zero. |

The command updates target slot 0 or 1 and then selects the active slot from
flag bit 1. It has no USB acknowledgement and no command timeout. The
dashboard should display that command delivery is not confirmed.

**Current firmware caveat:** the `COMMAND` switch case falls through to the
`CONFIG` handler after applying the command. This is a firmware defect and may
produce a spurious configuration response. A dashboard should tolerate an
unexpected type-5 response after sending a command.

### `TELEMETRY` event (type 3)

The drone sends this 64-byte record every 100 ms while the main loop runs.

**Breaking change:** this record grew from 54 to 64 bytes when the watchdog
field was added. Existing field offsets are unchanged, so a decoder that reads
by offset and ignores trailing bytes keeps working, but any decoder that
asserts a 54-byte payload length must be updated.

| Offset | Type | Field | Dashboard meaning | Current source |
| ---: | --- | --- | --- | --- |
| 0 | `uint16` | `loopTimeAvg` | Average loop cost, µs | Measured |
| 2 | `uint16` | `loopTimeMax` | Worst observed loop cost, µs | Measured since boot |
| 4 | `uint16` | `runTime` | Power-on time, seconds | Measured; wraps at 65535 s |
| 6 | `uint8` | `rssi` | RFM69 RSSI | Narrowed from signed RSSI; interpret with care |
| 7 | `uint8` | `currentMode` | Drone state | See [Drone states](#drone-states) |
| 8 | `int16` | `gimbalPitch` | Gimbal pitch | Live value |
| 10 | `int16` | `gimbalYaw` | Gimbal yaw | Live value |
| 12 | `int16` | `topServoSet` | Top servo setpoint | Live value |
| 14 | `int16` | `bottomServoSet` | Bottom servo setpoint | Live value |
| 16 | `uint8` | `motor1Set` | Bottom motor output | What was actually commanded to the ESC. Reads `0` whenever output is suppressed - see [Motor setpoint reporting](#motor-setpoint-reporting) |
| 17 | `uint8` | `motor2Set` | Top motor output | As above |
| 18 | `uint16` | `voltage` | Battery voltage | Smoothed pack voltage from `Battery::getVoltage()`, fixed-point ×1000 (V → mV). Divide by 1000 to display volts. Clamped to 0–65535, so the field cannot wrap on a bad reading. See [Battery Monitor](battery/batteryinfo.md) |
| 20 | `int16` × 4 | `qR`, `qI`, `qJ`, `qK` | Quaternion | Drone-body-frame orientation (remapped from the raw BNO08x mounting axes), fixed-point ×32767 (component range −1.0–1.0) |
| 28 | `int16` × 3 | `accelX`, `accelY`, `accelZ` | Acceleration | Gyro world-frame linear acceleration, fixed-point ×1000 (m/s² → mm/s²) |
| 34 | `int16` × 3 | `velX`, `velY`, `velZ` | Velocity | Gyro dead-reckoned velocity, fixed-point ×1000 (m/s → mm/s) |
| 40 | `int16` × 3 | `posX`, `posY`, `posZ` | Position | Gyro dead-reckoned position, fixed-point ×100 (m → cm) |
| 46 | `float` | `latitude` | Latitude | Fixed test value currently |
| 50 | `float` | `longitude` | Longitude | Fixed test value currently |
| 54 | `uint8` | `watchDog` | Link watchdog flags | Live; see [Link watchdog and heartbeat](#link-watchdog-and-heartbeat) |
| 55 | 9 bytes | reserved | Unused | Transmitted as zero; ignore |

### `RAW` identify handshake (type 0)

A dashboard that opens a serial port cannot tell from the port alone whether it
has reached a drone directly or a base station relaying for one. The identify
handshake answers that without asking the user to pick a mode.

Send a `RAW` frame with exactly one payload byte, `0x3F`. A drone replies with a
`RAW` frame of one byte, `0x01` (`DEVICE_KIND_DRONE_DIRECT`). A base station
answers the same query with its own device-kind byte.

Any other `RAW` payload is ignored, and no reply is sent. Send the query on every
fresh connection; the firmware keeps no per-client state.

### `DEBUG_TEXT` event (type 1)

Payload is an arbitrary text fragment of 1–64 bytes. Messages longer than 64
bytes are split into independent frames. Treat payload as display text, not as
a machine-stable event API.

### `RADIO_PACKET` event (type 2)

This is emitted for every radio packet sent or received while USB radio relay
is enabled in the firmware. Payload length is exactly 11 bytes:

| Offset | Type | Field |
| ---: | --- | --- |
| 0 | `uint8` | Direction: `0` received by drone, `1` sent by drone |
| 1 | `uint8` | RFM69 packet number (`headerId`) |
| 2 | `uint8` | RFM69 message type (`headerFlags`) |
| 3–10 | 8 raw bytes | RFM69 message payload |

Decode bytes 3–10 with the RFM69 message tables below. This endpoint is the
best way for a USB dashboard to observe radio traffic without a separate radio
receiver.

### USB configuration (type 5)

All configuration requests begin with:

| Offset | Type | Field |
| ---: | --- | --- |
| 0 | `uint8` | Configuration format version (`3`) |
| 1 | `uint8` | Operation: `READ = 1`, `SET = 2` |
| 2 | `uint8` | Number of entries |

Each request entry is six bytes: `ConfigKey` (`uint16` little-endian) followed
by `value` (`int32` little-endian). A `READ` request ignores the value field.

`SET` accepts 1–9 entries and has a request length of `3 + 6 × count`. All
valid values are applied, and EEPROM is written at most once. Invalid entries
do not prevent other valid entries from applying.

`SET_RESPONSE` (`operation = 0x82`) begins with `version`, operation, count,
and a request-level `ConfigResult`; it then contains one 3-byte `{key,
result}` record per requested entry.

`READ` accepts 1–8 entries because its response contains values. `READ_RESPONSE`
(`operation = 0x81`) begins with the same four-byte response header and then
contains one 7-byte `{key, result, value}` record per requested entry.

Configuration keys and values:

| Key value | Name | Accepted set value | Default |
| ---: | --- | --- | ---: |
| 0 | `DebugMode` | 0 or 1 | 0 |
| 1 | `TxPowerDbm` | 14–20 | 20 |
| 2 | `UsbRelayEnabled` | 0 or 1 | 1 |
| 3 | `RadioEnabled` | 0 or 1 | 1 |
| 4 | `SkipRadioHandshake` | 0 or 1 | 1 |
| 5 | `GimbalPitchOffset` | 60–120 | 90 |
| 6 | `GimbalYawOffset` | 60–120 | 89 |
| 7 | `Motor1Offset` | −100–100 | 0 |
| 8 | `Motor2Offset` | −100–100 | 0 |
| 9 | `GyroEnabled` | 0 or 1 | 1 (`0` on the Nucleo bench build) |

`DebugMode` is accepted and persisted but does not yet gate any behavior.

`RadioEnabled` and `GyroEnabled` decide whether the boot sequence talks to the
RFM69 and the BNO08x at all. Setting either to `0` lets the vehicle reach `SAFE`
with that peripheral absent, which is how the Nucleo bench target boots; the
corresponding telemetry then reports stale zeros rather than live data. Both
default to `1` on the flight build.

`ConfigResult` values are: `OK = 0`, `INVALID_VALUE = 1`, `INVALID_KEY = 2`,
`UNSAFE_STATE = 3`, `UNKNOWN_VERSION = 4`, and `UNKNOWN_OP = 5`. Changes are
rejected with `UNSAFE_STATE` while the drone is in `MAN_FLIGHT` or
`AUTO_FLIGHT`.

## RFM69 radio protocol

The RFM69 payload is always exactly eight bytes. RadioHead headers are outside
that payload:

| RadioHead header | Meaning |
| --- | --- |
| `headerId` | Packet sequence number, incremented by the drone for transmissions |
| `headerFlags` | AFC message type |

The firmware currently configures the radio at 915 MHz, uses the RFM69
high-power mode with the persisted 14–20 dBm setting, and uses the configured
16-byte encryption key. The dashboard should not expose the key as a normal
user-facing setting.

### RFM69 message types

| Value | Name | Payload |
| ---: | --- | --- |
| 0 | `SETUP` | Eight text bytes; drone sends `AFCDrone` during handshake. Inbound setup is ignored. |
| 1 | `STATUS0` | System status record |
| 2 | `STATUS1` | Gimbal/servo status record |
| 3 | `STATUS2` | Motor/battery status record |
| 4 | `STATUS3` | Quaternion record |
| 5 | `STATUS4` | Acceleration record |
| 6 | `STATUS5` | Velocity record |
| 7 | `STATUS6` | Position record |
| 8 | `COMMAND` | Same 8-byte command layout as USB |
| 9 | `CONFIG` | One configuration request/response |
| 10 | `HEARTBEAT` | Link keepalive and requested state. See [Link watchdog and heartbeat](#link-watchdog-and-heartbeat). |

The flight loop queues status types 1–7 every 100 ms. The radio transmitter
sends no more than one queued packet per 10 ms receive window, so individual
status packets can arrive later than the 100 ms telemetry tick.

### RFM69 status payloads

| Type | Byte layout | Current behavior |
| --- | --- | --- |
| `STATUS0` | `uint16 loopTimeAvg`, `uint16 loopTimeMax`, `uint16 runTime`, `uint8 currentMode`, `uint8 watchDog` | Live fields. `currentMode` is at offset 6, `watchDog` at offset 7; RSSI moved to `STATUS2`. See [Link watchdog and heartbeat](#link-watchdog-and-heartbeat) for the `watchDog` bits. |
| `STATUS1` | `int16 gimbalPitchNorm`, `int16 gimbalYawNorm`, `uint16 topServoSet`, `uint16 bottomServoSet` | Live fields. |
| `STATUS2` | `uint16 motor1set`, `uint16 motor2set`, `uint16 voltage`, `uint16 rssi` | All live. Motor values follow the same suppressed-reads-zero rule as the USB record - see [Motor setpoint reporting](#motor-setpoint-reporting). `voltage` is the smoothed pack voltage in millivolts (V ×1000), clamped to 0–65535. `rssi` carries the signed RFM69 RSSI in a `uint16` - reinterpret as `int16`. |
| `STATUS3` | Four `int16` quaternion fields: `qR`, `qI`, `qJ`, `qK` | Drone-body-frame orientation, fixed-point ×32767. |
| `STATUS4` | Three `int16` acceleration fields plus `int16 reserved` | Gyro world-frame acceleration, fixed-point ×1000 (mm/s²). |
| `STATUS5` | Three `int16` velocity fields plus `int16 reserved` | Gyro dead-reckoned velocity, fixed-point ×1000 (mm/s). |
| `STATUS6` | Three `int16` position fields plus `int16 reserved` | Gyro dead-reckoned position, fixed-point ×100 (cm). |

### RFM69 command (type 8)

This is byte-for-byte the same as the USB `COMMAND` payload. It updates a
target slot and the active target slot. There is no acknowledgement, timeout,
or command authorization check.

### RFM69 configuration (type 9)

The 8-byte layout is:

| Offset | Type | Field |
| ---: | --- | --- |
| 0 | `uint8` | Config format version (`3`) |
| 1 | `uint8` | Request operation (`READ = 1`, `SET = 2`) or response result |
| 2 | `uint16` | `ConfigKey`, little-endian |
| 4 | `uint32` | Value, little-endian; interpret as signed for configuration values that allow negatives |

Only one key can be transported per radio message. Use the USB batch endpoint
when setting multiple values.

### Radio configuration limitations

The current `radio_handleConfig()` implementation should not be treated as a
reliable control endpoint yet:

- A `READ` response copies the returned value but does not copy an invalid-key
  result into the response status byte.
- A `SET` response falls through to the default switch case and is sent as
  `UNKNOWN_OP` rather than the actual set result.
- `READ_RESPONSE` and `SET_RESPONSE` operation values are implemented for USB,
  but are not yet used by the radio handler.

A dashboard may display mirrored radio configuration traffic, but should use
the USB `CONFIG` endpoint for configuration controls until these issues are
resolved.

## Link watchdog and heartbeat

The drone requires a periodic `HEARTBEAT` to keep moving. The same message also
carries the state the dashboard is asking for, so the heartbeat is both the
keepalive and the mode control channel. It is accepted over USB (type 10) and
RFM69 (type 10) with an identical 8-byte payload.

### One watchdog, either transport

There is a single watchdog, and a heartbeat on **either** link feeds it. This is
deliberate: a tethered bench dashboard on USB keeps the vehicle alive with the
radio unplugged, and the base station does the same with nothing on USB. Neither
link has to know the other exists.

The consequence is that neither link can detect the *other* one failing. A
dashboard watching only its own traffic cannot conclude from a healthy watchdog
that the radio is up; use `rssi` and the drone's own state for that.

The edge-triggered `state` request below is also shared state, tracked once for
both transports rather than per link. **Two links asking for different states at
the same time will fight**, because each differing heartbeat reads as a change
and is honored, flipping the vehicle between them at heartbeat rate. Command
from one link at a time; a passive observer on the other should either send no
heartbeats or send the same requested state as the commanding link.

### `HEARTBEAT` payload

| Offset | Type | Field | Meaning |
| ---: | --- | --- | --- |
| 0 | `uint8` | `state` | Requested drone state. See [Drone states](#drone-states). |
| 1 | bit 0 | `enableMotors` | Defined in the wire format but not yet read by the firmware. Send zero. |
| 1 | bit 1 | `enableGimbal` | Defined in the wire format but not yet read by the firmware. Send zero. |
| 1–7 | remaining 54 bits | reserved | Send zero. |

### Cadence

The watchdog expires **100 ms** after the last heartbeat. Send heartbeats well
inside that - 20–50 ms is a reasonable dashboard cadence, which tolerates one
or two lost packets before a trip.

Only `HEARTBEAT` feeds the watchdog. A dashboard that streams `COMMAND` packets
without heartbeats will be treated as a dead link and disarmed, however fresh
its commands are.

### What expiry does

1. Every actuator guard fails closed within one 1 kHz control tick: motors are
   driven to their idle pulse width and servo writes are suppressed.
2. The drone transitions to `SAFE` and latches an internal trip flag.
3. The `TRIPPED` bit of the `watchDog` telemetry byte latches, the status LED
   switches to a distinctive three-flash-then-pause pattern, and a `DEBUG_TEXT`
   message reading `DRONE: COMM WATCHDOG EXPIRED -> SAFE` is emitted.

### Detecting a trip

Both transports carry a `watchDog` flags byte: USB telemetry offset 54, RFM69
`STATUS0` offset 7. Identical encoding on both.

| Bit | Name | Meaning |
| ---: | --- | --- |
| 0 | `FED` | The watchdog timer has not expired. Live state; it clears and sets on its own as the link comes and goes. Note this reads *set* for the first 100 ms after power-on, before any heartbeat has ever arrived, because the timer starts from zero - do not read an early `FED` as proof a heartbeat was received. It is harmless to the vehicle: nothing that can move is reachable that early in boot. |
| 1 | `TRIPPED` | The watchdog expired somewhere movement was allowed. **Latched**: set at the moment of expiry and cleared only when the dashboard releases its requested state to `SAFE`. |
| 2–7 | reserved | Zero. |

`TRIPPED` is the flag to drive a dashboard alert from. It is latched precisely
so the event survives the outage that caused it: over the radio nothing reaches
the ground station while the link is down, so by the time packets flow again
the `currentMode` transition and the `DEBUG_TEXT` message have already been and
gone. A dashboard that connects after a trip still sees `TRIPPED` set.

`FED` is useful on its own as a link-health indicator - it shows the watchdog
is being satisfied before a trip happens, which `currentMode` cannot.

Two weaker signals remain available, but neither should be the primary
mechanism. `currentMode` transitioning to `SAFE` cannot distinguish a trip from
a commanded disarm. The `DEBUG_TEXT` message `DRONE: COMM WATCHDOG EXPIRED ->
SAFE` is USB-only, is emitted once per trip, and is display text rather than a
stable event API.

Expiry is only evaluated at or above `READY_ARMED`. Before then, no heartbeat
has been asked for and a cold watchdog is not a fault.

### Requested state is edge triggered

**This is the rule most likely to surprise a dashboard implementation.** The
firmware acts on `state` only when it **changes** from the previously accepted
request. A heartbeat that repeats the last requested state is treated as a
keepalive and nothing else.

The reason is that a heartbeat sent before a dropout is byte-identical to one
sent after it. If repeats were honored, a dashboard that reconnects and resumes
sending its last mode would silently re-arm the vehicle the instant the link
came back, with no operator in the decision. Acting only on a change makes
re-arming require a deliberate action.

The consequence for a dashboard is a required behavior change:

- **On link loss, set the requested state back to `SAFE`.** Keep sending
  heartbeats requesting `SAFE` while the link is down or reconnecting.
- **To resume, request the flight state again.** That `SAFE` → flight
  transition is the edge the firmware honors, and it is also what clears the
  latched trip indication.

A dashboard that instead keeps asserting its pre-dropout flight state after a
trip will find the vehicle stays in `SAFE` indefinitely, with no error
response - the request is silently ignored because it is not a change.

### Escalation dwell

An escalation out of `SAFE` is additionally refused until the vehicle has been
in `SAFE` for **500 ms**. This stops an automatic reconnect from walking
through `SAFE` and back into flight faster than an operator could intervene.

The refusal does not consume the request: a dashboard that holds its new
requested state will have it honored automatically on the next heartbeat once
the dwell has passed. No second operator action is needed.

### Motor setpoint reporting

`motor1Set` and `motor2Set`, in both the USB record and RFM69 `STATUS2`, report
what was actually commanded to the ESCs, not what was requested. When output is
suppressed - the watchdog is unfed, or the vehicle is not in `MAN_FLIGHT` - the
ESCs are driven to their idle pulse width and **both fields read `0`**, even
while a dashboard is still sending a non-zero throttle.

Treat a commanded throttle that reads back as `0` as confirmation that the guard
is holding, not as a lost command. To display what was requested, show the
dashboard's own outgoing value alongside these fields rather than expecting them
to echo it.

### Current failsafe behavior

`SAFE` zeroes the motors. This is correct for the current bench and tethered
configuration, but a dashboard should not present it as a recoverable-in-flight
failsafe: it is a disarm. A distinct link-loss flight mode is planned, and this
section will change when it lands.

## Drone states

State values are **not contiguous**. Do not index an array by them, and treat
any unlisted value as unknown rather than clamping it.

| Value | State | Dashboard interpretation |
| ---: | --- | --- |
| 0 | `BOOT` | Initial hardware setup. |
| 1 | `RADIO_SETUP` | Radio initialization/handshake. |
| 2 | `SENSOR_SETUP` | IMU and sensor initialization. |
| 3 | `CONTROL_SETUP` | Gimbal, motor, and control timer setup. |
| 4 | `SAFE` | Startup is complete and all systems are up, but no movement is allowed. This is the state a watchdog trip drops the vehicle into. |
| 10 | `READY_ARMED` | Gimbal movement allowed; motors stay at idle. |
| 11 | `MAN_FLIGHT` | Manually commanded motors and gimbal. Persistent configuration writes are rejected. |
| 12 | `AUTO_FLIGHT` | Flight-control-driven motors and gimbal. Persistent configuration writes are rejected. **Not implemented** - a request for it is refused. |
| 255 | `FAULT_ERROR` | Unrecoverable fault. The firmware will not leave this state; a power cycle is required. A link-loss trip never routes here. See [Behavior in `FAULT_ERROR`](#behavior-in-fault_error) - most of the protocol stops. |

The ordering is significant to the firmware: everything at or above
`READY_ARMED` (10) permits some movement and is watchdog-policed, and
everything below `SAFE` (4) is startup. A dashboard can use the same
comparison to decide when to show flight-critical indicators.

### Behavior in `FAULT_ERROR`

A boot fault is entered from the startup sequence, which never completes, so the
main loop never begins running. A dashboard should expect the following while
the vehicle is in this state:

- **`DEBUG_TEXT` still arrives**, including a `FAULT` message repeated once per
  second. USB framing, CRC and the identify handshake all keep working.
- **No `TELEMETRY` and no `STATUS0`–`STATUS6`.** Telemetry is assembled by the
  main loop, which is not running. The last values a dashboard received before
  the fault are the last it will get.
- **The radio is serviced but silent.** Incoming packets are still read, so the
  link does not go dead, but nothing is queued for transmission.
- **`HEARTBEAT` is accepted and ignored.** The requested state is refused: the
  firmware will not leave `FAULT_ERROR` under any request.

Because `currentMode` stops being transmitted, a dashboard cannot observe the
transition *into* this state over the radio. The absence of telemetry alongside
a live `DEBUG_TEXT` stream on USB is the reliable indication.

## Queue behavior

Every queue in the firmware is 16 deep and, when full, displaces its **oldest**
entry to make room for the newest. This holds in both directions and on both
transports.

The reasoning is the same either way. Telemetry is a stream of snapshots, so the
frame being queued now is worth more than the stale one it replaces; and an
inbound command or heartbeat that is still waiting has already been superseded by
the one arriving behind it. In both cases keeping the newest is what a dashboard
actually wants, and dropping it would be the worst available choice.

### Outbound (drone → dashboard)

Under congestion a dashboard loses *intermediate* frames rather than the most
recent one, so the data it receives is always the freshest available.

The practical consequence is that **USB packet sequence numbers can skip**. A
gap means frames were displaced by newer ones, not that the link is corrupt.
Track gaps as a link-quality indicator; do not treat them as an error or attempt
to request a retransmission, which the protocol does not support.

### Inbound (dashboard → drone)

A CRC-valid frame is always accepted. If 16 frames are already waiting, the
oldest is discarded rather than the arriving one, so the newest command or
heartbeat is never the one lost.

Reaching that depth requires the firmware's main loop to stall for longer than
it takes to receive 16 frames, which does not happen in normal operation - the
loop runs orders of magnitude faster than the serial link delivers. A dashboard
does not need to rate-limit for this. But note the implication if it ever does
happen: the frames discarded are the oldest queued, which could include a
`CONFIG` request whose response the dashboard is waiting on. Treat a config
response that never arrives as a timeout and retry, rather than assuming the
request was rejected.

## Dashboard implementation guidance

- Frame and CRC-validate every USB message before decoding it.
- Treat telemetry fields marked as placeholders - `latitude` and `longitude`
  are the remaining ones - as unavailable, rather than as real measurements.
- Expect sequence-number gaps under load; see
  [Queue behavior](#queue-behavior).
- Track USB frame sequence gaps and retain recent `DEBUG_TEXT` messages for
  diagnostics.
- Render radio traffic from `RADIO_PACKET` as an inspector/debug view; decode
  its embedded payload with the RFM69 tables above.
- Do not indicate that a command has been accepted merely because it was sent:
  command acknowledgements and timeouts are not implemented.
- Use USB `CONFIG` reads to populate settings and USB `CONFIG` sets to save
  changes. Show every per-key result returned by a set response.
