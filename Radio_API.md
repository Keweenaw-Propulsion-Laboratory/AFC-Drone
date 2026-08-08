# Radio API

The `Radio` class manages the RFM69 radio used by the autonomous drone. It initializes the transceiver, negotiates a connection with the base station, queues outgoing telemetry, receives incoming commands, and dispatches messages according to their packet type.

> [!IMPORTANT]
> This document describes both the **intended public API** in `radio.h` and the **current behavior** implemented in `radio.cpp`.
>
> The final section lists implementation mismatches that currently affect compilation or protocol operation. Base-station software should not treat the protocol as stable until those items are resolved.

## Radio configuration

The radio uses the RadioHead `RH_RF69` driver.

| Setting | Value | Description |
|---|---:|---|
| Radio | RFM69 | Sub-GHz packet radio |
| Frequency | `915.0 MHz` | Configured by `RF69_FREQ` |
| Chip-select pin | `10` | SPI chip-select |
| Interrupt pin | `40` | RFM69 interrupt output |
| Reset pin | `41` | RFM69 hardware reset |
| Radio LED pin | `13` | Declared by `LED` |
| Transmit power | `20 dBm` | High-power mode enabled |
| Encryption | Enabled | 16-byte key configured during setup |
| TX queue capacity | 16 messages | `Circular_Buffer<RadioPacket, 16>` |
| RX queue capacity | 16 messages | Declared, but not currently used |
| Minimum time between transmissions | `10 ms` | Defined by `RX_WINDOW_MIN` |

The current encryption key is:

```text
01 02 03 04 05 06 07 08 01 02 03 04 05 06 07 08
```

The base station must use the same frequency, modulation configuration, and encryption key.

---

## Public API

### `Radio::setup()`

```cpp
static bool setup();
```

Advances the nonblocking radio setup state machine by one step.

The function is intended to be called repeatedly while the drone is in `DroneStates::RADIO_SETUP`.

#### Return value

| Value | Meaning |
|---|---|
| `true` | The current setup step completed successfully or is still waiting normally |
| `false` | A fatal initialization step failed |

A return value of `true` does **not** mean setup is complete. Call `setupComplete()` to determine whether the connection handshake has finished.

Initialization failures are added to `ErrorHandler`:

| Error | Code | Severity | Cause |
|---|---:|---:|---|
| `radioInitFail` | `1` | `1` | `RH_RF69::init()` failed |
| `radioFreqSetFail` | `2` | `1` | The 915 MHz frequency could not be set |

### `Radio::setupComplete()`

```cpp
static bool setupComplete();
```

Returns `true` when the radio setup state is `RadioSetupStates::COMPLETE`.

### `Radio::update()`

```cpp
static void update();
```

Runs periodic radio processing after setup is complete.

The method:

1. Checks for an incoming packet.
2. Receives and dispatches supported packet types.
3. Waits while the RFM69 is transmitting.
4. Maintains a receive window after each transmission.
5. Pops the next message from the outgoing queue.
6. Adds the protocol header and begins a nonblocking transmission.

`update()` currently performs no work until `setupComplete()` returns `true`.

### `Radio::sendMessage()`

```cpp
static void sendMessage(RadioMessage data, MessageType type);
```

Adds an 8-byte message payload to the outgoing queue.

This method does not immediately transmit the packet. Transmission occurs later when `Radio::update()` removes it from the queue.

#### Parameters

| Parameter | Description |
|---|---|
| `data` | Eight-byte payload stored in a `RadioMessage` |
| `type` | Message type associated with the payload |

The method has no return value and does not report whether the 16-entry queue was full.

### `Radio::getMessage()`

```cpp
static bool getMessage(
    uint8_t (&buffer)[RH_RF69_MAX_MESSAGE_LEN],
    uint8_t& bufferLength
);
```

Receives a raw packet directly from the RFM69.

#### Parameters

| Parameter | Direction | Description |
|---|---|---|
| `buffer` | Output | Destination for received bytes |
| `bufferLength` | Input/output | Buffer capacity on entry and received length on return, according to the RadioHead receive convention |

#### Return value

| Value | Meaning |
|---|---|
| `true` | A packet was received successfully |
| `false` | No packet was available or reception failed |

Callers should initialize `bufferLength` before calling:

```cpp
uint8_t buffer[RH_RF69_MAX_MESSAGE_LEN];
uint8_t length = sizeof(buffer);

if (Radio::getMessage(buffer, length)) {
    // Process length bytes.
}
```

### Status sender methods

```cpp
static void sendStatus0();
static void sendStatus1();
static void sendStatus2();
static void sendStatus3();
static void sendStatus4();
static void sendStatus5();
static void sendStatus6();
```

Each method is intended to collect one category of telemetry and queue the corresponding status packet.

Current implementation status:

| Method | Current behavior |
|---|---|
| `sendStatus0()` | Implemented and queues `STATUS0` |
| `sendStatus1()` | Partially implemented and queues `STATUS1` |
| `sendStatus2()` | Stub; does not queue a packet |
| `sendStatus3()` | Stub; does not queue a packet |
| `sendStatus4()` | Stub; does not queue a packet |
| `sendStatus5()` | Stub; does not queue a packet |
| `sendStatus6()` | Stub; does not queue a packet |

---

## Setup state machine

```cpp
enum class RadioSetupStates : uint8_t {
    RESET1,
    RESET2,
    RADIO_INIT,
    SET_CONFIG,
    SEND_CONN,
    WAIT_ACK,
    COMPLETE
};
```

| Numeric value | State | Current action |
|---:|---|---|
| `0` | `RESET1` | Configures the reset pin, drives it high, and records the current time |
| `1` | `RESET2` | Drives reset low after 10 ms and advances after 20 ms total |
| `2` | `RADIO_INIT` | Calls `radio.init()` |
| `3` | `SET_CONFIG` | Sets frequency, encryption key, and 20 dBm high-power mode |
| `4` | `SEND_CONN` | Queues the 8-byte text payload `AFCDrone` as a `SETUP` message |
| `5` | `WAIT_ACK` | Waits for an 8-byte acknowledgment and retries after 1 second |
| `6` | `COMPLETE` | Setup is complete |

### Connection request

The connection identifier is exactly eight ASCII bytes:

```text
A F C D r o n e
41 46 43 44 72 6F 6E 65
```

It is queued with message type `SETUP`.

Under the normal runtime framing described below, the transmitted request is intended to be:

```text
Byte 0       Message number
Byte 1       Message type = 0 (SETUP)
Bytes 2-9    ASCII "AFCDrone"
```

### Setup acknowledgment

The expected acknowledgment constant is:

```text
69 69 69 69 69 69 69 69
```

The setup implementation currently accepts a received packet when:

- The received length is exactly 8 bytes.
- The first byte is `0x69`.

It does not currently verify bytes 1 through 7.

### Retry behavior

When no acknowledgment is received within 1,000 ms, setup returns to `SEND_CONN` and increments `retryCounter`.

There is no maximum retry count.

---

## Runtime packet format

The source constructs a runtime frame by concatenating a 2-byte application header and an 8-byte payload.

```text
+------------+-------------+-----------------------------+
| Byte 0     | Byte 1      | Bytes 2 through 9           |
+------------+-------------+-----------------------------+
| msgNum     | packetType  | RadioMessage payload        |
+------------+-------------+-----------------------------+
```

| Region | Size | Description |
|---|---:|---|
| `header_t` | 2 bytes | Sequence number and message type |
| `RadioMessage` | 8 bytes | Type-specific payload |
| Total | 10 bytes | Application-level radio frame |

### Application header

```cpp
struct __attribute__((packed)) header_t {
    uint8_t msgNum;
    uint8_t packetType;
};
```

| Offset | Field | Type | Description |
|---:|---|---|---|
| `0` | `msgNum` | `uint8_t` | Outgoing packet sequence number |
| `1` | `packetType` | `uint8_t` | Numeric `MessageType` value |

`globalPacketNum` starts at zero and is incremented after each queued packet is removed for transmission. As an unsigned 8-bit value, it wraps from 255 to 0.

Packet-loss and sequence validation are marked as TODO and are not currently implemented.

### Payload union

```cpp
union RadioMessage {
    uint64_t raw;
    StatusMsg0_t status0;
    StatusMsg1_t status1;
    StatusMsg2_t status2;
    StatusMsg3_t status3;
    StatusMsg4_t status4;
    StatusMsg5_t status5;
    StatusMsg6_t status6;
    Command_t command;
    char textArray[8];
};
```

Every payload is exactly 8 bytes:

```cpp
static_assert(
    sizeof(RadioMessage) == sizeof(uint64_t),
    "Radio messages must be 8 bytes"
);
```

Messages should be zero-initialized before fields are assigned:

```cpp
Radio::RadioMessage message{};
```

This is especially important for payloads with reserved or currently unassigned fields.

---

## Message types

```cpp
enum class MessageType : uint8_t {
    SETUP   = 0,
    STATUS0 = 1,
    STATUS1 = 2,
    STATUS2 = 3,
    STATUS3 = 4,
    STATUS4 = 5,
    STATUS5 = 6,
    STATUS6 = 7,
    COMMAND = 8,
    CONFIG  = 9
};
```

| Value | Name | Intended direction | Payload |
|---:|---|---|---|
| `0` | `SETUP` | Drone ↔ base station | Setup-specific data |
| `1` | `STATUS0` | Drone → base station | General timing and state |
| `2` | `STATUS1` | Drone → base station | Gimbal, actuator, and battery data |
| `3` | `STATUS2` | Drone → base station | Quaternion |
| `4` | `STATUS3` | Drone → base station | Acceleration |
| `5` | `STATUS4` | Drone → base station | Velocity |
| `6` | `STATUS5` | Drone → base station | Position |
| `7` | `STATUS6` | Drone → base station | GPS coordinates |
| `8` | `COMMAND` | Base station → drone | Command targets |
| `9` | `CONFIG` | Base station → drone | Configuration data |

`CONFIG` and received `SETUP` handlers are declared but currently empty.

---

# Telemetry payloads

## `STATUS0`: General system status

```cpp
struct __attribute__((packed)) StatusMsg0_t {
    uint16_t loopTimeAvg;
    uint16_t loopTimeMax;
    uint16_t RunTime;
    uint8_t rssi;
    uint8_t currentMode;
};
```

| Offset | Field | Type | Unit / encoding | Current source |
|---:|---|---|---|---|
| `0` | `loopTimeAvg` | `uint16_t` | Microseconds, intended | `Drone::rollAvg` |
| `2` | `loopTimeMax` | `uint16_t` | Microseconds | `Drone::worstTime` |
| `4` | `RunTime` | `uint16_t` | Seconds | `millis() / 1000` |
| `6` | `rssi` | `uint8_t` | Not finalized | `lastRssi` |
| `7` | `currentMode` | `uint8_t` | `DroneStates` numeric value | `Drone::state` |

### Drone state values

```cpp
enum class DroneStates : uint8_t {
    BOOT,
    RADIO_SETUP,
    SENSOR_SETUP,
    READY_ARMED,
    FLIGHT,
    FAULT_ERROR
};
```

| Value | State |
|---:|---|
| `0` | `BOOT` |
| `1` | `RADIO_SETUP` |
| `2` | `SENSOR_SETUP` |
| `3` | `READY_ARMED` |
| `4` | `FLIGHT` |
| `5` | `FAULT_ERROR` |

### Runtime rollover

`RunTime` rolls over after 65,535 seconds, approximately 18 hours, 12 minutes, and 15 seconds.

### Current implementation notes

- `lastRssi` is initialized to zero but is not updated anywhere in the supplied source.
- `lastRssi` is signed 16-bit, while the transmitted field is unsigned 8-bit.
- `Drone::rollAvg` is declared and referenced but is not defined in the supplied source.
- `main.cpp` calculates a local `rollingAverage`, but it does not assign that value to `Drone::rollAvg`.

---

## `STATUS1`: Gimbal, motors, and battery

```cpp
struct __attribute__((packed)) StatusMsg1_t {
    int8_t gimbalPitchNorm;
    int8_t gimbalYawNorm;
    uint8_t topServoSet;
    uint8_t bottomServoSet;
    uint8_t motor1set;
    uint8_t motor2set;
    uint16_t voltage;
};
```

| Offset | Field | Type | Current meaning |
|---:|---|---|---|
| `0` | `gimbalPitchNorm` | `int8_t` | Requested gimbal pitch converted from `Gimbal::getPitch()` |
| `1` | `gimbalYawNorm` | `int8_t` | Requested gimbal yaw converted from `Gimbal::getYaw()` |
| `2` | `topServoSet` | `uint8_t` | Top servo command in degrees |
| `3` | `bottomServoSet` | `uint8_t` | Bottom servo command in degrees |
| `4` | `motor1set` | `uint8_t` | Motor telemetry field; not assigned |
| `5` | `motor2set` | `uint8_t` | Motor telemetry field; not assigned |
| `6` | `voltage` | `uint16_t` | Battery voltage field; not assigned |

The gimbal implementation stores requested pitch and yaw as floats. `sendStatus1()` converts them to signed 8-bit integers, discarding fractional degrees.

The servo outputs are limited to 60–120 degrees before being reported.

> [!WARNING]
> `sendStatus1()` currently creates an uninitialized `RadioMessage` and assigns only the first four bytes. Motor and voltage bytes may therefore contain indeterminate data.

---

## `STATUS2`: Orientation quaternion

```cpp
struct __attribute__((packed)) StatusMsg2_t {
    int16_t qR;
    int16_t qI;
    int16_t qJ;
    int16_t qK;
};
```

| Offset | Field | Type | Description |
|---:|---|---|---|
| `0` | `qR` | `int16_t` | Quaternion real component |
| `2` | `qI` | `int16_t` | Quaternion i component |
| `4` | `qJ` | `int16_t` | Quaternion j component |
| `6` | `qK` | `int16_t` | Quaternion k component |

The BNO08x gyro provides quaternion data, but no conversion scale from float to `int16_t` has been implemented. `sendStatus2()` is currently a stub.

---

## `STATUS3`: Acceleration

```cpp
struct __attribute__((packed)) StatusMsg3_t {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t empty;
};
```

| Offset | Field | Type | Description |
|---:|---|---|---|
| `0` | `accelX` | `int16_t` | X-axis acceleration |
| `2` | `accelY` | `int16_t` | Y-axis acceleration |
| `4` | `accelZ` | `int16_t` | Z-axis acceleration |
| `6` | `empty` | `int16_t` | Reserved |

Units, scale, and coordinate frame are not yet defined. `sendStatus3()` is currently a stub.

---

## `STATUS4`: Velocity

```cpp
struct __attribute__((packed)) StatusMsg4_t {
    int16_t velX;
    int16_t velY;
    int16_t velZ;
    int16_t empty;
};
```

| Offset | Field | Type | Description |
|---:|---|---|---|
| `0` | `velX` | `int16_t` | X-axis velocity |
| `2` | `velY` | `int16_t` | Y-axis velocity |
| `4` | `velZ` | `int16_t` | Z-axis velocity |
| `6` | `empty` | `int16_t` | Reserved |

Units, scale, origin, and coordinate frame are not yet defined. `sendStatus4()` is currently a stub.

---

## `STATUS5`: Position

```cpp
struct __attribute__((packed)) StatusMsg5_t {
    int16_t posX;
    int16_t posY;
    int16_t posZ;
    int16_t empty;
};
```

| Offset | Field | Type | Description |
|---:|---|---|---|
| `0` | `posX` | `int16_t` | X-axis position |
| `2` | `posY` | `int16_t` | Y-axis position |
| `4` | `posZ` | `int16_t` | Z-axis position |
| `6` | `empty` | `int16_t` | Reserved |

Units, scale, origin, and coordinate frame are not yet defined. `sendStatus5()` is currently a stub.

---

## `STATUS6`: GPS position

```cpp
struct __attribute__((packed)) StatusMsg6_t {
    float latitude;
    float longitude;
};
```

| Offset | Field | Type | Intended unit |
|---:|---|---|---|
| `0` | `latitude` | 32-bit `float` | Decimal degrees |
| `4` | `longitude` | 32-bit `float` | Decimal degrees |

GPS support is commented out in the drone startup sequence, and `sendStatus6()` is currently a stub.

Both endpoints must agree on 32-bit IEEE 754 floating-point encoding and byte order.

---

# Command payload

## Header declaration

```cpp
struct __attribute__((packed)) Command_t {
    struct __attribute__((packed)) flags {
        uint8_t targSlot : 1;
        uint8_t activeSlot : 1;
        uint8_t empty : 6;
    } flags;

    int16_t gimbalX;
    int16_t gimbalY;
    uint8_t motor0Speed;
    uint8_t motor1Speed;
    uint8_t empty0;
};
```

| Offset | Field | Type | Description |
|---:|---|---|---|
| `0` | `flags` | `uint8_t` bit field | Target and active slot selection |
| `1` | `gimbalX` | `int16_t` | Raw gimbal X target |
| `3` | `gimbalY` | `int16_t` | Raw gimbal Y target |
| `5` | `motor0Speed` | `uint8_t` | Motor 0 target |
| `6` | `motor1Speed` | `uint8_t` | Motor 1 target |
| `7` | `empty0` | `uint8_t` | Reserved |

### Flag byte

| Bit | Name | Current behavior |
|---:|---|---|
| `0` | `targSlot` | `0` writes target slot 0; `1` writes target slot 1 |
| `1` | `activeSlot` | Copied to the global `drone_activeSlot` |
| `2–7` | `empty` | Reserved |

For cross-language compatibility, the base station should treat the flags as an explicit byte:

```cpp
constexpr uint8_t TARGET_SLOT_MASK = 1U << 0;
constexpr uint8_t ACTIVE_SLOT_MASK = 1U << 1;
```

C++ bit-field allocation order is implementation-dependent, so explicit masks are safer than reproducing the bit-field declaration in another compiler or language.

## Gimbal scaling

The periodic target-processing function divides both raw gimbal commands by `1310.0f`:

```cpp
Gimbal::set(
    target.gimbalX / 1310.0f,
    target.gimbalY / 1310.0f
);
```

This maps approximately the full signed 16-bit range to ±25 degrees.

The gimbal lookup tables themselves cover −20 through +20 degrees, and `Gimbal::set()` clamps requests to that range before interpolation. Therefore, values beyond approximately ±26,200 produce no additional commanded travel.

Approximate conversion:

```text
gimbal degrees = raw command / 1310.0
raw command    = gimbal degrees × 1310.0
```

Examples:

| Raw value | Requested angle |
|---:|---:|
| `-26200` | `-20°` |
| `-13100` | `-10°` |
| `0` | `0°` |
| `13100` | `10°` |
| `26200` | `20°` |

## Target slots

The drone defines two target structures:

```cpp
struct {
    int16_t gimbalX;
    int16_t gimbalY;
    uint16_t targetRoll;
    uint8_t motor0Speed;
    uint8_t motor1Speed;
} Target_t;
```

Incoming commands are intended to update one target slot while independently selecting the active slot.

Current command-handler behavior:

```text
targSlot = 0  -> write drone_targ0
targSlot = 1  -> write drone_targ1
activeSlot    -> copy to drone_activeSlot
```

The separate `drone_update()` function currently selects:

```text
drone_activeSlot = 1 -> use drone_targ0
drone_activeSlot = 0 -> use drone_targ1
```

This selection is inverted relative to the target-slot naming and should be confirmed.

---

# Receive dispatch

The current runtime dispatcher recognizes:

| Message type | Current action |
|---|---|
| `SETUP` | No action |
| `COMMAND` | Calls `radio_handleCommand()` |
| `CONFIG` | No action |
| All status types | No receive-side action |
| Unknown value | Ignored |

The receive queue is declared but not currently populated or processed.

---

# Transmission scheduling

Outgoing messages are stored as:

```cpp
struct __attribute__((packed)) RadioPacket {
    Radio::RadioMessage message;
    Radio::MessageType type;
};
```

The type is not part of the 8-byte payload. It is converted into the second byte of the 10-byte frame when the packet is transmitted.

After a packet begins transmission:

1. `lastTxTime` is set to the current `millis()` value.
2. `update()` returns while the driver reports transmit mode.
3. For the first 10 ms after `lastTxTime`, the code requests receive mode and sends nothing else.
4. After the receive window, the next queued packet may be transmitted.

This creates an intended maximum application packet rate of roughly 100 packets per second before accounting for radio airtime and processing.

---

# Serialization requirements

## Byte order

The implementation serializes packed C++ structures with `memcpy()`. Multi-byte integers and floats therefore use the microcontroller's native byte order.

The base station must either:

- Use the same byte order and field representations, or
- Decode every multi-byte field explicitly according to the drone's wire format.

## Floating-point format

`STATUS6` assumes a 4-byte `float`. A compile-time check is recommended:

```cpp
static_assert(sizeof(float) == 4, "Radio protocol requires 32-bit floats");
```

## Packed and unaligned fields

Several 16-bit fields begin at odd byte offsets, especially in `Command_t`. Packed access may be unaligned. The current target is expected to tolerate this, but field-by-field serialization would be more portable.

---

# Example packet encoding

The following example encodes a command for target slot 0, makes slot 0 active, requests +10 degrees X, requests −5 degrees Y, and sets both motors to zero.

```text
msgNum       = 37
packetType   = 8 (COMMAND)
flags        = 0b00000010
gimbalX      = 13100
gimbalY      = -6550
motor0Speed  = 0
motor1Speed  = 0
empty0       = 0
```

Assuming little-endian 16-bit integers, the 10 application bytes are:

```text
25 08 02 2C 33 6A E6 00 00 00
```

Breakdown:

```text
25          msgNum = 37
08          COMMAND
02          activeSlot flag
2C 33       gimbalX = 13100
6A E6       gimbalY = -6550
00          motor0Speed
00          motor1Speed
00          reserved
```

---

# Integration with drone startup

`Drone::startup()` drives radio setup as follows:

```cpp
case DroneStates::RADIO_SETUP:
    if (!Radio::setup()) {
        state = DroneStates::FAULT_ERROR;
    }

    if (Radio::setupComplete()) {
        state = DroneStates::SENSOR_SETUP;
    }
    break;
```

The top-level Arduino `setup()` blocks until `Drone::startup()` reaches `READY_ARMED`:

```cpp
void setup() {
    while (!Drone::startup()) {}
}
```

After startup, `main.cpp` repeatedly calls `Drone::update()`. The supplied `Drone::update()` is empty and does not currently call `Radio::update()`.

---

# Known implementation issues

These items are directly visible in the supplied source and should be resolved before the protocol is used as a stable base-station interface.

## 1. The setup connection request is queued but cannot be transmitted

`SEND_CONN` calls `sendMessage()`, which only adds the request to `radio_tx_buffer`.

The queue is transmitted only by `Radio::update()`, but `Radio::update()` does nothing until `setupComplete()` is true. During startup, `Drone::startup()` calls `Radio::setup()` but not `Radio::update()`.

As written, the `AFCDrone` connection request remains queued while setup waits for an acknowledgment.

Possible resolutions include:

- Transmit the setup request directly inside `SEND_CONN`.
- Allow `Radio::update()` to service the TX queue during `SEND_CONN` and `WAIT_ACK`.
- Call a setup-specific radio update function from the startup state machine.

## 2. The acknowledgment receive length is uninitialized

`WAIT_ACK` declares:

```cpp
uint8_t buffLength;
```

and passes it to `getMessage()` without assigning the buffer capacity.

It should be initialized:

```cpp
uint8_t buffLength = sizeof(recvBuffer);
```

## 3. Runtime receive parsing uses two different header mechanisms

Transmit code places `header_t` in bytes 0 and 1 of the application payload.

Receive code obtains `currentPacketNum` and `messageType` from:

```cpp
radio.headerId();
radio.headerFlags();
```

but then skips the first two bytes of the received buffer when copying the payload.

The source does not set RadioHead's header ID or flags before transmission. The receiver should either:

- Copy `header_t` from `buffer[0]` and `buffer[1]`, or
- Use RadioHead header fields for the protocol and stop embedding `header_t` in the data buffer.

## 4. Received length is not checked before copying eight payload bytes

`Radio::update()` copies eight bytes from `buffer + 2` without first checking that `len` is at least 10.

Malformed or short packets should be rejected before `memcpy()`.

## 5. `Command_t` has no `targetRoll` member

`radio_handleCommand()` reads:

```cpp
msg.command.targetRoll
```

but `Command_t` in `radio.h` does not declare this field.

The current source will not compile unless another, different header is being used.

Adding a 16-bit `targetRoll` while retaining all current command fields would make the payload larger than eight bytes. The command layout must therefore be redesigned or split across multiple packet types.

## 6. `STATUS1` transmits uninitialized fields

`sendStatus1()` assigns only gimbal and servo values. Motor and battery fields remain uninitialized because the union is not zero-initialized.

At minimum:

```cpp
RadioMessage msg{};
```

should be used.

## 7. RSSI is never updated

`lastRssi` remains zero in the supplied source. The receiver should update it from the radio driver after a valid packet if `STATUS0.rssi` is intended to report link strength.

The signed/unsigned wire representation also needs to be finalized.

## 8. `Drone::rollAvg` is declared but not defined

`sendStatus0()` references `Drone::rollAvg`, but no definition is present in the supplied `.cpp` files.

`main.cpp` computes a separate local rolling average that is never stored in the `Drone` class.

## 9. Periodic radio processing is not called

`main.cpp` calls `Drone::update()`, but `Drone::update()` is empty. It does not call:

- `Radio::update()`
- `drone_update()`
- `Drone::updateLEDS()`
- Gyro event processing

Consequently, runtime radio messages are not currently sent, received, or applied after startup.

## 10. Active-slot selection appears inverted

The command handler writes slot 0 when `targSlot` is zero and slot 1 when it is one.

The target application function chooses slot 0 when `drone_activeSlot` is true and slot 1 when it is false. This is opposite the conventional numeric interpretation.

## 11. Gimbal interpolation can index beyond the lookup table

`Gimbal::set()` clamps pitch and yaw to the final lookup value of `20`, then calculates `column + 1` and `row + 1`.

At an exact upper-bound input, this may access index 9 in a 9-element dimension. Radio commands at or above the positive clamp can therefore expose an out-of-bounds lookup.

## 12. No command timeout or failsafe is implemented

The command handler updates persistent target structures, but there is no timestamp, timeout, arming check, or automatic motor-safe state when communication is lost.

For an autonomous aircraft, command freshness and failsafe behavior should be part of the protocol contract.

---

# Protocol details still requiring definition

The supplied source does not yet define:

1. Quaternion fixed-point scaling.
2. Acceleration, velocity, and position units.
3. Coordinate frames and axis signs.
4. GPS validity and unavailable-value encoding.
5. Motor command range and units.
6. Battery voltage scaling.
7. RSSI encoding.
8. `CONFIG` payload formats.
9. Whether status packets require acknowledgments.
10. Queue-full behavior.
11. Command timeout and failsafe response.
12. Protocol versioning.
13. Compatibility behavior for unknown message types.
14. Authentication or replay protection beyond the static radio encryption key.
15. Whether setup acknowledgment should use the normal 10-byte frame or the current raw 8-byte format.

---

# Recommended wire-format stabilization

Before implementing the base station, define a versioned protocol contract with:

- One unambiguous header mechanism.
- Explicit little-endian or big-endian encoding.
- Explicit scaling for every integer telemetry field.
- A command sequence number or freshness timestamp.
- A command timeout.
- Reserved-byte requirements.
- A protocol version included in setup.
- Exact packet lengths for every message type.
- Compile-time size checks for every packed structure.
- Decode-time bounds and range validation.

Example size checks:

```cpp
static_assert(sizeof(Radio::header_t) == 2);
static_assert(sizeof(Radio::StatusMsg0_t) == 8);
static_assert(sizeof(Radio::StatusMsg1_t) == 8);
static_assert(sizeof(Radio::StatusMsg2_t) == 8);
static_assert(sizeof(Radio::StatusMsg3_t) == 8);
static_assert(sizeof(Radio::StatusMsg4_t) == 8);
static_assert(sizeof(Radio::StatusMsg5_t) == 8);
static_assert(sizeof(Radio::StatusMsg6_t) == 8);
static_assert(sizeof(Radio::Command_t) == 8);
static_assert(sizeof(Radio::RadioMessage) == 8);
```
