## Description
### Notion Task(s): 
  - [Task Name](link)
### Summary of Changes: 

This PR does two things: it adds `docs/code-conventions.md`, which writes down
how we want firmware in this repo to be written, and it migrates every existing
module to follow it.

#### Part 1 — The convention

`docs/code-conventions.md` is a sixteen-section guide, linked from the README.
Sections 1–4 are the ones to read before your first PR; the rest is reference.
The main points:

1. **Write for the target, not for a textbook.** There is exactly one of each
   thing on the vehicle, the control loop is hard real-time at 1 kHz, and there
   is no OS to catch a mistake. So: no `new`/`malloc`, no `virtual`, no
   `std::vector`/`string`/`function`, no exceptions or RTTI. We do use
   `enum class`, `constexpr`, references, `static_assert`, and namespaces —
   compile-time features that cost nothing at runtime and catch mistakes before
   the code reaches the vehicle.
2. **Every subsystem is a `namespace`, not a class of statics.** A `static` in a
   `.cpp` hides strictly more than `private:` in a header, and a namespace needs
   none of the out-of-line member definitions a static class does. The doc also
   explains why a namespace beats `radio_`-style prefixes: the namespace is
   enforced by the compiler, so the safe outcome is what happens when somebody
   forgets, whereas a prefix has to be remembered on every declaration forever.
   The `.cpp` reopens `namespace X { }` rather than using `using namespace X;`,
   which cannot define anything in the namespace and silently emits a global
   instead — the doc works through that failure and what it costs you.
3. **A module owns its state, and nobody outside it may write to that state.**
   No `extern` on a mutable variable in a header, ever. Reads go through an
   accessor; writes go through a command function that can validate. Never
   duplicate state to work around encapsulation.
4. **Setup is a non-blocking state machine.** No `delay()` in `setup()`; record
   `millis()` and return. Nothing that waits on another vehicle or an operator
   belongs in startup, and startup brings up electronics only — no physical
   actuation.
5. **The main loop contract is explicit.** Radio, USB and gyro servicing run
   every pass; flight control runs on the 1000 µs tick. Anything called from the
   control tick must never call `delay()`, must not block on I/O, must have a
   bounded iteration count, and must fit the whole tick inside its budget.
   Prefer `float` over `double`.
6. **ISRs touch almost nothing.** No I2C, SPI, `Serial`, `delay()`, or tx-queue
   pushes from an interrupt.
7. **Wire formats are byte-for-byte contracts** with the ground station and the
   dashboard, so every payload struct asserts its own size.
8. **No bare numbers in logic; Doxygen on every public function; document units
   and ranges.** "Angle in degrees, −20 to +20" is useful; "the angle" is not.

Sections 15 and 16 give step-by-step recipes for adding a subsystem, a config
setting, and a telemetry field, plus the migration status of each module.

#### Part 2 — The changes needed to get there

1. **Module pattern.** Every subsystem is now `namespace X { }` in both the
   header and the `.cpp`. Previously the `.cpp` files opened with
   `using namespace X;`, which silently defines a *new global* instead of the
   declared member when a definition is unqualified — that is what produced the
   earlier link errors. `Gyro` was a class of statics and is now a namespace,
   with the BNO08x handle, sensor value, setup state, and raw sensor-frame
   quaternion moved out of the header into `gyro.cpp`.
2. **State ownership.** No module exports mutable state any more.
   `radio_avgRSSI` became `Radio::getAvgRSSI()`, the public `Gyro` variables
   became accessors, and `Configs::config`, `Drone::controlTimer`,
   `Gyro::debugTimer` and `Gyro::report` were given internal linkage.
3. **Control loop and telemetry.** `Drone::update()` is now called directly from
   the control timer ISR rather than being gated on a flag polled in `loop()`,
   so its cadence no longer depends on how long a pass through radio/USB/gyro
   servicing takes. Loop-cost timing (last/best/worst/rolling average) moved
   into the ISR alongside it, and a `missedTicks` counter was added for ticks
   where `update()` overran its own 1000 µs period. `loop()` now owns telemetry:
   it takes one interrupt-guarded `Drone::Telemetry_t` snapshot every 100 ms and
   hands the same snapshot to all eight senders, so every packet in a frame
   describes the same instant.
4. **Naming.** Dropped prefixes that were redundant inside their own namespace
   (`radio_sendMessage` → `sendMessage`, `usb_header_t` → `header_t`,
   `config_migrate` → `migrate`, `drone_targ0` → `activeTarget`), and fixed
   `getTopSevo`, `setupTimmer`, `debugTimmer` and several doc-comment
   misspellings.
5. **Constants.** The remaining nine `#define`s became `constexpr`; two unused
   ones were deleted.
6. **Wire formats.** Added `static_assert`s on the size of all ten radio payload
   structs and on `PersistentConfig`, so a layout change fails the build instead
   of silently breaking ground-station parsing or reinterpreting saved EEPROM
   trim under the wrong layout.
7. **Dead code kept, not deleted.** `Radio::handleSetup`,
   `Gyro::quaternionToEulerRV`/`GI` and `Gyro::debug` turned out to be
   unreachable once internal linkage made that visible. They are marked
   `[[maybe_unused]]` rather than removed.

Compiled code size was identical at every step except the internal-linkage pass,
which saved 64 bytes.

## Breaking Changes
1. **Every module's public API was renamed.** Anything calling `config_load`,
   `radio_update`, `radio_sendMessage`, `usb_update`, `usb_send_text` and the
   like must move to the namespaced names (`Configs::load`, `Radio::update`,
   `Radio::sendMessage`, `USB::update`, `USB::sendText`). All in-tree call
   sites are updated in this PR; anything out of tree is not.
2. **Public module state is gone.** `extern int16_t radio_avgRSSI` and the
   public `Gyro` variables no longer exist — use `Radio::getAvgRSSI()` and the
   `Gyro` accessors.
3. **Radio and USB types were renamed.** `radio_Header` → `PacketHeader`,
   `radio_Packet` → `DataPacket`, `radio_MessageType` → `Radio::MessageType`,
   `RadioStates` → `TransceiverStates`, `usb_message_types` →
   `USB::MessageTypes`, `usb_radio_direction` → `USB::RadioDirection`, and
   payload fields `motor1set`/`motor2set` → `bottomMotorSet`/`topMotorSet`,
   `motor0Speed`/`motor1Speed` → `bottomMotor`/`topMotor`.
   **This is a source-level break only** — field order, sizes and packing are
   unchanged, so the bytes on the wire are identical and the ground station
   needs no change.
4. **`Gimbal::selfTest()` now requires an argument.** `selfTest(bool lookup)`
   selects between the combined-axis sweep and the independent-servo sweep. It
   has no callers in the firmware today.

## Hardware Impact
- [ ] **Pin Changes:** Does this change any digital/analog pin assignments?

  No. `YAW_SERVO_PIN` (25) and `PITCH_SERVO_PIN` (24) changed from `#define` to
  `constexpr int` with the same values. No other pin assignment was touched.

- [ ] **Power/Current:** Any changes to PWM frequencies or high-draw components?

  No. No PWM frequency, resolution, or motor/servo output range was changed.

- [x] **Timing:** Any changes to `delay()`, `intervals`, or interrupt service routines?

  Yes, three:
  1. `Drone::update()` now runs **inside** the control timer ISR
     (`Drone::onControlTick`) instead of being flagged there and executed from
     `loop()`. The tick period (`CONTROL_LOOP_US`) and timer priority are
     unchanged, but the flight control algorithm now executes at interrupt
     priority.
  2. Telemetry transmission moved out of the control path into `loop()` on a new
     100 ms interval, because pushing to the radio and USB tx queues is not
     interrupt safe.
  3. `Gimbal::selfTest()`'s two paths both block on `delay()` with the same
     1000/2000 ms values as before.

## Testing Checklist
- [x] Code compiles successfully for Teensy 4.1.

  `pio run -e teensy41` → SUCCESS, no warnings. FLASH code 44244 / data 9616;
  RAM1 variables 21152, code 41304; RAM2 variables 12416.

- [x] Verified working on hardware.
