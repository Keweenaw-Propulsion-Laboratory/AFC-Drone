# AFC Drone — Code Conventions

This document describes **how to write code** for the AFC Drone flight
computer. The [README](../README.md) describes *what the system does*; this
describes *how we build it*.

If you are new to the team, read [Sections 1–4](#1-what-you-are-writing-code-for)
before your first pull request. The rest is reference material — come back to it
when you hit that topic.

**Contents**

1. [What you are writing code for](#1-what-you-are-writing-code-for)
2. [Module layout](#2-module-layout)
3. [The module pattern](#3-the-module-pattern)
4. [Naming](#4-naming)
5. [Where state lives](#5-where-state-lives)
6. [Non-blocking setup](#6-non-blocking-setup)
7. [The main loop contract](#7-the-main-loop-contract)
8. [Interrupt service routines](#8-interrupt-service-routines)
9. [Wire formats](#9-wire-formats)
10. [Constants and magic numbers](#10-constants-and-magic-numbers)
11. [Errors](#11-errors)
12. [Comments](#12-comments)
13. [Building and warnings](#13-building-and-warnings)
14. [Git and pull requests](#14-git-and-pull-requests)
15. [Recipes](#15-recipes)
16. [Migration status](#16-migration-status)

---

## 1. What you are writing code for

The flight computer is a **Teensy 4.1** (600 MHz Cortex-M7) running the Arduino
framework via PlatformIO. Three facts drive nearly every convention below:

- **There is exactly one of each thing.** One gimbal, one IMU, one radio, one
  vehicle. Nothing is ever created or destroyed at runtime.
- **The control loop is hard real-time.** It ticks at 1 kHz — one tick every
  1000 microseconds. Anything that blocks, waits, or takes an unpredictable
  amount of time will make the vehicle miss ticks.
- **There is no operating system to catch you.** A null dereference, a buffer
  overrun, or a stack overflow does not print a friendly error. The vehicle
  reboots, or worse, keeps flying with corrupt state.

The same sources also build for a **Nucleo-L152RE bench target** (32 MHz
Cortex-M3, **no FPU**) with nothing attached but the ST-LINK's USB serial port.
It exists so the protocol, configuration and state-machine code can be tested
away from the airframe. Your code has to compile and behave on both targets; see
[Section 13](#13-building-and-warnings).

We are writing C++, but a specific *dialect* of it. We do not use:

| Feature | Why not |
| --- | --- |
| `new` / `delete` / `malloc` | Heap fragmentation and unbounded allocation time. All memory is static or stack. |
| `virtual` / inheritance | Costs a vtable indirection per call and buys nothing when there is one implementation. |
| `std::vector`, `std::string`, `std::function` | All allocate. Use fixed-size arrays, `char[]`, and plain function pointers. |
| Exceptions / RTTI | Disabled on embedded targets; code size and non-deterministic unwinding. |

We *do* use `enum class`, `constexpr`, references, `static_assert`, function
overloading, and namespaces. These are all compile-time features — they cost
nothing at runtime and catch mistakes before the code ever reaches the vehicle.

> **If you just took an OOP course:** the instinct to write `class Gimbal` and
> then `Gimbal gimbal;` is a good instinct in general, and wrong here. See
> [Section 3](#3-the-module-pattern).

---

## 2. Module layout

Each subsystem is one header/source pair, named after the subsystem:

```
include/gimbal.h    <- the public interface: what other modules may call
src/gimbal.cpp      <- the implementation and all of the module's private state
```

The header answers *"what can I do with the gimbal?"* It should be short enough
to read in a minute. The source file answers *"how?"* and can be as long as it
needs to be.

**Rules:**

- Every header starts with `#pragma once`. We do not use include guards.
- A header includes only what its own declarations need. If `usb.h` mentions a
  radio type only in a function signature, it *forward-declares* it rather than
  including `radio.h` — see [usb.h:11-14](../include/usb.h#L11-L14) for an
  example of this done correctly.
- Implementation-only includes (`Arduino.h`, `Servo.h`, driver headers) belong
  in the `.cpp`, not the header. Every include in a header is an include for
  everyone who uses that header. (`gimbal.h`, `gyro.h` and `radio.h` still break
  this rule — see [Section 16](#16-migration-status).)
- Do not create a new module for something that is three functions long. Add it
  to the module it belongs to.

---

## 3. The module pattern

**Every subsystem is a `namespace`, not a class.** The public interface goes in
the header inside `namespace Subsystem { ... }`. All private state and helpers
go in the `.cpp` marked `static`.

### The template

```cpp
// include/example.h
#pragma once
#include <cstdint>

namespace Example {

/** Prepares the hardware. Call once during SENSOR_SETUP. */
bool setup();

/** True once setup() has finished all of its stages. */
bool setupComplete();

/** Commands a new setpoint, in degrees. */
void set(float angle);

/** The most recently commanded setpoint, in degrees. Read-only. */
float angle();

} // namespace Example
```

```cpp
// src/example.cpp
#include "example.h"
#include "Arduino.h"

namespace Example {

// ---- private state: not visible outside this file ----
static float currentAngle = 0.0f;
static bool  ready        = false;

static float clamp(float v, float low, float high);   // private helper

// ---- public interface ----
bool setup() { /* ... */ }

float angle() { return currentAngle; }

} // namespace Example
```

Call sites read `Example::set(12.0f)`, the same as `Gyro::update()` or
`Radio::setupComplete()` do today. Every module in `src/` now follows this
pattern.

### Why not a class of statics

Earlier versions of `Drone`, `Gyro` and `Gimbal` were classes with every member
marked `static`. That is a class being used as a namespace, and it cost us two
things:

1. **`static` in a `.cpp` hides more than `private:` does.** A `private:` member
   still appears in the header — everyone can *see* it, changing it forces a
   rebuild of every file that includes the header, and it is only one
   `#define private public` away from being reachable. A `static` variable in a
   `.cpp` cannot be named from another file at all, and can be changed freely
   without touching anyone else's build. This was not theoretical: the
   `private:` section in the old `gimbal.h` is precisely what pushed the
   gimbal's real state out into public globals — see
   [Section 5](#5-where-state-lives).

2. **It is pure ceremony.** Every static class member must be defined a second
   time, out of line, in the `.cpp` just to satisfy the linker — dozens of lines
   that exist only to repeat what the header already said, and one more place
   to forget to update. Namespaces need none of it.

### Why not `radio_`-style prefixes

`radio.cpp`, `usb.cpp`, `motor.cpp` and `configs.cpp` were originally written
with module-prefixed free functions (`radio_update()`, `config_save()`) and
file-scope `static` state. That is a legitimate C style, and it already got the
**most important** thing right: the state was hidden in the `.cpp`. The gap to a
namespace is small, but three things decide it:

1. **A namespace is enforced by the compiler; a prefix is only discipline.**
   This is the deciding argument, and it failed here once. `configs.cpp` defined
   a file-local helper as:

   ```cpp
   PersistentConfig defaults()
   ```

   It appeared in no header and was used nowhere else, but it was neither
   `static` nor prefixed, so it exported a global symbol named `defaults` that
   was free to collide with anything else at link time. Nothing warned, because
   nothing can: `-Wall -Werror` has no opinion about naming conventions. Now
   that the file is wrapped in `namespace Configs { ... }`, the identical slip
   lands at `Configs::defaults()` ([configs.cpp:84](../src/configs.cpp#L84)) —
   still scoped, still safe. **The namespace makes the safe outcome the default
   when someone forgets.**

2. **Types get the scope for free.** The old `radio.h` hand-typed the prefix
   into `radio_SetupStates`, `radio_MessageType` and so on, and again at every
   use (`radio_MessageType::CONFIG`). Under a namespace these are
   `Radio::MessageType::CONFIG`, with the scope declared once.

3. **Module-internal code can drop the qualification.** Inside
   `namespace Radio { }`, the module's own code calls `updateLink()` plainly,
   while every outside caller must write `Radio::updateLink()`. That asymmetry
   makes crossing a module boundary visible at the call site, which is exactly
   where a reader should be paying attention.

**The honest cost:** `grep -rn "Radio::"` misses the module's own unqualified
internal calls. In practice that is small, because a module's internals live in
exactly one `.cpp` you can read top to bottom — but if you are hunting every use
of something, search the header for the declaration rather than grepping for
call sites.

### Reopen the namespace in the `.cpp`; never `using namespace`

The `.cpp` must wrap its contents in `namespace Radio { ... }`, the same way the
header does. Do **not** write `using namespace Radio;` at the top and then
define things unqualified. This is not a style preference — the two do
completely different things, and the second one is silently wrong.

A using-directive only makes existing names *visible*. It does not let you
*define* anything in the namespace. So this:

```cpp
// radio.h
namespace Radio { void update(); }

// radio.cpp
#include "radio.h"
using namespace Radio;

void update() { }        // defines ::update, NOT Radio::update
```

...compiles cleanly under `-Wall -Werror` — nothing is wrong yet, since you are
allowed to declare a global function called `update`. `nm` on the object file
shows `T update()`, not `T Radio::update()`. `Radio::update` is now declared and
never defined, and you find out at **link** time, from an error that points at
some unrelated caller:

```
undefined reference to `Radio::update()'
```

This is exactly the failure this codebase hit during the namespace migration.
Reopening the namespace properly buys three things:

1. **The symbol always lands in the namespace.** When you slip, the mistake
   stays scoped instead of leaking a global symbol that is free to collide with
   anything else at link time — the same hazard as the unprefixed, non-`static`
   `defaults()` described above.

2. **Mismatches against the header become compile errors, in the right file.**
   Given `bool setup();` and `extern int counter;` in the header, definitions
   inside `namespace Radio { }` that get the return type or the variable type
   wrong fail immediately:

   ```
   error: ambiguating new declaration of 'void Radio::setup()'
   error: conflicting declaration 'float Radio::counter'
   ```

   The identical two mistakes under `using namespace Radio;` compile with exit
   code 0 and quietly emit global `::setup` and `::counter`.

   The limit: this catches differences that are not *overloadable* — return
   type, variable type, `const`ness. A parameter-type slip (`set(int)` where the
   header says `set(float)`) is a legal overload, so it still compiles and you
   still get a link error at the call site. Better than a stray global, but not
   caught at the definition.

3. **The file reads as the module's implementation.** Opening
   `namespace USB {` right after the includes — see
   [usb.cpp:18](../src/usb.cpp#L18) — says plainly "everything below is USB,"
   and the module's private helpers are scoped without anyone having to
   remember a prefix.

`using namespace` is fine in the narrow, local sense it was designed for — a
`using namespace std::chrono_literals;` inside a function body, say. It is never
the way to implement a module.

### Why not real objects

Making `Gimbal` a class you instantiate (`Gimbal gimbal;`) would add a level of
indirection and a static-initialization-order hazard in exchange for the ability
to have two gimbals, which the vehicle will never have. Classes earn their keep
when you need many instances or swappable implementations. We need neither.

**There is no performance difference between any of these three spellings.** A
`static` member function, a namespaced function, and a bare C function compile
to byte-identical machine code — same call, no `this` pointer, no vtable. We are
choosing between them purely on how easy the code is to read and how hard it is
to misuse. Do not "optimize" by rewriting one as another.

### When a `struct` *is* the right answer

Use a plain `struct` for **data that travels** — packets, setpoints, sensor
readings. `Target_t` and `Telemetry_t` in [drone.h:8](../include/drone.h#L8) and
the `StatusMsg*_t` family in [radio.h](../include/radio.h) are correct as they
are. Structs hold
data; namespaces hold behavior.

---

## 4. Naming

| Thing | Style | Example |
| --- | --- | --- |
| Namespace / module | `PascalCase` | `Gimbal`, `Radio`, `Gyro` |
| Function | `camelCase` | `setupComplete()`, `sendStatus0()` |
| Local variable | `camelCase` | `bottomSpeed`, `startTime` |
| File-scope `static` | `camelCase` | `static float rollingRssi;` |
| Compile-time constant | `SCREAMING_SNAKE` | `ESC_MIN_US`, `CONTROL_LOOP_HZ` |
| `enum class` type | `PascalCase` | `Drone::States`, `ConfigKey` |
| `enum class` value | `SCREAMING_SNAKE` | `READY_ARMED`, `FAULT_ERROR` |
| Wire-format struct | `PascalCase` or `snake_case_t` | `StatusMsg0_t`, `header_t` |

**Do not prefix functions with the module name once they are namespaced.**
`Radio::sendMessage()`, not `Radio::radio_sendMessage()`. The namespace already
says it.

Spelling counts in a public interface. The misspelled identifiers
(`setupTimmer`, `opperation`) have been fixed; a few remain in comments
(`timmer`, `gaurd`, `fllight`). Do not add more, and fix them when you are
already editing the line.

---

## 5. Where state lives

This is the most important section in this document.

**A module owns its state. Nobody outside the module may write to it.**

Several modules used to break this rule by declaring writable globals in their
headers:

```cpp
// old include/gimbal.h — do not copy this pattern
extern float gimbal_pitch;
extern float gimbal_yaw;
```

Any file that included `gimbal.h` could write `gimbal_pitch = 5.0f;`. That
compiles cleanly, produces no warning, and leaves the gimbal's idea of its own
position disagreeing with the servo's actual position. There is no way to find
out who did it.

This happened for an understandable reason: `radio.cpp` and `usb.cpp` genuinely
need to *read* the gimbal setpoint for telemetry, and the class's `private:`
section was in the way. The fix is a read accessor, not a global, and that is
what the code does now:

```cpp
// include/gimbal.h
namespace Gimbal {
    float getPitch();   // read-only accessor
    float getYaw();
}
```

```cpp
// src/gimbal.cpp
namespace Gimbal {
    static float currentPitch = 0.0f;   // only this file can write it
    float getPitch() { return currentPitch; }
}
```

The accessor compiles down to a single load instruction — identical to reading
the global — but now the only code that can *change* the value is the module
that is responsible for it.

**Rules:**

- No `extern` on a mutable variable in a header. Ever.
- `extern const` and `constexpr` in a header are fine — they cannot be written.
- If another module needs to read your state, add an accessor function.
- If another module needs to *change* your state, add a command function that
  validates the input (`Gimbal::set()`, `Motor::setMotor()`), so the module can
  clamp, apply trim, and keep its invariants.
- Never duplicate state to work around encapsulation. If you find yourself
  adding a second variable that mirrors a private one, you want an accessor.

---

## 6. Non-blocking setup

Startup runs as a **state machine**, not as a sequence of blocking calls. Look
at [Gyro::setup()](../src/gyro.cpp#L77) or
[Radio::setup()](../src/radio.cpp#L79) for the shape of it:

```cpp
bool setup() {
    switch (state) {
    case SetupStates::FIRST:
        if (!doTheThing()) {
            USB::sendText("Thing failed");
            return false;           // false means FATAL — go to FAULT_ERROR
        }
        state = SetupStates::SECOND;
        return true;                // true means "still working, call me again"
    // ...
    }
}

bool setupComplete() { return state == SetupStates::COMPLETE; }
```

`setup()` in [main.cpp:15](../src/main.cpp#L15) calls `Drone::startup()`
repeatedly, and each startup stage calls its modules' `setup()` until they report
complete. Startup ends in `SAFE`.

**The contract:**

- `setup()` returns `false` **only for unrecoverable failure**. This puts the
  whole vehicle in `FAULT_ERROR`. Returning `false` because you are simply not
  done yet will ground the vehicle.
- `setup()` returns `true` to mean "no error so far" — it does **not** mean
  finished. Finished is `setupComplete()`.
- **No `delay()` in setup.** To wait, record `millis()` into a `static`
  variable, advance to a waiting state, and compare on the next call. See
  [radio.cpp:97-119](../src/radio.cpp#L97-L119) for the reset-timing example.
- Anything that waits on *another vehicle or operator* must not be part of
  `setupComplete()`. Losing the base station must never prevent the drone from
  finishing startup — this is why the radio link handshake is tracked
  separately in `Radio::linkConnected()` and runs in the background from
  `Radio::update()`.
- **Optional hardware must be skippable.** If a peripheral can be absent (the
  bench target has no radio and no IMU), gate its bring-up on a config flag
  checked *before* touching the bus, and have `setupComplete()` report success
  when it is disabled. See `RadioEnabled` / `GyroEnabled` and the `DISABLED`
  setup state in [radio.h](../include/radio.h).
- No physical actuation during startup. Setup brings up electronics only.

---

## 7. The main loop contract

The firmware has two execution contexts, and you need to know which one your
code belongs to.

**`loop()`, every pass (as fast as possible)** — [main.cpp](../src/main.cpp):
`Radio::update()`, `USB::update()`, `Gyro::update()`, `Battery::update()`,
`Drone::serviceWatchdog()`, `Drone::updateLEDS()`. I/O servicing, sensor polling
and bookkeeping. These run continuously so communications stay responsive.

**`loop()`, every 100 ms:** telemetry. `loop()` takes **one** snapshot with
`Drone::getTelemetry()` and hands it to `USB::sendTelemetry()` and every
`Radio::sendStatusN()`, so all packets in a frame describe the same instant.

**Control tick, every 1000 µs, in interrupt context:** `Drone::update()`. The
hardware timer ISR calls it directly — see [Section 8](#8-interrupt-service-routines)
for why. It reads the active target, runs the control algorithm, commands the
gimbal and motors, and records the telemetry snapshot. It never transmits
anything itself.

**Rules for anything called from the control tick:**

- Everything in [Section 8](#8-interrupt-service-routines) applies: no I2C,
  SPI, `Serial`, heap allocation, or pushes to the radio/USB transmit queues.
- **Never call `delay()`.** Not for a millisecond, not for a microsecond.
- **No blocking I/O.** No `while (!Serial)`, no spin-waiting on a sensor, no
  EEPROM writes. `Configs::save()` blocks for milliseconds — this is why
  configuration changes are rejected in `MAN_FLIGHT` and `AUTO_FLIGHT`
  ([configs.cpp:258](../src/configs.cpp#L258)).
- **Bound your work.** No loop whose iteration count depends on incoming data
  without a hard cap.
- **Budget:** the whole tick must fit in 1000 µs. The worst tick time
  (`Drone::getWorstTime()`, `loopTimeMax` in telemetry) and the missed-tick
  count (`Drone::getMissedTicks()`) are reported — watch them after your change.
  A rising missed-tick count means the control algorithm is overrunning.
- Prefer `float` over `double`. The Teensy 4.1's Cortex-M7 does have a
  double-precision FPU, but single precision is still fewer cycles (markedly so
  for divide and square root), uses half the memory, and is what the sensor
  reports and wire formats already use. On the Nucleo's Cortex-M3 there is no
  FPU at all, so every floating-point operation is a software call, and
  `double` is far slower again. Write `0.1f`, not `0.1`; an unsuffixed literal
  promotes the whole expression to `double`. Prefer integer math or a lookup
  table where it is easy — see `BREATHE_TABLE` in
  [drone.cpp](../src/drone.cpp).

**Rules for `loop()` code:**

- It must stay short. `loop()` is not on the control tick, but radio receive
  windows, the 100 ms watchdog and USB throughput all depend on it coming round
  quickly. No `delay()` here either.
- Anything shared with the control tick follows the masking rules in
  [Section 8](#8-interrupt-service-routines) — `Drone::setTarget()` is the
  pattern.

If you need to do something slow and periodic (logging, EEPROM, a long
computation), do it from `loop()` gated on `millis()`, the same way telemetry
is, never from the control tick.

---

## 8. Interrupt service routines

We currently have one ISR: `Drone::onControlTick()` at
[drone.cpp:82](../src/drone.cpp#L82). Read it before writing another.

It is worth knowing why it does not follow the usual advice. The textbook rule
is that an ISR sets a flag and `loop()` does the work, and this project did that
originally. It was changed deliberately: polling a flag meant the control
algorithm did not start until the current pass through radio/USB/gyro servicing
finished, which put the blocking I2C read of the BNO08x — on the order of
100 µs — straight into the control loop's jitter budget. `onControlTick()` now
calls `Drone::update()` directly, so the only variance is the timer's own
interrupt latency. **That decision is what makes the rest of this section load
bearing**, because the whole control algorithm now runs in interrupt context.

**Rules:**

- Write an ISR that sets a flag and returns unless you have the same reason
  `onControlTick()` had. If you do run real work in one, everything it reaches
  inherits every rule below — audit the whole call tree, not just the handler.
- **Never** call I2C, SPI, `Serial`, `delay()`, or anything that allocates from
  an ISR. These are not reentrant and will deadlock or corrupt state.
- **Never wait on time inside an ISR** — no spinning until `millis()` advances,
  no busy-wait on `micros()`. Nothing will preempt you to do the work you are
  waiting for, so the vehicle hangs.
- **Reading `millis()` or `micros()` is fine**, and the control tick does it:
  `recordTelemetry()` stamps `runtimeSec` ([drone.cpp:454](../src/drone.cpp#L454))
  and the comm watchdog compares an `elapsedMillis`
  ([drone.cpp:627](../src/drone.cpp#L627)). Both are plain loads of a counter
  that SysTick maintains, and the counters keep advancing during the tick
  because **SysTick outranks the control timer on both targets**: Teensy 4 sets
  SysTick to priority 32 (`SCB_SHPR3 = 0x20200000` in the core's `startup.c`)
  against the control timer's 64, and STM32L1 sets `TICK_INT_PRIORITY = 0`
  against the timer's 4. Lower number wins, so SysTick preempts the tick.
  This is a property of the priority numbers, not a guarantee — see the priority
  rule below.
- Any variable shared between an ISR and normal code **must** be `volatile`, or
  the compiler will cache it in a register and your flag will never appear to
  change. See `currentState`, `missedTicks`, and `activeTarget`
  ([drone.cpp:34](../src/drone.cpp#L34), [49](../src/drone.cpp#L49),
  [61](../src/drone.cpp#L61)).
- `volatile` prevents caching. It does **not** make an operation atomic. A
  `volatile uint32_t` counter incremented in an ISR and read in `loop()` is
  fine on a 32-bit core; a 64-bit value or a multi-field struct is not — you
  would need to disable interrupts around the access. `Drone::setTarget()`
  ([drone.cpp:310](../src/drone.cpp#L310)) is the worked example: four fields
  the tick consumes as one setpoint, so the writes are masked.
- **Mask in loop context, never in the ISR.** `loop()` cannot preempt an ISR,
  so masking inside the handler accomplishes nothing — by the time it is
  running, any half-finished access from `loop()` has already happened. The
  `loop()` side is what has to mask, whichever direction the data flows:
  `Drone::setTarget()` masks a loop-context *write* the ISR reads, and
  `Drone::getTelemetry()` masks a loop-context *read* of the snapshot the ISR
  writes.
- Keep ISR priority deliberate. The control timer runs at priority 64
  ([drone.cpp:27](../src/drone.cpp#L27)), ahead of the default 128, so
  peripheral interrupts cannot delay the control tick. **Do not raise it past
  SysTick's**: a control timer numerically below 32 on the Teensy, or 0 on the
  STM32, would stop `millis()` advancing for the duration of the tick and
  quietly freeze both the runtime stamp and the comm watchdog.

Anything touching an ISR, a pin assignment, or a timing interval must be called
out in the pull request — the [PR template](../.github/pull_request_template.md)
asks for exactly this.

---

## 9. Wire formats

Any struct that is transmitted over USB or radio, or stored in EEPROM, is a
**wire format**. The ground station and the dashboard parse these byte-for-byte.
Getting one wrong produces telemetry that is silently, plausibly wrong.

**Rules:**

- Mark every wire struct `__attribute__((packed))`. Without it the compiler
  inserts padding and the layout no longer matches what the other end expects.
- Use fixed-width types only: `uint8_t`, `int16_t`, `uint32_t`. Never `int`,
  `long`, or `float` where the size matters across the link.
- Everything is **little-endian**, matching the native order of both targets.
- **Assert the size.** `usb.cpp` does this well
  ([usb.cpp:198-216](../src/usb.cpp#L198-L216)), as do `radio.h` and
  `configs.h`:

  ```cpp
  static_assert(sizeof(header_t) == 4, "USB header wire size changed");
  ```

  A `static_assert` turns a silent protocol break into a compile error. Add one
  for every new wire struct.
- Radio payloads are **exactly 8 bytes**. USB payloads are at most **64 bytes**.
  Pad with an explicit `empty` or `reserved` field rather than leaving slack.
- Append new fields to the **end** of an existing wire struct, into its reserved
  tail. Dashboards decode by offset, so inserting a field anywhere else shifts
  every field above it.
- Enums that go on the wire get **explicit or append-only values**.
  `Drone::States` spells out every value, and `Configs::ConfigKey` is
  append-only, so a new entry never renumbers the ones an existing ground
  station already knows.
- Changing an existing wire struct is a **breaking change**. Update
  [docs/dashboard-protocol.md](dashboard-protocol.md) in the same pull request,
  and say so in the PR description so the ground-station software is updated in
  step.

---

## 10. Constants and magic numbers

Prefer `static constexpr` over `#define`:

```cpp
static constexpr int ESC_MIN_US = 1000;   // good: typed, scoped, debuggable
#define ESC_MIN_US 1000                   // avoid: no type, no scope
```

A `#define` is a blind text substitution with no type and no respect for scope.
`constexpr` costs exactly the same at runtime — zero — and the compiler will
catch you using it wrongly.

The source modules now use `constexpr` throughout. The one accepted use of
`#define` is a **build-time default** that `platformio.ini` can override with a
`-D` flag, wrapped in `#ifndef`:

```cpp
#ifndef DEFAULT_RADIO_ENABLED
#define DEFAULT_RADIO_ENABLED true
#endif
```

See `DEFAULT_RADIO_ENABLED` / `DEFAULT_GYRO_ENABLED` in
[configs.cpp](../src/configs.cpp) and `USB_BAUD` in [usb.cpp](../src/usb.cpp).
Use this only when a value genuinely differs per build target.

**No bare numbers in logic.** A number that is not obviously self-explaining
gets a name and a unit:

```cpp
if (millis() - lastPing >= 1000) { ... }               // 1000 what? why?
if (millis() - lastPing >= LINK_RETRY_MS) { ... }      // says it
```

Put the unit in the name — `_MS`, `_US`, `_HZ`, `_DBM`, `_DEG`. Unit confusion
is the single most common source of embedded bugs.

---

## 11. Errors

**Today:** report failures with `USB::sendText()`, and return `false` from a
setup stage when the failure is fatal so the vehicle goes to `FAULT_ERROR`:

```cpp
if (!radio.init()) {
    USB::sendText("Radio start failed");   // human-readable, for the bench
    return false;
}
```

Prefer the NUL-terminated `sendText(const char*)` overload for string literals;
a hand-counted length is easy to get wrong. Debug text is USB-only — it does
not reach the ground station and is lost on reboot — so a condition the
operator must see remotely belongs in telemetry as well (the latched
`TRIPPED` watchdog flag is the model).

Anything that can print repeatedly must be **throttled**. A message sent from a
tight loop saturates the USB transmit buffer and starves every other frame; see
the once-per-second `FAULT` message in `Drone::startup()`.

**Planned:** a structured `ErrorHandler` that buffers errors with a code and a
severity and reports them over both USB and radio. Its interface is sketched,
fully commented out, in [error.h](../include/error.h):

| Severity | Meaning | Example |
| ---: | --- | --- |
| 0 | Low | A dropped telemetry packet |
| 1 | Medium | Radio init failed; vehicle still safe |
| 2 | High | Sensor giving implausible data |
| 3 | Critical | Loss of control authority |

Until it exists, the `// TODO add error` comments (e.g.
[gyro.cpp:94](../src/gyro.cpp#L94)) mark the places that will report through it.

---

## 12. Comments

The bar is: **comments explain *why*, code explains *what*.**

```cpp
// Bad — restates the code
i++;  // increment i

// Good — explains a decision the code cannot show
// begin_I2C() leaves the bus at Teensy's default 100 kHz; the BNO08x
// is polled over I2C every main-loop iteration, so bump to 400 kHz
// Fast Mode (the documented safe max) to keep that poll cheap.
Wire.setClock(400000);
```

The second example is real, from [gyro.cpp:98-101](../src/gyro.cpp#L98-L101). It
tells the next person why the line exists and what constraint they would violate
by removing it. That is the standard.

**Doxygen comments on public interfaces.** Every function in a header gets one:

```cpp
/**
 * Validate, apply, and persist one setting.
 *
 * @param key   Which setting to change.
 * @param value Signed representation; booleans must be 0 or 1.
 * @return OK, or the reason the change was rejected.
 * @warning Blocks for several milliseconds when the value actually changes.
 */
ConfigResult set(ConfigKey key, int32_t value);
```

Use `@warning` for anything that blocks, runs in interrupt context, or has a
non-obvious safety consequence.

**Always document units and ranges.** "angle in degrees, −20 to +20" is useful;
"the angle" is not.

`// MARK:` comments divide long files into navigable sections; keep using them.

---

## 13. Building and warnings

```
PlatformIO: Build    (checkmark in the status bar)
PlatformIO: Upload   (arrow in the status bar)
```

[platformio.ini](../platformio.ini) defines two environments, and **both must
build** before a pull request is opened:

```sh
pio run -e teensy41        # flight computer
pio run -e nucleo_l152re   # bench target
```

The Nucleo build exists to run the protocol, configuration and state-machine
code with no flight hardware attached. Keep it working:

- **Keep target-specific code out of the modules.** Teensy-only APIs
  (`elapsedMillis`, `IntervalTimer`) are provided for other targets by shims in
  [platform.h](../include/platform.h) / [platform.cpp](../src/platform.cpp),
  which is force-included only in the Nucleo build. Add a shim there rather
  than sprinkling `#ifdef TEENSYDUINO` through `drone.cpp`.
- **Per-target defaults go in `platformio.ini`** as `-D` flags over an
  `#ifndef` default ([Section 10](#10-constants-and-magic-numbers)), e.g. the
  Nucleo build turns the radio and IMU off by default.
- Library problems that only affect one target are fixed with a build script,
  not by editing the library — see
  [scripts/stm32_skip_rh_ask.py](../scripts/stm32_skip_rh_ask.py).

Both environments build with `-Wall -Werror` on `build_src_flags`
([platformio.ini](../platformio.ini)), which means **every warning in `src/` is
a build failure.** This is deliberate. The warnings that `-Wall` catches —
sign-comparison mistakes, uninitialized reads, unused results — are exactly the
class of bug that produces a vehicle that flies wrong rather than one that
fails to boot.

Do not silence a warning with a cast until you understand what it is telling
you. The narrowing-conversion fix in
[motor.cpp:41-48](../src/motor.cpp#L41-L48) is a good example of taking a
warning seriously: clamping in signed space *before* narrowing, because
narrowing first would wrap a small negative throttle up to near-maximum.

The flags apply only to `src/`, not to third-party libraries in `.pio/`. Never
edit anything under `.pio/` — it is regenerated from `lib_deps`.

---

## 14. Git and pull requests

- **Branch from `main`.** Name it `Feat/short-description`,
  `Improvement/short-description`, or `fix/short-description`, matching what is
  already in the history.
- **Never commit directly to `main`.**
- Commit messages use a `type: summary` prefix — `feat:`, `fix:`, `docs:`.
  Write what changed and why, not "updates".
- Fill in the [pull request template](../.github/pull_request_template.md)
  honestly. The Hardware Impact checkboxes exist because pin, PWM, and interrupt
  changes are the ones that damage hardware or drop a vehicle.
- **"Compiles successfully" — for both environments — is the minimum, not the
  goal.** If you changed control-loop code, report the observed worst tick
  time (`loopTimeMax`) and missed-tick count before and after.
- If you changed a wire format, the state machine, or the heartbeat/watchdog
  behavior, update [dashboard-protocol.md](dashboard-protocol.md) and the
  [README](../README.md) in the same pull request.
- Keep pull requests small. A 200-line PR gets a real review; a 2000-line PR
  gets a rubber stamp.

---

## 15. Recipes

### Adding a new subsystem

1. Create `include/thing.h` and `src/thing.cpp` from the template in
   [Section 3](#3-the-module-pattern).
2. Give it `setup()` / `setupComplete()` if it needs hardware bring-up, written
   as a state machine ([Section 6](#6-non-blocking-setup)).
3. Add the setup call to the right stage of `Drone::startup()` in
   [drone.cpp](../src/drone.cpp). If the hardware can be absent, give it an
   enable flag in the config ([Section 6](#6-non-blocking-setup)).
4. Add the periodic call to `loop()` for I/O and polling. Call it from
   `Drone::update()` only if it is genuinely part of flight control — and then
   it runs in interrupt context ([Section 8](#8-interrupt-service-routines)).
5. Anything that moves hardware must check `Drone::getFlightWatchdogStatus()`
   and the current state before writing an output, the way `Motor::setMotor()`
   and `Gimbal::set()` do.
6. Report failures as described in [Section 11](#11-errors).
7. Make sure both environments build.
8. Add a row to the module table in the [README](../README.md#code-structure).

### Adding a configuration setting

A config key touches several places. Missing one produces a setting that
silently does not persist, or worse, corrupts an existing one.

1. Add the field to `PersistentConfig` in
   [configs.h](../include/configs.h) — at the **end**, so every existing field
   keeps its offset — and update the `static_assert` on its size.
2. **Append** the key to the `ConfigKey` enum. Never insert: the values are wire
   IDs, and inserting renumbers every key after it.
3. **Increment `CONFIG_VERSION`** in [configs.cpp:25](../src/configs.cpp#L25).
4. Freeze the outgoing layout as a `PersistentConfigVN` struct in
   `configs.cpp` (with a `static_assert` on its size), and add a case to
   `migrate()` that validates the old image under that struct and carries its
   fields forward, so an existing vehicle's saved trim survives the upgrade
   instead of resetting to defaults. The `case (2)` migration is the model.
5. Add cases to **both** `apply()` and `read()` — they are separate switches,
   and a missing case in either returns `INVALID_KEY`.
6. Add the default to `defaults()`. If it should differ on the bench target,
   use an `#ifndef` default and a `-D` flag
   ([Section 10](#10-constants-and-magic-numbers)).
7. Add the key to the config tables in
   [dashboard-protocol.md](dashboard-protocol.md) and the
   [README](../README.md#persistent-configuration).

### Adding a telemetry field

1. Add the field to `Drone::Telemetry_t` in [drone.h](../include/drone.h) and
   fill it in `recordTelemetry()` in [drone.cpp](../src/drone.cpp). This runs
   in the control ISR, so read the source module through an **accessor** and
   follow the [Section 8](#8-interrupt-service-routines) rules. Values owned by
   `loop()` context (RSSI, battery voltage, GPS) are read live by the senders
   instead.
2. Extend or add a `StatusMsg*_t` in [radio.h](../include/radio.h), staying
   within the 8-byte radio payload, with a `static_assert` on its size. Pack
   floats with `floatToFixed()` / `floatToFixedU()` and a named scale factor.
3. Populate it in the matching `Radio::sendStatusN()` in
   [radio.cpp](../src/radio.cpp) from the snapshot it is passed. A new status
   message also needs a `MessageType` value and a call in `loop()`'s telemetry
   block in [main.cpp](../src/main.cpp).
4. Mirror it in the USB `Telemetry` struct in [usb.cpp](../src/usb.cpp) if the
   dashboard needs it. That record is already 64 bytes — the full USB payload —
   so adding a field there means replacing a field or adding a new message type.
5. Update [dashboard-protocol.md](dashboard-protocol.md) in the same PR.

---

## 16. Migration status

The namespace migration is complete: every module in `src/` is a namespace with
file-`static` state, there are no writable `extern` globals left in any header,
and the source modules use `constexpr` instead of `#define`. **Match the
conventions above in new code.** When you fix one of the items below, do it in
its own commit, separate from behavior changes.

| Module | Current form | Remaining work |
| --- | --- | --- |
| `drone`, `motor`, `battery`, `usb` | Namespace + file-`static` | None |
| `configs` | Namespace + file-`static` | Mark `defaults()` ([configs.cpp:84](../src/configs.cpp#L84)) `static`; it is scoped now but still has external linkage |
| `radio` | Namespace + file-`static` | Move the `RH_RF69.h` include out of the header; re-indent the header; wire up or remove the unused `handleSetup()` |
| `gimbal` | Namespace + file-`static` | Move the `Servo.h` include out of the header |
| `gyro` | Namespace + file-`static` | Move the `Adafruit_BNO08x.h` include out of the header; publish sensor output through a double buffer so the control ISR cannot read a torn update (see the note in `recordTelemetry()`) |
| `error` | Entirely commented out | Implement as `namespace ErrorHandler` ([Section 11](#11-errors)) |

**Other known cleanup, highest value first:**

1. Implement `ErrorHandler` and fill in the `// TODO add error` sites.
2. Add radio packet-sequence checking (`// TODO` in `Radio::update()`).
3. Fix the remaining comment typos noted in [Section 4](#4-naming).
4. Add the Nucleo build to the
   [PR template](../.github/pull_request_template.md) testing checklist.
