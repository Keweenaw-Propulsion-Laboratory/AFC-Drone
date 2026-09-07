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
  including `radio.h` — see [usb.h:6-9](../include/usb.h#L6-L9) for an example
  of this done correctly.
- Implementation-only includes (`Arduino.h`, `Servo.h`, driver headers) belong
  in the `.cpp`, not the header. Every include in a header is an include for
  everyone who uses that header.
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

Call sites read `Example::set(12.0f)` — the same as they do today for
`Gyro::update()`.

### Why not a class of statics

You will see `class Drone`, `class Gyro`, `class Gimbal`, and `class
ErrorHandler` in the current code with every member marked `static`. That is a
class being used as a namespace, and it costs us two things:

1. **`static` in a `.cpp` hides more than `private:` does.** A `private:` member
   still appears in the header — everyone can *see* it, changing it forces a
   rebuild of every file that includes the header, and it is only one
   `#define private public` away from being reachable. A `static` variable in a
   `.cpp` cannot be named from another file at all, and can be changed freely
   without touching anyone else's build. This is not theoretical: the
   `private:` section in `gimbal.h` is precisely what pushed the gimbal's real
   state out into public globals — see [Section 5](#5-where-state-lives).

2. **It is pure ceremony.** Every static class member must be defined a second
   time, out of line, in the `.cpp` just to satisfy the linker. See
   [drone.cpp:18-32](../src/drone.cpp#L18-L32) and
   [gyro.cpp:4-25](../src/gyro.cpp#L4-L25) — roughly thirty lines that exist
   only to repeat what the header already said, and one more place to forget to
   update. Namespaces need none of it.

### Why not `radio_`-style prefixes

This is the closer call, and worth understanding properly, because most of the
project is written this way today. `radio.cpp`, `usb.cpp`, `motor.cpp`, and
`configs.cpp` use module-prefixed free functions with file-scope `static` state
— and that is a legitimate, well-established C style that already gets the
**most important** thing right: the state is hidden in the `.cpp`. Prefixes are
not the problem in this codebase; the writable globals in
[Section 5](#5-where-state-lives) are.

The gap is genuinely small. Three things decide it:

1. **A namespace is enforced by the compiler; a prefix is only discipline.**
   This is the deciding argument, and it has already failed here once.
   [configs.cpp:42](../src/configs.cpp#L42) defines:

   ```cpp
   PersistentConfig defaults()
   ```

   That is a file-local helper — it appears in no header and is used nowhere
   else. But it is neither `static` nor prefixed, so it exports a global symbol
   named `defaults` from the object file, free to collide with anything else at
   link time. Nothing warned, because nothing can: `-Wall -Werror` has no
   opinion about naming conventions. Wrap the file in `namespace Config { ... }`
   and the identical slip lands at `Config::defaults()` — still scoped, still
   safe. **The namespace makes the safe outcome the default when someone
   forgets.** The prefix has to be remembered on every declaration, forever, by
   every member.

2. **Types get the scope for free.** [radio.h](../include/radio.h) hand-types
   the prefix into `radio_SetupStates`, `radio_LinkStates`, `radio_MessageType`,
   `radio_Header`, and `radio_Packet`, and then again at every use
   (`radio_MessageType::CONFIG`). Under a namespace these are
   `Radio::MessageType::CONFIG`, with the scope declared once at the top of the
   file. Functions, types, enums, and constants all use one mechanism instead of
   each re-spelling the prefix.

3. **Module-internal code can drop the qualification.** Inside
   `namespace Radio { }`, the module's own code calls `updateLink()` plainly,
   while every outside caller must write `Radio::updateLink()`. That asymmetry
   is useful — it makes crossing a module boundary visible at the call site,
   which is exactly where a reader should be paying attention.

**The honest cost:** `grep -rn "radio_"` finds every declaration, definition,
and call of a prefixed module in one shot, whereas `Radio::` misses the module's
own unqualified internal calls. That is a real loss. In practice it is small,
because a module's internals live in exactly one `.cpp` you can read top to
bottom — but if you are hunting every use of something, search the header for
the declaration rather than grepping for call sites.

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
readings. `Target_t` in [drone.h:7](../include/drone.h#L7) and the `StatusMsg*_t`
family in [radio.h](../include/radio.h) are correct as they are. Structs hold
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
| `enum class` type | `PascalCase` | `DroneStates`, `ConfigKey` |
| `enum class` value | `SCREAMING_SNAKE` | `READY_ARMED`, `FAULT_ERROR` |
| Wire-format struct | `PascalCase` or `snake_case_t` | `StatusMsg0_t`, `usb_header_t` |

**Do not prefix functions with the module name once they are namespaced.**
`Radio::sendMessage()`, not `Radio::radio_sendMessage()`. The namespace already
says it.

Spelling counts in a public interface. `setupTimmer`, `volitile`, and
`opperation` appear in the current code; do not add more, and fix them when you
are already editing the line.

---

## 5. Where state lives

This is the most important section in this document.

**A module owns its state. Nobody outside the module may write to it.**

Today several modules break this rule by declaring writable globals in their
headers:

```cpp
// include/gimbal.h — do not copy this pattern
extern float gimbal_pitch;
extern float gimbal_yaw;
```

Any file that includes `gimbal.h` can write `gimbal_pitch = 5.0f;`. That
compiles cleanly, produces no warning, and leaves the gimbal's idea of its own
position disagreeing with the servo's actual position. There is no way to find
out who did it.

This happened for an understandable reason: `radio.cpp` and `usb.cpp` genuinely
need to *read* the gimbal setpoint for telemetry, and the class's `private:`
section was in the way. The fix is a read accessor, not a global.

**Instead:**

```cpp
// include/gimbal.h
namespace Gimbal {
    float pitch();   // read-only accessor
    float yaw();
}
```

```cpp
// src/gimbal.cpp
namespace Gimbal {
    static float currentPitch = 0.0f;   // only this file can write it
    float pitch() { return currentPitch; }
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
  validates the input (`Gimbal::set()`, `Motor::setSpeed()`), so the module can
  clamp, apply trim, and keep its invariants.
- Never duplicate state to work around encapsulation. If you find yourself
  adding a second variable that mirrors a private one, you want an accessor.

---

## 6. Non-blocking setup

Startup runs as a **state machine**, not as a sequence of blocking calls. Look
at [Gyro::setup()](../src/gyro.cpp#L27) or
[radio_setup()](../src/radio.cpp#L65) for the shape of it:

```cpp
bool setup() {
    switch (state) {
    case SetupStates::FIRST:
        if (!doTheThing()) {
            ErrorHandler::addError(ErrorHandler::someFailure);
            return false;           // false means FATAL — go to FAULT_ERROR
        }
        state = SetupStates::SECOND;
        return true;                // true means "still working, call me again"
    // ...
    }
}

bool setupComplete() { return state == SetupStates::COMPLETE; }
```

`Drone::startup()` calls these repeatedly from `setup()` in
[main.cpp:19](../src/main.cpp#L19) until they report complete.

**The contract:**

- `setup()` returns `false` **only for unrecoverable failure**. This puts the
  whole vehicle in `FAULT_ERROR`. Returning `false` because you are simply not
  done yet will ground the vehicle.
- `setup()` returns `true` to mean "no error so far" — it does **not** mean
  finished. Finished is `setupComplete()`.
- **No `delay()` in setup.** To wait, record `millis()` into a `static`
  variable, advance to a waiting state, and compare on the next call. See
  [radio.cpp:76-89](../src/radio.cpp#L76-L89) for the reset-timing example.
- Anything that waits on *another vehicle or operator* must not be part of
  `setupComplete()`. Losing the base station must never prevent the drone from
  reaching `READY_ARMED` — this is why the radio link handshake is tracked
  separately in `radio_linkConnected()`.
- No physical actuation during startup. Setup brings up electronics only.

---

## 7. The main loop contract

`loop()` in [main.cpp](../src/main.cpp) does two different jobs, and you need to
know which one your code belongs to.

**Every pass (as fast as possible):** `radio_update()`, `usb_update()`,
`Gyro::update()`. I/O servicing and sensor polling. These run continuously so
communications stay responsive.

**Every control tick (exactly 1000 µs):** `Drone::update()`. Flight control —
reading the active target, running the control algorithm, commanding the gimbal
and motors.

The tick is flagged by a hardware timer ISR and consumed by `loop()`, so the
control algorithm always runs in normal context at a jitter-free cadence.

**Rules for anything called from the control tick:**

- **Never call `delay()`.** Not for a millisecond, not for a microsecond.
- **No blocking I/O.** No `while (!Serial)`, no spin-waiting on a sensor, no
  EEPROM writes. `config_save()` blocks for milliseconds — this is why
  configuration changes are rejected during `FLIGHT`
  ([configs.cpp:212](../src/configs.cpp#L212)).
- **Bound your work.** No loop whose iteration count depends on incoming data
  without a hard cap.
- **Budget:** the whole tick must fit in 1000 µs. `Drone::worstTime` and
  `Drone::missedTicks` are reported in telemetry — watch them after your change.
  A rising `missedTicks` means the control algorithm is overrunning.
- Prefer `float` over `double`. The Teensy 4.1's Cortex-M7 does have a
  double-precision FPU, so `double` is not catastrophic here the way it is on
  an AVR — but single precision is still fewer cycles (markedly so for divide
  and square root), uses half the memory, and is what the sensor reports and
  wire formats already use. Write `0.1f`, not `0.1`; an unsuffixed literal
  promotes the whole expression to `double`.

If you need to do something slow and periodic (logging, EEPROM, a long
computation), do it from the every-pass section gated on `millis()`, not from
the control tick.

---

## 8. Interrupt service routines

We currently have one ISR: `Drone::onControlTick()` at
[drone.cpp:181](../src/drone.cpp#L181). Read it before writing another — it is
deliberately five lines long.

**Rules:**

- An ISR sets a flag and returns. That is all. The real work happens in
  `loop()`.
- **Never** call I2C, SPI, `Serial`, `delay()`, `millis()`-dependent logic, or
  anything that allocates from an ISR. These are not reentrant and will
  deadlock or corrupt state.
- Any variable shared between an ISR and normal code **must** be `volatile`, or
  the compiler will cache it in a register and your flag will never appear to
  change. See `Drone::controlTick` and `Drone::missedTicks`
  ([drone.h:55-59](../include/drone.h#L55-L59)).
- `volatile` prevents caching. It does **not** make an operation atomic. A
  `volatile uint32_t` counter incremented in an ISR and read in `loop()` is
  fine on a 32-bit core; a 64-bit value or a multi-field struct is not — you
  would need to disable interrupts around the access.
- Keep ISR priority deliberate. The control timer runs at priority 64, ahead of
  the default 128, so peripheral interrupts cannot delay the control tick.

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
- Everything is **little-endian**, matching the Teensy's native order.
- **Assert the size.** `usb.cpp` does this well
  ([usb.cpp:162-178](../src/usb.cpp#L162-L178)):

  ```cpp
  static_assert(sizeof(usb_header_t) == 4, "USB header wire size changed");
  ```

  A `static_assert` turns a silent protocol break into a compile error. Add one
  for every new wire struct.
- Radio payloads are **exactly 8 bytes**. USB payloads are at most **60 bytes**.
  Pad with an explicit `empty` field rather than leaving slack.
- Changing an existing wire struct is a **breaking change**. Update
  [docs/dashboard-protocol.md](dashboard-protocol.md) and
  [Radio_API.md](../Radio_API.md) in the same pull request, and say so in the PR
  description so the ground-station software is updated in step.

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

Both styles exist in the code today. `motor.cpp` and `usb.cpp` use `constexpr`;
`gimbal.cpp` and `drone.cpp` still use `#define`. New code uses `constexpr`.

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

Failures that the operator needs to know about go through `ErrorHandler`, not
only through `usb_send_text()`. Debug text does not reach the ground station and
is lost on reboot.

```cpp
if (!radio.init()) {
    ErrorHandler::addError(ErrorHandler::radioInitFail);
    usb_send_text("Radio start failed");   // human-readable, for the bench
    return false;
}
```

Add new error codes as `static constexpr Error` members of `ErrorHandler`
([error.h:32-33](../include/error.h#L32-L33)), with a unique code and a
severity:

| Severity | Meaning | Example |
| ---: | --- | --- |
| 0 | Low | A dropped telemetry packet |
| 1 | Medium | Radio init failed; vehicle still safe |
| 2 | High | Sensor giving implausible data |
| 3 | Critical | Loss of control authority |

There are several `// TODO add error` comments in the code
(e.g. [gyro.cpp:33](../src/gyro.cpp#L33)). Filling these in is a good first
contribution.

---

## 12. Comments

The bar is: **comments explain *why*, code explains *what*.**

```cpp
// Bad — restates the code
i++;  // increment i

// Good — explains a decision the code cannot show
// begin_I2C() leaves the bus at Teensy's default 100 kHz; the BNO08x is
// polled every main-loop iteration, so bump to 400 kHz Fast Mode (the
// documented safe max) to keep that poll cheap.
Wire.setClock(400000);
```

The second example is real, from [gyro.cpp:37-40](../src/gyro.cpp#L37-L40). It
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

The project builds with `-Wall -Werror` on `build_src_flags`
([platformio.ini](../platformio.ini)), which means **every warning in `src/` is
a build failure.** This is deliberate. The warnings that `-Wall` catches —
sign-comparison mistakes, uninitialized reads, unused results — are exactly the
class of bug that produces a vehicle that flies wrong rather than one that
fails to boot.

Do not silence a warning with a cast until you understand what it is telling
you. The narrowing-conversion fix in
[motor.cpp:30-37](../src/motor.cpp#L30-L37) is a good example of taking a
warning seriously: clamping in signed space *before* narrowing, because
narrowing first would wrap a small negative throttle up to near-maximum.

The flags apply only to `src/`, not to third-party libraries in `.pio/`. Never
edit anything under `.pio/` — it is regenerated from `lib_deps`.

---

## 14. Git and pull requests

- **Branch from `master`.** Name it `Feature/short-description`,
  `Improvement/short-description`, or `fix/short-description`, matching what is
  already in the history.
- **Never commit directly to `master`.**
- Commit messages use a `type: summary` prefix — `feat:`, `fix:`, `docs:`.
  Write what changed and why, not "updates".
- Fill in the [pull request template](../.github/pull_request_template.md)
  honestly. The Hardware Impact checkboxes exist because pin, PWM, and interrupt
  changes are the ones that damage hardware or drop a vehicle.
- **"Compiles successfully" is the minimum, not the goal.** If you changed
  control-loop code, report the observed `worstTime` and `missedTicks` before
  and after.
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
   [drone.cpp](../src/drone.cpp).
4. Add the periodic call to `loop()` — every pass for I/O, inside the
   `controlTick` block only if it is genuinely part of flight control.
5. Add error codes to `ErrorHandler` for anything that can fail.
6. Add a row to the module table in the [README](../README.md#code-structure).

### Adding a configuration setting

A config key touches six places. Missing one produces a setting that silently
does not persist, or worse, corrupts an existing one.

1. Add the field to `PersistentConfig` in
   [configs.h](../include/configs.h) — at the **end**, to minimise layout churn.
2. Add the key to the `ConfigKey` enum. **Appending is much safer than
   inserting** — inserting shifts every wire ID above it.
3. **Increment `CONFIG_VERSION`** in [configs.cpp:8](../src/configs.cpp#L8).
4. Add a case to `config_migrate()` so an existing vehicle's saved trim survives
   the upgrade instead of resetting to defaults.
5. Add validation cases to **both** `config_apply()` and `config_read()` — they
   are separate switches, and a missing case in either returns `INVALID_KEY`.
6. Add the default to the default-config initializer, and a row to the config
   table in the [README](../README.md#persistent-configuration).

### Adding a telemetry field

1. Extend or add a `StatusMsg*_t` in [radio.h](../include/radio.h), staying
   within the 8-byte radio payload.
2. Add a `static_assert` on its size.
3. Populate it in the matching `radio_sendStatus*()` in
   [radio.cpp](../src/radio.cpp). Read the source module through an **accessor**,
   not a global.
4. Mirror it in the USB telemetry struct in [usb.cpp](../src/usb.cpp) if the
   dashboard needs it.
5. Update [Radio_API.md](../Radio_API.md) and
   [dashboard-protocol.md](dashboard-protocol.md) in the same PR.

---

## 16. Migration status

The codebase predates this document and does not fully follow it yet. **Match
the conventions above in new code**; convert an existing module when you are
already working in it, as its own commit, separate from behaviour changes.

| Module | Current form | Work needed |
| --- | --- | --- |
| `motor` | Free functions + file-`static` | Wrap in `namespace Motor`; replace `extern` speed globals with accessors |
| `configs` | Free functions + file-`static` | Wrap in `namespace Config`; scope or `static`-qualify the leaked `defaults()` symbol |
| `radio` | Free functions + file-`static` | Wrap in `namespace Radio`; move stray types out of the odd header indentation |
| `usb` | Free functions + file-`static` | Wrap in `namespace Usb` |
| `gimbal` | Class of statics **plus** `extern` globals | Convert; delete the unused private `pitch`/`yaw`; replace globals with accessors |
| `drone` | Class of statics **plus** `extern` globals | Convert; replace `drone_targ0`/`drone_targ1`/`drone_rollAvg` with accessors |
| `gyro` | Class with everything `public` | Convert; move sensor handle and raw state private, expose accessors |
| `error` | Class with nested class | Convert to `namespace ErrorHandler` |

**Known cleanup, highest value first:**

1. Remove the writable `extern` globals ([Section 5](#5-where-state-lives)).
   `radio.cpp` and `usb.cpp` only ever read them, so this is mechanical and
   low-risk.
2. Delete the dead private `Gimbal::pitch` / `Gimbal::yaw` declarations in
   [gimbal.h:35-36](../include/gimbal.h#L35-L36) — they are declared but never
   defined or used, and mislead the reader about where the real state is.
3. Mark `defaults()` in [configs.cpp:42](../src/configs.cpp#L42) `static` (or
   scope it) so it stops exporting a global symbol. A one-word change, and a
   good illustration of why [Section 3](#3-the-module-pattern) prefers a
   namespace over a naming convention.
4. Replace `#define` constants with `static constexpr`.
5. Fill in the `// TODO add error` sites.

`motor.cpp` is 63 lines and self-contained — it is the recommended first
conversion for a new member.
