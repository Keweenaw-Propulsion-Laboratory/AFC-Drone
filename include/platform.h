#pragma once

#include <Arduino.h>

/**
 * Core-compatibility shims for targets other than the Teensy 4.1.
 *
 * The flight computer is a Teensy and nothing here is used when building for
 * it - every declaration below is conditional. The purpose is to let the same
 * sources build for a bench target that has a USB connection and nothing else,
 * so the protocol, config and state-machine code can be exercised away from
 * the airframe.
 *
 * This header is force-included ahead of every translation unit in src/ (see
 * build_src_flags), because the USB macro below has to be cleared before any
 * of this project's headers are parsed.
 */

/**
 * ST's CMSIS device headers define USB as the USB peripheral base pointer:
 *
 *     #define USB ((USB_TypeDef *) USB_BASE)
 *
 * which collides with this project's USB namespace and breaks every USB::
 * call site. Nothing here drives the USB peripheral directly - on a Nucleo-64
 * the serial link is the ST-LINK's virtual COM port, which is USART2 - so
 * discarding the macro costs nothing.
 */
#ifdef USB
#undef USB
#endif

#if !defined(TEENSYDUINO)

/**
 * Teensy's core ships elapsedMillis and elapsedMicros; no other Arduino core
 * does. Both are thin wrappers over a captured timestamp, and both wrap
 * correctly at the 32-bit rollover because the subtraction is done in unsigned
 * arithmetic.
 */
class elapsedMillis {
public:
    elapsedMillis() : since(millis()) {}
    elapsedMillis(unsigned long value) : since(millis() - value) {}
    operator unsigned long() const { return millis() - since; }
    elapsedMillis &operator=(unsigned long value) { since = millis() - value; return *this; }
    elapsedMillis &operator-=(unsigned long value) { since += value; return *this; }
    elapsedMillis &operator+=(unsigned long value) { since -= value; return *this; }

private:
    unsigned long since;
};

class elapsedMicros {
public:
    elapsedMicros() : since(micros()) {}
    elapsedMicros(unsigned long value) : since(micros() - value) {}
    operator unsigned long() const { return micros() - since; }
    elapsedMicros &operator=(unsigned long value) { since = micros() - value; return *this; }
    elapsedMicros &operator-=(unsigned long value) { since += value; return *this; }
    elapsedMicros &operator+=(unsigned long value) { since -= value; return *this; }

private:
    unsigned long since;
};

#if defined(ARDUINO_ARCH_STM32)

class HardwareTimer; // stm32duino

/**
 * Teensy's IntervalTimer, backed by an STM32 hardware timer.
 *
 * Drone::startControlTimer() uses this to drive the 1 kHz control tick, and
 * that cadence is the whole point of the class: the control algorithm must not
 * inherit the jitter of whatever loop() happens to be doing. Matching the
 * Teensy interface keeps drone.cpp free of target conditionals.
 */
class IntervalTimer {
public:
    /** @param microseconds tick period. Returns false if the timer is taken. */
    bool begin(void (*callback)(), uint32_t microseconds);
    void end();

    /**
     * Teensy takes an 8-bit NVIC priority. STM32's Cortex-M3 implements the
     * top 4 bits only, so the value is shifted to match; the relative ordering
     * callers rely on is preserved.
     */
    void priority(uint8_t value);

private:
    HardwareTimer *timer = nullptr;
};

#endif // ARDUINO_ARCH_STM32

#endif // !TEENSYDUINO
