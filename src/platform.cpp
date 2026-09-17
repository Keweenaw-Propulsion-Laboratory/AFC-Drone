#include "platform.h"

#if defined(ARDUINO_ARCH_STM32)

#include <HardwareTimer.h>

/**
 * Timer peripheral driving the control tick.
 *
 * TIM6 and TIM7 are not available: this variant hands them to tone() and to
 * the Servo library respectively, and the gimbal and motors both use Servo.
 * TIM5 is used only for its update interrupt and never drives an output
 * compare channel, so it does not conflict with anything on the pin map.
 */
#ifndef CONTROL_TIMER_INSTANCE
#define CONTROL_TIMER_INSTANCE TIM5
#endif

bool IntervalTimer::begin(void (*callback)(), uint32_t microseconds) {
    if (timer != nullptr || callback == nullptr || microseconds == 0) {
        return false;
    }

    // Function-local statics rather than a heap allocation, so the control
    // timer does not depend on the allocator being in a sane state. One
    // instance is all the vehicle needs; a second begin() is refused rather
    // than silently retargeting the first one's timer.
    static HardwareTimer hardwareTimer(CONTROL_TIMER_INSTANCE);
    static bool claimed = false;

    if (claimed) {
        return false;
    }
    claimed = true;

    timer = &hardwareTimer;
    timer->setOverflow(microseconds, MICROSEC_FORMAT);
    timer->attachInterrupt(callback);
    timer->resume();
    return true;
}

void IntervalTimer::end() {
    if (timer == nullptr) {
        return;
    }
    timer->pause();
    timer->detachInterrupt();
}

void IntervalTimer::priority(uint8_t value) {
    if (timer == nullptr) {
        return;
    }
    // Cortex-M3 on this part implements the top 4 bits of the priority field,
    // so Teensy's 0-255 scale maps onto 0-15 by discarding the low nibble.
    timer->setInterruptPriority(value >> 4, 0);
}

#endif // ARDUINO_ARCH_STM32
