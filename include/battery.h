#pragma once

#include <cstdint>

/**
* Battery Monitoring System
*
* Samples the 4S LiPo pack voltage through a Teensy analog input, smooths it,
* and classifies it into a coarse state the flight software can act on.
*
* The pack is measured through a 5:1 resistor divider built from 10k parts -
* 40k high side (four 10k in series) over 10k to ground. The Teensy 4.1 ADC
* reference is fixed at 3.3 V, so 3.3 V at the pin is 16.5 V at the pack, which
* is why the conversion constant is 16.5. Specify 1% resistors: the code assumes
* exactly 5.00:1 and cannot trim for the parts actually fitted.
*
* Note that a fully charged pack (16.8 V) puts 3.36 V on the pin and saturates
* the ADC, so anything above 16.5 V reads as 16.5 V. See
* docs/battery/batteryinfo.md for the full divider notes and fault thresholds.
*
* @warning BATTERY_PIN in battery.cpp is currently -1 (unassigned). Until a real
*          pin is set, setup() reports the problem over USB and leaves the module
*          uninitialized, getVoltage() returns 0.0 V, and getBatteryState()
*          reports FAULT_ERROR.
*/
namespace Battery {

    /**
     * Configure the analog input pin and set the ADC to 12-bit resolution,
     * then take one priming sample.
     *
     * Call once during startup. If BATTERY_PIN is unassigned this reports the
     * fact over USB and leaves the module uninitialized without failing.
     *
     * @return False only for unrecoverable failure, which puts the vehicle in
     *         FAULT_ERROR. Currently always returns true; an unassigned pin is
     *         reported through setupComplete(), not through this return value.
     * @warning analogReadResolution() is a global ADC setting, not per-pin. It
     *          changes the resolution seen by every analogRead() in the program.
     */
    bool setup();

    /**
     * Sample the battery and fold the reading into its rolling voltage average.
     *
     * Call every pass from loop(). Does nothing until setup() has run
     * successfully. Self-throttles so the ADC is sampled at a bounded rate
     * rather than once per loop pass; the smoothing is an exponential moving
     * average with a 16-sample weight.
     */
    void update();

    /**
     * @return True once the analog pin has been configured and the module is
     *         sampling. False if BATTERY_PIN is unassigned or setup() has not
     *         run, in which case every reading below is meaningless.
     */
    bool setupComplete();

    /**
     * The smoothed pack voltage.
     *
     * This is the rolling average maintained by update(), not an instantaneous
     * reading, so it lags a real change by roughly the filter's time constant.
     *
     * @return Pack voltage in volts, 0.0-16.5. Returns 0.0 before the first
     *         successful update().
     */
    float getVoltage();

    /**
     * Position within the mapped voltage window, as a percentage.
     *
     * Derived from getVoltage() over the 13.09 V - 16.5 V range:
     *
     *     percent = (voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100.0f
     *
     * This is not true battery state of charge. 0% is the bottom of the mapped
     * window, not an empty pack: 13.09 V is 3.27 V per cell, which still holds
     * usable charge. It is a linear map of voltage, and a LiPo's discharge curve
     * is not linear.
     *
     * @return Percentage of the mapped window. Nominally 0.0-100.0, but NOT
     *         clamped: a voltage inside the fault tolerance but below 13.09 V
     *         yields a negative value, down to about -9.6% at the lower fault
     *         threshold. Callers that need a bounded value must clamp it.
     */
    float getPercent();

    /**
     * Coarse battery state, in ascending order of remaining charge.
     *
     * FAULT_ERROR:      Reading outside the valid voltage range. Indicates a
     *                   disconnected sense line, an unassigned pin, or an
     *                   implausible reading - not a discharged pack.
     * CRITICAL_BATTERY: 0-20% of the mapped window. Harmful to cell health;
     *                   land now.
     * LOW_BATTERY:      20-40%. Wrap up flight operations and prepare to land.
     * MIDPOINT:         40-60%.
     * MAXIMUM:          60-100%. The normal in-flight state for a healthy pack;
     *                   it does NOT mean fully charged.
     */
    enum class BatteryStates : uint8_t {
        FAULT_ERROR,
        CRITICAL_BATTERY,
        LOW_BATTERY,
        MIDPOINT,
        MAXIMUM,
    };

    /**
     * Classify the current smoothed voltage.
     *
     * Range-checks the voltage before mapping it, so a reading more than 2.5%
     * outside the 13.09 V - 16.5 V window returns FAULT_ERROR rather than being
     * reported as a charge level.
     *
     * @return The matching BatteryStates value. Thresholds are on the mapped
     *         percentage from getPercent(), not on true state of charge.
     * @warning There is no hysteresis. A pack sitting on a threshold will
     *          alternate between two states on successive calls.
     */
    BatteryStates getBatteryState();
}
