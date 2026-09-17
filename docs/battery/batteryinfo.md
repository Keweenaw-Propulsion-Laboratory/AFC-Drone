## Battery Monitor Use Case

The Battery Monitor module tracks the voltage of the 4S LiPo pack powering the flight
computer and classifies it into a coarse state that flight software can act on.
It samples a Teensy analog input at 12-bit resolution through a 5:1 resistor divider,
converts the reading to pack voltage, smooths it, and maps it onto a percentage across
a fixed voltage window (13.09V to 16.50V). Those levels become discrete `BatteryStates` values (`MAXIMUM`,
`MIDPOINT`, `LOW_BATTERY`, `CRITICAL_BATTERY`, or `FAULT_ERROR`).

`Battery::update()` is called every pass from `loop()` and self-throttles its ADC
sampling; `Battery::getVoltage()` returns the cached rolling average rather than
triggering a fresh conversion, so voltage, percentage, and state are always mutually
consistent within one sample.

> [!IMPORTANT]
> **`BATTERY_PIN` is currently `-1` (unassigned).** Until a real analog pin is set in
> [`src/battery.cpp`](../../src/battery.cpp), `Battery::setup()` reports the problem
> over USB and leaves the module uninitialized. In that state `getVoltage()` returns
> `0.0V`, `setupComplete()` returns false, and `getBatteryState()` returns
> `FAULT_ERROR`. No real battery data is produced.

## Hardware

The Teensy 4.1 ADC reference is **fixed at 3.3V** — `analogReference()` is a no-op on
this core. A 4S pack reaches 16.8V fully charged, so it is measured through a
**5:1 resistor divider** built from 10k resistors:

```
   Pack +
      |
     10k  ┐
      |   |
     10k  |
      |   ├─ R1 = 40k  (four 10k in series)
     10k  |
      |   |
     10k  ┘
      |
      ├────────────>  ANALOG PIN      3.30 V when the pack is at 16.50 V
      |
     10k  ─ R2 = 10k
      |
     GND  (common with Pack -)
```

* **R1 (high side):** 40kΩ — four 10kΩ resistors in series
* **R2 (low side):** 10kΩ
* **Ratio:** (R1 + R2) / R2 = 50k / 10k = **5.00:1**

| Quantity | Value | Note |
| :--- | :--- | :--- |
| Divider output at 16.50V | 3.300V | Exactly the ADC reference |
| Maximum measurable pack voltage | 16.50V | 3.3V × 5.00 |
| ADC resolution at the pack | ~4.03mV per LSB | 3.3V / 4095, referred through the divider |
| Quiescent drain | 330µA at 16.5V | ~7.9mAh per day, continuous |
| Source impedance seen by the ADC | 8.0kΩ | R1 ∥ R2 |

### Why `V_REF = 16.5f` is correct here

`getVoltage()` computes:

```
voltage = (raw / ADC_STEPS) * V_REF
```

With this divider the single `V_REF = 16.5f` constant is numerically right, because
3.3V × 5.00 = 16.5V exactly. It is still two physical quantities folded into one
number — the ADC reference and the divider ratio — so it cannot be trimmed against the
resistors actually fitted, and changing the pack chemistry or the divider silently
changes the ADC scaling. Splitting it into `ADC_REF_V = 3.3f` and
`DIVIDER_RATIO = 5.0f` would keep the same result while making each adjustable on its
own.

### Consequences to be aware of

**A fully charged pack saturates the ADC.** At 16.8V the divider puts 3.36V on the pin,
above the 3.3V reference, so the reading pins at 4095 and reports 16.50V. The top
0.3V of charge is invisible, and every pack between 16.50V and 16.80V reads identically.
This also means the over-voltage half of the fault check (`> 16.91V`) can never fire —
the conversion cannot produce a value that high. Verify 3.36V is within the analog
pin's absolute maximum rating for the fitted Teensy before flight; if more headroom is
wanted, raise the ratio (for example 6:1 from 50k/10k) and adjust the scaling constant
to match.

**Use 1% resistors or better.** The code assumes exactly 5.00:1, so any divider error
appears directly as a voltage error:

| Resistor tolerance | Worst-case ratio | A 16.50V pack reads |
| :--- | :--- | :--- |
| 5% | 5.42:1 | 15.22V (−7.8%) |
| 1% | 5.08:1 | 16.24V (−1.6%) |
| 0.1% | 5.01:1 | 16.47V (−0.2%) |

With 5% parts a full pack can be misreported as `MIDPOINT`, since the state bands are
only ~0.68V wide. Stacking four resistors for R1 helps slightly — independent
tolerances partly average out — but it does not substitute for specifying the part.
After assembly, measure the pack with a meter and compare against the reported voltage;
if they disagree, trim the scaling constant rather than assuming the nominal ratio.

**The 8kΩ source impedance is high for a 12-bit conversion.** The ADC's sampling
capacitor has to charge through that impedance within the sample window, and high
source impedance shows up as readings that sag toward zero. A small capacitor
(0.1µF is typical) across R2 gives the sampler a local charge reservoir and also
low-pass filters switching noise from the ESCs. This is a recommendation, not a
measured requirement — if it is omitted, check the reported voltage against a meter
before trusting it.

**The divider draws current whenever the pack is connected.** 330µA is negligible in
flight but never switches off; a pack left connected for a week loses roughly 55mAh.
Disconnect the pack for storage.

## Battery Voltage Specifications & Safety Guide

This implementation monitors a standard **4S LiPo battery** configured with custom
operating thresholds mapped to preserve battery health.

* **Nominal Voltage:** 14.8V (3.70V per cell) — reference only, not used in any calculation
* **Maximum Voltage (100%):** 16.5V (4.125V per cell)
* **Minimum Mapped Voltage (0%):** 13.09V (3.27V per cell)
* **Fault Tolerance:** Readings below approximately 12.76V or above 16.91V are treated
  as `FAULT_ERROR`, using a 2.5% margin around the mapped voltage limits. Normal
  bounds are 13.09–16.50V.

### Capacity Mapping Reference Table

The percentage below is **position within the mapped voltage window**, not true state
of charge. 0% is the bottom of the window, not an empty pack — 3.27V per cell still
holds usable charge. The mapping is linear in voltage, while a LiPo's real discharge
curve is not, so these figures will not match the state-of-charge chart alongside this
document.

| Mapped Capacity (%) | Total 4S Pack Voltage | Per-Cell Voltage | System Logic State (`BatteryStates`) |
| :--- | :--- | :--- | :--- |
| **60.1% – 100.0%** | **15.15V – 16.50V** | 3.79V – 4.13V | `MAXIMUM` |
| **40.1% – 60.0%** | **14.46V – 15.14V** | 3.62V – 3.78V | `MIDPOINT` |
| **20.1% – 40.0%** | **13.78V – 14.45V** | 3.45V – 3.61V | `LOW_BATTERY` |
| **0.0% – 20.0%** | **13.09V – 13.77V** | 3.27V – 3.44V | `CRITICAL_BATTERY` |
| **Out of Bounds** | **< 12.76V or > 16.91V** | N/A | `FAULT_ERROR` |

`MAXIMUM` names the top band, not a full charge — a pack at 61% of the window reports
`MAXIMUM`.

`Battery::getPercent()` returns this figure **unclamped**. A voltage inside the fault
tolerance but below 13.09V produces a negative percentage, down to about −9.6% at the
lower fault threshold. Callers that need a bounded 0–100 value must clamp it
themselves. `getBatteryState()` is unaffected, because it range-checks the voltage
before mapping it.

### Smoothing

`update()` folds each sample into an exponential moving average with a 16-sample
weight (`VOLTAGE_ALPHA = 1/16`) rather than reporting instantaneous readings. A LiPo
sags substantially under throttle, and an unfiltered sample would swing the reported
state on every load change.

The filter weight is defined in **samples**, so the resulting time constant depends
entirely on how often `update()` actually takes a reading. `loop()` is free-running and
calls `update()` every pass, so `update()` self-throttles to one ADC conversion per
`MIN_SAMPLE_PERIOD_MS` (10 ms). Sixteen samples at that rate gives a filter time
constant of roughly 160 ms — slow enough to ride out throttle transients, fast enough
to react to a genuine pack failure well inside the 100 ms telemetry interval.

There is **no hysteresis** on the state thresholds. A pack sitting on a boundary will
alternate between two adjacent states on successive calls. Anything that triggers an
irreversible action on `LOW_BATTERY` or `CRITICAL_BATTERY` should debounce the state
itself.

### Critical Health Guidelines

* **Safe Operating Target:** Maintain battery levels within the `MAXIMUM` or `MIDPOINT`
  states during standard flight operations.
* **Low Battery Alert:** Entering `LOW_BATTERY` (13.78V–14.45V / 20%–40%) signals that
  flight operations should wrap up and the drone should prepare to land immediately.
* **Critical Battery Boundary:** Discharging into `CRITICAL_BATTERY` (13.77V and below
  / 20% or less) causes harmful stress to cell health and reduces overall cycle life.
* **Fault Handling:** Voltage readings more than 2.5% below 13.09V or above 16.50V
  return `FAULT_ERROR` to flag hardware disconnects or invalid sensor readings.
  Readings within that tolerance remain `CRITICAL_BATTERY` or `MAXIMUM` as appropriate.
  `FAULT_ERROR` means the measurement cannot be trusted — it does **not** mean the pack
  is discharged. In practice only the lower bound can trigger: the 5:1 divider saturates
  the ADC at 16.50V, so the conversion can never report the 16.91V that the upper bound
  would need. See [Hardware](#hardware).

## Integration

| Where | Call |
| :--- | :--- |
| [`src/drone.cpp`](../../src/drone.cpp) | `Battery::setup()` in the `BOOT` startup stage |
| [`src/main.cpp`](../../src/main.cpp) | `Battery::update()` every pass through `loop()` |
| [`src/radio.cpp`](../../src/radio.cpp) | `Battery::getVoltage()` into the `STATUS2` `voltage` field |
| [`src/usb.cpp`](../../src/usb.cpp) | `Battery::getVoltage()` into the USB telemetry `voltage` field |

Both telemetry fields are `uint16_t` carrying **millivolts**: the float from
`getVoltage()` is packed through `Radio::floatToFixedU()` with
`RADIO_VOLTAGE_SCALE = 1000.0f`, matching the `RADIO_*_SCALE` convention the other
fixed-point fields follow. Ground-station software divides by 1000 to display volts.

The unsigned packer clamps to 0–65535 rather than wrapping, which matters because a
negative value converted through the signed `floatToFixed()` would land near 65535 and
read as a plausible ~65 V pack. At 1 mV resolution the field covers 0–65.535 V, so a
fully charged 4S pack (16.8 V → 16800) leaves substantial headroom.

The module does not report through `ErrorHandler`; `setup()` uses `USB::sendText()`
only, so an unassigned pin is visible on the bench but does not reach the ground
station. `ErrorHandler` is entirely commented out in
[`include/error.h`](../../include/error.h) at present, so there is nothing to call yet.
