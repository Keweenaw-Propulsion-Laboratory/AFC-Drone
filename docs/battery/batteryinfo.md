## Battery Monitor Use Case

The Battery Monitor module provides real-time voltage and capacity tracking for a 4S LiPo battery powering the flight computer system. 
Operating on a Teensy microcontroller configured for 12-bit ADC resolution, it reads raw analog voltage, converts it to pack voltage, and calculates the remaining battery percentage across a mapped range (13.09V to 16.50V). 
The system categorizes these capacity levels into discrete `BatteryStates` enums (`MAXIMUM`, `MIDPOINT`, `LOW_BATTERY`, `CRITICAL_BATTERY`, or `FAULT_ERROR`), 
allowing flight software to trigger automated landing warnings and safety protocols.

## Battery Voltage Specifications & Safety Guide

This implementation monitors a standard **4S LiPo battery** configured with custom operating thresholds mapped to preserve battery health.

* **Nominal Voltage:** 14.8V (3.70V per cell)
* **Maximum Voltage (100%):** 16.5V (4.125V per cell)
* **Minimum Mapped Voltage (0%):** 13.09V (3.27V per cell)

### Capacity Mapping Reference Table

| State of Charge (%) | Total 4S Pack Voltage | Per-Cell Voltage | System Logic State (`BatteryStates`) |
| :--- | :--- | :--- | :--- |
| **60.1% – 100.0%** | **15.15V – 16.50V** | 3.79V – 4.13V | `MAXIMUM` |
| **40.1% – 60.0%** | **14.46V – 15.14V** | 3.62V – 3.78V | `MIDPOINT` |
| **20.1% – 40.0%** | **13.78V – 14.45V** | 3.45V – 3.61V | `LOW_BATTERY` |
| **0.0% – 20.0%** | **13.09V – 13.77V** | 3.27V – 3.44V | `CRITICAL_BATTERY` |
| **Out of Bounds** | **< 13.09V or > 16.50V** | N/A | `FAULT_ERROR` |

### Critical Health Guidelines

* **Safe Operating Target:** Maintain battery levels within the `MAXIMUM` or `MIDPOINT` states during standard flight operations.
* **Low Battery Alert:** Entering `LOW_BATTERY` (13.78V–14.45V / 20%–40%) signals that flight operations should wrap up and the drone should prepare to land immediately.
* **Critical Battery Boundary:** Discharging into `CRITICAL_BATTERY` (13.77V and below / 20% or less) causes harmful stress to cell health and reduces overall cycle life.
* **Fault Handling:** Calculations resulting in percentages outside the 0.0%–100.0% range return `FAULT_ERROR` to flag hardware disconnects or invalid sensor readings.