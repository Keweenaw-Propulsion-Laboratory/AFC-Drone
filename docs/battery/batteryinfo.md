# 4S LiPo Battery Monitoring & Configuration

This document outlines the logic, parameters, and variable references required to implement a 4S LiPo battery level indicator and monitoring system in C.

---

## 📊 Battery Voltage Specifications & Safety Guide

The system monitors a standard **4S LiPo battery** configured with custom safe-operating thresholds for capacity mapping to preserve cycle life.

* **Nominal Voltage:** 14.8V (3.7V per cell)
* **Maximum Voltage (100%):** 16.8V (4.2V per cell)
* **Safe Discharge / Empty Limit (0%):** 14.0V (3.5V per cell)

### Capacity Mapping Reference Table

| State of Charge | Total 4S Pack Voltage | Per-Cell Voltage | System Logic State |
| :--- | :--- | :--- | :--- |
| **100%** | **16.8V** | 4.20V | Maximum Full Charge |
| **50%** | **15.4V** | 3.85V | Midpoint |
| **10%** | **14.28V** | 3.57V | **Low Battery Alert: Need to Land / Return** |
| **0%** | **14.0V** | 3.50V | **Safe Stop / Recharge Floor** |
| **Danger Zone** | **12.8V and lower** | 3.20V and lower | **Critical Battery Damage** |

### ⚠️ Critical Health Guidelines
* **Safe Discharge Limit:** Stop discharging at **3.5V–3.6V per cell** (14.0V–14.4V total for 4S) to preserve battery longevity. This maps to the 0% UI limit.
* **Health Hazard:** Avoid discharging below **3.2V per cell** (12.8V total).
* **Danger Zone:** Discharging below **3.0V per cell** (12.0V total) causes irreversible capacity loss, chemical swelling, or complete battery failure.
* **Low-Voltage Cutoff (LVC):** Ensure your Electronic Speed Controller (ESC) or firmware matches these guidelines to stop power draw automatically before entering the danger zone.

---

## ⚙️ C-Implementation Architecture

### 1. Data Acquisition & Logic
* **Data Pull:** Read the total battery pack voltage using an analog input pin on the microcontroller. Telemetry or hardware voltage checkers should be used to verify accuracy.
* **Storage:** Pull the current voltage from the battery and save that value into a designated tracking variable.
* **Calculation:** Map the captured current voltage linearly between **14.0V (0%)** and **16.8V (100%)**.
* **Output:** Convert the final voltage calculation into an integer percentage between 0% and 100% to display remaining battery life.

### 2. Memory & Variable Allocation
This system utilizes **static allocation**, allowing the compiler to determine the exact memory footprint required at compile time.

#### Global Motor Overrides
The battery monitoring script interfaces with the motor speeds to allow for low-voltage throttling or speed-matching routines. The following external variables are referenced:

```c
/**
 * Global motor speed limits dictated by the control loop
 * Static Allocation: memory footprint is resolved at compile time.
 */
extern uint8_t motor_topSetSpeed;    // Target speed for the top propulsion motor
extern uint8_t motor_bottomSetSpeed; // Target speed for the bottom propulsion motor
```
