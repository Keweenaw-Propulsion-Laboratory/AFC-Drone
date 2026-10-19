# AFC Test Vehicle - System Requirements

**Aeronautics and Rocketry Enterprise - Keweenaw Propulsion Lab**

**AFC-Drone**

**Date:** Oct 17, 2026

---
## TODO
- [ ] **TBD values:** set the control tolerances, HW-08 nominal thrust, STR-06 payload mass.
- [ ] **Verify Trace table:** Ensure that the trace table is accurate and not missing any requirements. 


## 1. Scope

This document sets the top-level requirements for the Autonomous Flight Control (AFC) Drone (AFC-Drone) test vehicle. This is a stand-in prototype for a liquid-fueled lander for the Collegiate Propulsive Lander Challenge (CPLC). This prototype must mimic a rocket in as many ways as possible while providing a cost-effective and safe test platform. 

The following requirements outline the absolute minimums that the vehicle must complete in order to fulfill its role as a test platform. Design details such as part selection and implementation are not included here. Those details belong in design documentation.

## 2. Requirement conventions

Requirements follow [NASA SE Handbook Appendix C](https://www.nasa.gov/reference/appendix-c-how-to-write-a-good-requirement/), but have been adjusted for this project. 

- **Shall** marks a binding requirement. **Will** states a fact or intent. **Should** marks a goal.
- Each requirement states one verifiable need, with a number and units or a TBD/TBR.
- No requirement names a part, mechanism, or algorithm. If more than one design could meet it, it is written correctly.
- **TBD** means the value is not yet known. **TBR** means the value is set but still needs confirmation.

### Requirement outline

| Column | Meaning |
| --- | --- |
| ID | Category prefix and number: AFC (mission), CPLC, STR, HW, SAF |
| Name | Short title |
| Requirement | The "shall" statement |
| Rationale / Notes | Why it exists and where it comes from |
| Method | Test, Demonstration, Inspection, or Analysis |
| Artifact | The evidence that closes it, such as a flight log, video, CAD file, or report |
| Status | Incomplete, In progress, or Verified |

## 3. Mission requirements (AFC)

**AFC-01** is the top-level requirement. Every other requirement traces to it. **AFC-02** to **AFC-08** outline the broadest requirements for the AFC-Drone to be a valid stand-in for a rocket lander.

| ID | Name | Requirement | Rationale / Notes | Method | Artifact | Status |
| --- | --- | --- | --- | --- | --- | --- |
| AFC-01 | Mission statement | The AFC Test Vehicle shall meet scaled versions of all five CPLC milestones. | Proves the flight control approach before it moves to a liquid-fueled lander. | Analysis | All requirements in the Section 4 trace table closed | Incomplete |
| AFC-02 | Single thrust source | The vehicle shall get all of its lift from one primary propulsion device. | Matches a single-engine lander. CPLC requires one PPD. | Inspection | As-built vehicle | Incomplete |
| AFC-03 | Lander dynamics | The vehicle shall be unstable in pitch and roll without active control, like an inverted pendulum. | Control learned on a stable vehicle would not carry over to a rocket. | Analysis | Stability analysis | Incomplete |
| AFC-04 | All-electric | The vehicle shall use only stored electrical energy for propulsion and control. | Allows for the development of controls without the cost and hazard of chemical propulsion. | Inspection | As-built vehicle | Incomplete |
| AFC-05 | Endurance | The vehicle shall sustain controlled powered flight for at least 120 s on one charge. | Covers the longest CPLC challenge with margin. | Test | Flight log | Incomplete |
| AFC-06 | Safe to test | The vehicle shall be testable in the AERE lab space and at an outdoor site with no credible risk of injury or property damage. | Testing is limited to spaces KPL can use safely. | Inspection, Demonstration | Test site survey and flight video | Incomplete |
| AFC-07 | Autonomous control | The vehicle shall be capable of maintaining steady flight without human input. | Landing a rocket by hand is not feasible. Control algorithms can be designed and tested here. | Demonstration | Stability Analysis | Incomplete |
| AFC-08 | Data logging | The vehicle shall record its system states, sensor inputs, and actuator outputs. | The data collected can be used for further analysis and improvement of the control system. | Inspection | Data log file | Incomplete |

## 4. CPLC challenges

The [CPLC Challenge Rules](https://www.definityeducation.org/docs/challenge-rules.pdf) (Version: Feb 23, 2024) set five milestones, listed below in the rulebook's order. They are success criteria for test events, not vehicle requirements. The vehicle requirements needed to pass each one are spread through Sections 3 to 8 and listed in the trace table.

The rules' 500 lbf thrust minimum applies to full-scale engines, so thrust levels are scaled to this vehicle. Heights and distances stay at full CPLC values. Every attempt needs two angles of video and enough data for the judges to confirm success.

**Challenge 1: TVC Hotfire.** Fire the primary propulsion device for at least 10 s. During the firing, move the thrust direction at least 7° in all directions, like a circle, in a controlled way. The primary propulsion device itself must change the thrust direction. Secondary thrusters are not allowed. For AFC-Drone it is a thrust test with the vehicle held in place.

**Challenge 2: Throttleable Hotfire.** Fire for at least 10 s of controlled thrust. Hold a constant nominal thrust for 4 s, throttle smoothly down to below 40% of nominal and hold for 2 s, then return to nominal and hold for 1 s. Measured thrust must stay within ±5% of nominal around a thrust plan submitted before the test, with a 0.5 s timing allowance at each throttle change. This is also a held-in-place thrust test.

**Challenge 3: Bess Touchdown.** Starting with the whole vehicle at least 10 m above ground and at zero velocity, descend under the primary propulsion device. Reach zero vertical velocity within 1 m of the ground, then make a Successful Landing. A 10 m hop also counts. A tether is recommended.

**Challenge 4: Tethered Hover.** Hover in a controlled way for 10 s on a tether, with the vehicle's centroid staying inside a 1 m cube.

**Challenge 5: Hop.** Ascend from the ground until the whole vehicle is 50 m above ground, then make a Successful Landing inside a 10 m diameter circle. The vehicle must stay within the circle's vertical walls while landing. The vehicle must have a kill switch, and GNSS data is required.

Challenges 3 to 5 also require a **Payload**: a single 15 kg mass that takes no part in vehicle operation. A **Successful Landing** is a gentle vertical landing after which the vehicle can be reused with minor repair.

### Trace table

| Challenge | Covered by |
| --- | --- |
| 1. TVC Hotfire | HW-02, AFC-05, CPLC-01 |
| 2. Throttleable Hotfire | HW-08, HW-09, AFC-05, CPLC-01 |
| 3. Bess Touchdown | AFC-07, STR-05, STR-06, CPLC-01 to CPLC-03 |
| 4. Tethered Hover | AFC-07, STR-05, STR-06, CPLC-01, CPLC-02 |
| 5. Hop | AFC-07, HW-01, HW-03, HW-04, STR-06, CPLC-01, CPLC-03, CPLC-04, SAF-01 |

### Requirements that apply to every challenge

| ID | Name | Requirement | Rationale / Notes | Method | Artifact | Status |
| --- | --- | --- | --- | --- | --- | --- |
| CPLC-01 | Attempt data | The vehicle shall record its control system data during every challenge attempt, and its avionics, acceleration, and GNSS data during every flight attempt. | Judges need data to award each milestone. | Demonstration | Downloaded logs | Incomplete |
| CPLC-02 | Tether provision | The vehicle shall fly both with and without a tether attached. | Challenges 3 and 4 use a tether. Challenge 5 does not. | Demonstration | Flight video | Incomplete |
| CPLC-03 | Successful landing | After every landing, the vehicle shall be reusable with no more than minor repair. | The rules' definition of a Successful Landing. Crush-type legs are not allowed. | Inspection | Post-flight inspection | Incomplete |
| CPLC-04 | Kill switch | The vehicle shall have a kill switch | The kill switch allows an operator to put the vehicle into a safe state if a problem occurs during a test and is required by CPLC | Inspection | N/A | Incomplete |

## 5. Structural requirements (STR)

These requirements set the physical constraints of the vehicle. This vehicle will need to be flown repeatedly and be safe enough to perform tests indoors.

| ID | Name | Requirement | Rationale / Notes | Method | Artifact | Status |
| --- | --- | --- | --- | --- | --- | --- |
| STR-01 | Size | The vehicle shall fit within a 1 m × 1 m × 1 m envelope. | Must fly inside the AERE lab test area (AFC-06). | Inspection | Measured as-built size | Incomplete |
| STR-02 | Mass | The vehicle takeoff weight shall not exceed 22.2 N (5 lb). | Limits crash energy for safe testing (AFC-06). | Inspection | Scale measurement | Incomplete |
| STR-03 | Landing loads | The vehicle shall survive a touchdown at the speed limit of TBD m/s with no damage that needs repair before the next flight. | Allows repeated test flights. | Test | Post-flight inspection | Incomplete |
| STR-04 | Stays upright | The vehicle shall remain upright after touchdown on ground sloped up to TBD°. | Tip-over at touchdown is a common lander failure. CPLC needs a successful landing. | Analysis, Test | Tip-over analysis | Incomplete |
| STR-05 | Tether | The vehicle shall accept a tether that holds it under all flight loads. | Needed for Challenges 3 and 4 (CPLC-02). | Analysis, Inspection | Load analysis | Incomplete |
| STR-06 | Payload | The vehicle shall carry a single payload mass of TBD kg that takes no part in vehicle operation. | Challenges 3 to 5 require a payload. The rules' 15 kg payload is scaled to this vehicle. | Inspection | As-built vehicle with payload | Incomplete |

## 6. Hardware requirements (HW)

These state what the propulsion, power, sensing, and communication functions must provide. They leave the choice of hardware open.

| ID | Name | Requirement | Rationale / Notes | Method | Artifact | Status |
| --- | --- | --- | --- | --- | --- | --- |
| HW-01 | Thrust margin | The vehicle thrust-to-weight ratio shall be greater than 1.3 for the whole flight, including at end-of-flight battery charge. | Leaves margin to climb 50 m and reject disturbances. | Test | Thrust test data | Incomplete |
| HW-02 | Thrust vectoring | The vehicle shall be able to move the direction of its primary thrust at least 7° from the long axis in every direction. | Challenge 1. The primary thrust source itself must change the thrust direction. | Test | Restrained thrust test data and video | Incomplete |
| HW-03 | State sensing | The vehicle shall measure its attitude, angular rate, acceleration, altitude, and position. | Control and CPLC data both need these states (CPLC-01). | Analysis, Test | Data log | Incomplete |
| HW-04 | Communication | The vehicle shall keep a two-way wireless link with the ground station across the whole test area, up to TBD m. | Carries commands, kill signals, and live data for the 50 m hop. Must be wireless because the vehicle flies untethered in Challenge 5. | Test | Range test log | Incomplete |
| HW-05 | Data storage | The vehicle shall store all flight data on board in a way that survives power loss. | Data must survive a crash or kill (CPLC-01). | Demonstration | Log recovered after power cut | Incomplete |
| HW-06 | Energy monitoring | The vehicle shall measure its remaining battery energy and report it to the operator during flight. | Needed for AFC-05 and SAF-04. | Demonstration | Ground station data | Incomplete |
| HW-07 | Power isolation | The vehicle shall have a way to remove all onboard power by hand. | Safe ground handling. | Demonstration | As-built vehicle | Incomplete |
| HW-08 | Throttle range | The vehicle shall be able to throttle smoothly from a nominal thrust of TBD N down to below 40% of nominal and back. | Challenge 2. | Test | Restrained thrust test data | Incomplete |
| HW-09 | Thrust accuracy | The vehicle shall hold measured thrust within ±5% of nominal of the commanded thrust, including during throttle changes. | Challenge 2 thrust-trace tolerance. | Test | Planned and measured thrust traces | Incomplete |

## 7. Safety requirements (SAF)

To provide a safe testing environment and prevent injury to personnel, the vehicle needs safety features. CPLC also requires a kill switch.

| ID | Name | Requirement | Rationale / Notes | Method | Artifact | Status |
| --- | --- | --- | --- | --- | --- | --- |
| SAF-01 | Remote kill | The operator shall be able to stop all thrust within 1 s at any time during flight. | Primary hazard control. Required by CPLC-04. | Demonstration | Test video and log | Incomplete |
| SAF-02 | Commanded landing | The operator shall be able to command the vehicle to land itself at any time during flight. | Ends a flight safely without dropping the vehicle. | Demonstration | Test video and log | Incomplete |
| SAF-03 | Link loss | The vehicle shall land itself if the command link is lost for more than 10 s. | Prevents a flyaway. | Demonstration | Test log | Incomplete |
| SAF-04 | Low energy | The vehicle shall land itself before battery energy falls below what a controlled landing needs. | Prevents an uncontrolled fall. | Demonstration | Test log | Incomplete |
| SAF-05 | Fault response | The vehicle shall land itself or stop all thrust when it loses information it needs for controlled flight. | A vehicle that cannot control itself is a hazard. | Demonstration | Fault test log | Incomplete |
| SAF-06 | Arming | The vehicle shall not produce thrust or move any actuator until the operator deliberately arms it. | Prevents accidental start during handling. | Demonstration | Test video | Incomplete |
| SAF-07 | Armed status | People near the vehicle shall be able to tell whether it is armed. | Tells the crew when it is unsafe to approach. | Inspection | As-built vehicle | Incomplete |
