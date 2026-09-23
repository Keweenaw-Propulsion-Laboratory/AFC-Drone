#include "drone.h"

#include "radio.h"
#include "usb.h"
#include "gyro.h"
#include "gimbal.h"
#include "motor.h"
#include "battery.h"

static constexpr int CONTROL_LOOP_HZ = 1000;
static constexpr int CONTROL_LOOP_US = (1000000 / CONTROL_LOOP_HZ);

/** Time the watchdog is allowed to be missed in milliseconds */
static constexpr uint16_t COMM_WATCHDOG_SAFETY_MS = 100;

// Higher priority (lower number) than the default (128) so the control tick
// isn't delayed behind lower-priority peripheral interrupts.
//
// Do NOT raise this past SysTick's priority. SysTick is what advances millis(),
// and the tick reads it - recordTelemetry() stamps runtimeSec, and the comm
// watchdog compares an elapsedMillis. Those reads are only safe because SysTick
// still preempts this handler: Teensy 4 runs SysTick at 32 (SCB_SHPR3 in the
// core's startup.c) and STM32L1 at 0 (TICK_INT_PRIORITY), against this value's
// 64 and 64>>4 = 4 respectively. Lower number wins. A control timer numerically
// below SysTick's would freeze millis() for the duration of every tick, which
// stalls the runtime stamp and stops the watchdog ever expiring.
static constexpr uint8_t CONTROL_TIMER_PRIORITY = 64;

static constexpr int STATUS_LED = 9; // TODO wire LED on flight computer

namespace Drone {

// Initialize state to BOOT
static volatile States currentState = States::BOOT;

// Smoothing factor for the rolling loop-time average. 0.1 means the average
// settles over roughly the last 10 ticks.
static constexpr float LOOP_TIME_ALPHA = 0.1f;

// Loop timers, all in microseconds. Written only by the control ISR.
static uint16_t lastLoopTime = 0;
static uint16_t worstTime = 0;
static uint16_t bestTime = 0xFFFF;
static uint16_t rollAvg = 0;
static float rollingAverage = 0.0f;

// Counts ticks where update() ran longer than CONTROL_LOOP_US, i.e. the
// control algorithm overran its own period.
static volatile uint32_t missedTicks = 0;

// The snapshot the control ISR records and loop() reads through
// getTelemetry(). Never transmitted from here - the ISR must not touch the
// radio or USB tx queues.
static Telemetry_t telemetry{};

// Hardware timer driving the control loop tick
static IntervalTimer controlTimer;

// Holds the current setpoint for the motor and gimbal. Written by loop()
// context through setTarget(), read by the control ISR in update().
static volatile Target_t activeTarget;

// MARK: Helpers

States getState() {return currentState;}

/**
 * ISR fired by the hardware timer at CONTROL_LOOP_HZ.
 *
 * Calls update() directly rather than setting a flag for loop() to service.
 * Polling a flag from loop() meant the control algorithm did not start until
 * the current pass through radio/USB/gyro servicing finished, which put the
 * blocking I2C read of the BNO08x - on the order of 100 us - directly into the
 * control loop's jitter budget. Running here instead means the only variance
 * is the timer's own interrupt latency.
 *
 * @warning Everything reachable from here runs in interrupt context. Keep it
 * to arithmetic and register writes: no I2C, SPI, Serial, heap allocation, or
 * pushes to the radio/USB tx queues, which loop() drains and which are not
 * interrupt safe.
 */
static void onControlTick() {
    const uint32_t startTime = micros();

    Drone::update(); // Flight control algorithm + telemetry recording

    const uint32_t cost = micros() - startTime;

    lastLoopTime = static_cast<uint16_t>(cost);
    if (lastLoopTime > worstTime) worstTime = lastLoopTime;
    if (lastLoopTime < bestTime)  bestTime  = lastLoopTime;

    if (rollingAverage == 0.0f) {
        rollingAverage = static_cast<float>(cost); // First tick after boot
    } else {
        rollingAverage = (LOOP_TIME_ALPHA * static_cast<float>(cost)) +
                         ((1.0f - LOOP_TIME_ALPHA) * rollingAverage);
    }
    rollAvg = static_cast<uint16_t>(rollingAverage);

    // update() did not finish inside its own period. The next tick is already
    // pending, so the loop is running late from here on.
    if (cost > static_cast<uint32_t>(CONTROL_LOOP_US)) {
        missedTicks++;
    }
}

/**
 * Starts the hardware timer that drives the flight control loop tick.
 * Called once, after the gimbal/motor outputs it will command are ready.
 */
static void startControlTimer() {
    controlTimer.begin(onControlTick, CONTROL_LOOP_US);
    controlTimer.priority(CONTROL_TIMER_PRIORITY);
}

uint16_t getLastLoopTime() {return lastLoopTime;}
uint16_t getWorstTime() {return worstTime;}
uint16_t getBestTime() {return bestTime;}
uint16_t getRollAvg() {return rollAvg;}
uint32_t getMissedTicks() {return missedTicks;}

void getTelemetry(Telemetry_t& out) {
    // The ISR writes `telemetry` field by field. Without this guard a read
    // from loop() can straddle a tick and return a mix of two instants - a
    // quaternion whose components come from different orientations, say.
    // Copying a few dozen bytes costs well under a microsecond of the control
    // loop's 1000 us period.
    noInterrupts();
    out = telemetry;
    interrupts();
}

// --- LED Logic ---

/**
 * True once analogWrite() has handed STATUS_LED to a timer.
 *
 * analogWrite() configures the pin for a timer's alternate function. Whether
 * the pin then follows the GPIO output register or the timer is decided by
 * that configuration, and digitalWrite() does not touch it - it only writes
 * the output register, which the timer overrides. pinMode() is the call that
 * runs pwm_stop() and puts the pin back under GPIO control.
 *
 * Without this, the first ledFader() pass leaves the pin under the timer
 * forever: every later blink and doubleFlash() writes an output register
 * nothing is listening to, and the LED holds the fader's last duty cycle.
 */
static bool ledUnderPwm = false;

/** digitalWrite() for STATUS_LED that reclaims the pin from the timer first. */
static void ledWrite(bool on) {
    if (ledUnderPwm) {
        pinMode(STATUS_LED, OUTPUT);
        ledUnderPwm = false;
    }
    digitalWrite(STATUS_LED, on ? HIGH : LOW);
}

// The total time in milliseconds for one full breathe cycle (inhale + exhale)
static constexpr uint32_t BREATHE_PERIOD_MS = 2000;

/**
 * One full period of 127.5 * (1 + sin(theta)), rounded to 8-bit duty cycles.
 *
 * Generated with:
 *   round(127.5 * (1 + sin(2 * pi * i / 128))) for i in 0..127
 *
 * A table keeps the fader off the floating point path. The nucleo_l152re bench
 * target is a Cortex-M3 with no FPU, so the old double-precision sin() was a
 * software call costing hundreds of cycles out of a loop() that has to fit
 * between 1 kHz control ticks. 128 steps over the 2 s period is a new duty
 * cycle every ~15.6 ms, changing by at most 7 counts - smooth to the eye.
 */
static const uint8_t BREATHE_TABLE[128] = {
    128, 134, 140, 146, 152, 158, 165, 170,
    176, 182, 188, 193, 198, 203, 208, 213,
    218, 222, 226, 230, 234, 237, 240, 243,
    245, 248, 250, 251, 253, 254, 254, 255,
    255, 255, 254, 254, 253, 251, 250, 248,
    245, 243, 240, 237, 234, 230, 226, 222,
    218, 213, 208, 203, 198, 193, 188, 182,
    176, 170, 165, 158, 152, 146, 140, 134,
    128, 121, 115, 109, 103,  97,  90,  85,
     79,  73,  67,  62,  57,  52,  47,  42,
     37,  33,  29,  25,  21,  18,  15,  12,
     10,   7,   5,   4,   2,   1,   1,   0,
      0,   0,   1,   1,   2,   4,   5,   7,
     10,  12,  15,  18,  21,  25,  29,  33,
     37,  42,  47,  52,  57,  62,  67,  73,
     79,  85,  90,  97, 103, 109, 115, 121,
};

static void ledFader() {
    // Collapse the infinite timeline of millis() into one breathe cycle, then
    // scale that into a table index. The multiply happens first so the whole
    // thing stays in integers; 1999 * 128 is nowhere near overflowing 32 bits.
    uint32_t phase = millis() % BREATHE_PERIOD_MS;
    uint32_t index = (phase * 128) / BREATHE_PERIOD_MS;

    // Output a true hardware PWM duty cycle to your fade pin
    // Note: Make sure your chosen LED pin supports PWM! (On Teensy 4.1, almost all pins do)
    analogWrite(STATUS_LED, BREATHE_TABLE[index]);
    ledUnderPwm = true;
}

static void doubleFlash() {
    const uint32_t cycleDuration = 1200; // Total duration of the pattern in ms
        
    // This collapses the infinite timeline of millis() into a repeating 0-1199ms window
    uint32_t currentCycleTime = millis() % cycleDuration;

    if (currentCycleTime < 100) {
        // 0ms to 99ms -> First Strobe
        ledWrite(true);
    } 
    else if (currentCycleTime >= 100 && currentCycleTime < 250) {
        // 100ms to 249ms -> Dark gap
        ledWrite(false);
    } 
    else if (currentCycleTime >= 250 && currentCycleTime < 350) {
        // 250ms to 349ms -> Second Strobe
        ledWrite(true);
    } 
    else {
        // 350ms to 1199ms -> Long dark pause before cycle resets
        ledWrite(false);
    }
    
}


/**
 * Updates the status LED to match the current state. See drone.h for the
 * per-state patterns.
 */
/**
 * Three quick strobes then a long pause: the comm watchdog tripped.
 *
 * Deliberately distinct from doubleFlash()'s two strobes and from the 50 ms
 * FAULT_ERROR panic blink, because those are the two patterns it is most
 * likely to be read next to.
 */
static void tripleFlash() {
    const uint32_t cycleDuration = 1500; // Total duration of the pattern in ms
    const uint32_t strobeSpacing = 200;  // Start to start of each strobe
    const uint32_t strobeOn = 80;        // Lit portion of each strobe

    // Collapse the infinite timeline of millis() into a repeating window, then
    // strobe inside the first three slots and stay dark for the rest.
    uint32_t currentCycleTime = millis() % cycleDuration;

    bool lit = currentCycleTime < (3 * strobeSpacing) &&
               (currentCycleTime % strobeSpacing) < strobeOn;

    ledWrite(lit);
}

void updateLEDS() {
    static bool ledOn = false;
    static uint32_t lastLEDToggle = 0;

    uint32_t blinkInterval = 500; // Nominal blink interval every 500ms

    // The trip indication outranks the per-state pattern. A watchdog trip
    // leaves the vehicle in SAFE, and SAFE's breathe reads as a healthy idle -
    // it gives the operator no way to tell "I was told to disarm" apart from
    // "the link dropped and I disarmed myself".
    if (getWatchdogTripped()) {
        tripleFlash();
        return;
    }

    switch (currentState) {
        case States::RADIO_SETUP :
            blinkInterval = 100; // Blink every 100 ms during radio setup
            break;
        
        case States::SENSOR_SETUP :
            blinkInterval = 300; // Blink every 300 ms during sensor setup
            break;

        case States::SAFE :
            ledFader();
            return;

        case States::READY_ARMED:
            blinkInterval = 500;
            break;

        case States::MAN_FLIGHT  :
        case States::AUTO_FLIGHT :
            doubleFlash();
            return;

        case States::FAULT_ERROR :
            blinkInterval = 50; // PaNiC
        default :
            break;    
        }

    if (millis() - lastLEDToggle >= blinkInterval) {
        ledOn = !ledOn;
        // Digital write forces the LED to either a solid 0 or 255 duty cycle
        ledWrite(ledOn);
        lastLEDToggle = millis();
    }
}

void setTarget(Target_t target) {
    // The control tick consumes all four fields as one setpoint. Without this
    // guard the tick can land mid-update and fly a mix of two commands - the
    // previous throttle with the new gimbal angle, say. Four stores is well
    // under a microsecond of the control loop's 1000 us period.
    noInterrupts();
    activeTarget.gimbalX = target.gimbalX;
    activeTarget.gimbalY = target.gimbalY;
    activeTarget.bottomMotor = target.bottomMotor;
    activeTarget.topMotor = target.topMotor;
    interrupts();
}


// MARK: Startup

/**
 * Performs the startup sequence
 */
bool startup() {
    // Step 1 Radio
    USB::update(); // Update the USB stack to allow for prints
    updateLEDS(); // Update status LEDS

    switch (currentState)
    {
    case States::BOOT:
        // This state handles any internal initialization that the controller may need to do
        pinMode(STATUS_LED, OUTPUT);       
        
        if(!Battery::setup()) {
            currentState = States::FAULT_ERROR;
            USB::sendText("DRONE: SETUP FAILURE in stage BOOT -> BATTERY");
            
        } else {
            currentState = States::RADIO_SETUP;
            USB::sendText("DRONE: State progressing from BOOT to RADIO_SETUP");
        }
        break;
    
    case States::RADIO_SETUP :
        if(!Radio::setup()) {
            currentState = States::FAULT_ERROR;
            USB::sendText("DRONE: SETUP FAILURE in stage RADIO_SETUP");
            break;
        }

        if (Radio::setupComplete()) {
            currentState = States::SENSOR_SETUP;
            USB::sendText("DRONE: State progressing from RADIO_SETUP to SENSOR_SETUP");
        }
        break;

    case States::SENSOR_SETUP :
        // Needs to init Gryo. Any other sensors can also go in here
        USB::sendText("Gyro");
        // Run setup functions here
        if (!Gyro::setup()) {
            currentState = States::FAULT_ERROR;
            USB::sendText("DRONE: SETUP FAILURE in stage SENSOR_SETUP -> GYRO");
            break;
        }

        // if (!GPS::setup()) {
        //     state = States::FAULT_ERROR;
        //     USB::sendText("DRONE: SETUP FAILURE in stage SENSOR_SETUP -> GPS")
        // }

        // Check for complete here. 
        if (Gyro::setupComplete()) { // Add && GPS::setupComplete()
            USB::sendText("DRONE: State progressing from SENSOR_SETUP to SAFE");
            currentState = States::CONTROL_SETUP;
        }

        break;

    case States::CONTROL_SETUP :
        USB::sendText("Control");
        Gimbal::setup();
        Motor::setup();
        startControlTimer();
        currentState = States::SAFE;
        break;
    
    case States::FAULT_ERROR : {
        // startup() is called from a tight while() loop, so an unthrottled
        // message here would saturate the USB tx buffer and starve every other
        // frame. Once a second is enough to make the fault visible.
        static uint32_t lastFaultMs = 0;
        static bool faultReported = false;
        const uint32_t nowMs = millis();
        if (!faultReported || nowMs - lastFaultMs >= 1000) {
            USB::sendText("FAULT");
            lastFaultMs = nowMs;
            faultReported = true;
        }

        // loop() is unreachable from here - startup() never returns true in
        // this state - so the radio would otherwise go completely dark on a
        // boot fault. Serviced every pass, NOT on the 1 Hz message throttle
        // above: RX_WINDOW_MIN is 10 ms and radio.available() has to be polled
        // far faster than once a second to catch an incoming packet at all.
        if (Radio::setupComplete()) {
            Radio::update();
        }
        break;
    }

    default:
        break;
    }

    if (currentState == States::SAFE){
        USB::sendText("Drone ARMED");
        return true;
    }

    return false;

}


/**
 * Captures one coherent frame of vehicle state for loop() to transmit later.
 *
 * Recording and sending are deliberately split: this runs every control tick
 * so every value is sampled at the same precise instant, while the actual
 * radio and USB writes happen in loop() at ~10 Hz, where blocking on SPI and
 * the tx queues is safe.
 *
 * The timing fields carry the previous tick's measurement, since the ISR
 * measures update() by wrapping the call to it. At 1 kHz that is 1 ms stale.
 *
 * @warning Runs in interrupt context. Reads only; no queue pushes.
 */
static void recordTelemetry() {
    telemetry.loopTimeLast = lastLoopTime;
    telemetry.loopTimeAvg  = rollAvg;
    telemetry.loopTimeMax  = worstTime;
    telemetry.loopTimeMin  = bestTime;
    telemetry.missedTicks  = missedTicks;
    // Safe in interrupt context: SysTick outranks the control timer, so this
    // is a plain load of a counter that is still advancing. See
    // CONTROL_TIMER_PRIORITY.
    telemetry.runtimeSec   = millis() / 1000;
    telemetry.state        = currentState;

    telemetry.gimbalPitch    = Gimbal::getPitch();
    telemetry.gimbalYaw      = Gimbal::getYaw();
    telemetry.topServoSet    = Gimbal::getTopServo();
    telemetry.bottomServoSet = Gimbal::getBottomServo();

    telemetry.topMotorSet    = Motor::getTopSpeed();
    telemetry.bottomMotorSet = Motor::getBottomSpeed();

    // NOTE: Gyro state is written by Gyro::update() in loop() context, so
    // these reads can straddle one of those writes and mix components from
    // two different sensor reports. Publishing the gyro's output through a
    // double buffer would close that hole; see the control-loop notes in
    // docs/code-conventions.md.
    telemetry.qR = Gyro::getQuatReal();
    telemetry.qI = Gyro::getQuatI();
    telemetry.qJ = Gyro::getQuatJ();
    telemetry.qK = Gyro::getQuatK();

    telemetry.accelX = Gyro::getWorldAccelX();
    telemetry.accelY = Gyro::getWorldAccelY();
    telemetry.accelZ = Gyro::getWorldAccelZ();

    telemetry.velX = Gyro::getDroneState().velocity.x;
    telemetry.velY = Gyro::getDroneState().velocity.y;
    telemetry.velZ = Gyro::getDroneState().velocity.z;

    telemetry.posX = Gyro::getDroneState().position.x;
    telemetry.posY = Gyro::getDroneState().position.y;
    telemetry.posZ = Gyro::getDroneState().position.z;
}

/**
 * Flight control algorithm, run once per control tick.
 *
 * @warning Called from onControlTick(), i.e. in INTERRUPT CONTEXT.
 */
void update() {
    static constexpr float GIMBAL_INT16_TO_FLOAT = 1638.0f;

    // Read field by field through the volatile rather than memcpy'ing over it.
    // No guard is needed on this side: loop() cannot preempt an ISR, so the
    // only torn-read window is a write that is already in progress, and
    // setTarget() closes that one by masking around its own stores.
    Target_t activeSlot;
    activeSlot.gimbalX     = activeTarget.gimbalX;
    activeSlot.gimbalY     = activeTarget.gimbalY;
    activeSlot.bottomMotor = activeTarget.bottomMotor;
    activeSlot.topMotor    = activeTarget.topMotor;

    // Set gimbal. Scale by 1638. Gives +- 20 degrees of range
    Gimbal::set(activeSlot.gimbalX / GIMBAL_INT16_TO_FLOAT, 
                activeSlot.gimbalY / GIMBAL_INT16_TO_FLOAT);

    // Set motor set points
    Motor::setMotor(activeSlot.bottomMotor, activeSlot.topMotor);

    recordTelemetry();
}


// MARK: Watchdog

/**
 * Watchdog helper struct. This contains both the elapsed millis counter
 * and the tripped boolean.
 *
 * `tripped` is latched when the watchdog expires in a state that allows
 * movement, and is cleared only when the dashboard releases its request back
 * to SAFE. It is purely an annunciator - the state machine is defended by the
 * edge rule in heartbeat(), not by this flag - but it is what keeps the trip
 * visible on the LED and to the operator after the link comes back.
 */
struct Watchdog {
    elapsedMillis watchdog;
    bool tripped;
};

static Watchdog mechanismWatchdog;

/**
 * How long the vehicle must sit in SAFE before an escalation out of it is
 * honored. Long enough that an auto-reconnecting dashboard cannot walk itself
 * back into flight faster than a human could intervene.
 */
static constexpr uint32_t ESCALATE_DWELL_MS = 500;

// millis() when the vehicle last entered SAFE from another state. Read by
// heartbeat() to enforce ESCALATE_DWELL_MS.
static uint32_t safeEntryMs = 0;

/**
 * The failsafe policy: what the vehicle does when the link goes stale.
 *
 * Deliberately the only place that decides this. Today SAFE is right, because
 * SAFE zeroes the motors and the vehicle is not yet flying free. Once it is,
 * zeroing the motors is a crash rather than a failsafe, and the replacement -
 * hold attitude, descend under control, stay recoverable when the link returns
 * - becomes a change to this function rather than to every actuator guard.
 *
 * Never FAULT_ERROR: requestState() refuses to leave it, so routing a comms
 * dropout there would mean a power cycle to recover from a momentary glitch.
 */
static void onWatchdogExpired() {
    mechanismWatchdog.tripped = true;
    USB::sendText("DRONE: COMM WATCHDOG EXPIRED -> SAFE");
    requestState(States::SAFE);
}

void serviceWatchdog() {
    // Below READY_ARMED nothing is allowed to move and no heartbeat has been
    // asked for yet, so a watchdog that has never been fed is not a fault -
    // without this the vehicle would latch a trip during startup. FAULT_ERROR
    // sorts above the flight states but is terminal, so it is excluded too.
    if (currentState < States::READY_ARMED || currentState == States::FAULT_ERROR) {
        return;
    }

    if (getFlightWatchdogStatus()) {
        return;
    }

    onWatchdogExpired();
}

void heartbeat(States requested) {
    // Liveness is unconditional. The packet arrived, so the link is alive,
    // whether or not we end up honoring what it asks for.
    feedFlightWatchdog();

    // Intent is edge triggered. The dashboard re-asserts its requested mode in
    // every heartbeat, so a repeat is a level, not an operator action - and a
    // heartbeat sent before a dropout is byte identical to one sent after.
    // That is what makes acting only on a change the whole defense here: once
    // a trip has forced SAFE, the dashboard's unchanged MAN_FLIGHT request
    // cannot re-arm the vehicle. It has to release to SAFE and ask again, and
    // that deliberate release-then-request is the edge.
    static States lastRequested = States::BOOT;
    if (requested == lastRequested) {
        return;
    }

    // An escalation out of SAFE also has to wait out the dwell. lastRequested
    // is left alone on this path on purpose: the next heartbeat then still
    // counts as an edge and is honored as soon as the dwell passes, instead of
    // needing another round trip through the dashboard.
    if (requested > States::SAFE && currentState <= States::SAFE &&
        millis() - safeEntryMs < ESCALATE_DWELL_MS) {
        return;
    }

    lastRequested = requested;

    // Releasing to SAFE is the operator acknowledging the trip, so it is what
    // clears the indication.
    if (requested <= States::SAFE) {
        mechanismWatchdog.tripped = false;
    }

    requestState(requested);
}

/**
 * Checks if the watchdog timer is still being fed.
 *
 * Reached from the control tick through the Motor and Gimbal output guards, so
 * this runs in interrupt context. elapsedMillis reads millis(), which keeps
 * advancing there because SysTick outranks the control timer - see
 * CONTROL_TIMER_PRIORITY.
 */
bool getFlightWatchdogStatus() {
    return mechanismWatchdog.watchdog < COMM_WATCHDOG_SAFETY_MS;
}

// Sets the time since last seen to 0. 
void feedFlightWatchdog() {
    mechanismWatchdog.watchdog = 0;
}

bool getWatchdogTripped() {
    return mechanismWatchdog.tripped;
}

bool requestState(States state) {
    // If the requested state is the current state skip
    if (getState() == state) {return true;}

    // Perform checks to allow a safe transition
    if (getState() == States::FAULT_ERROR) {
        // Unable to clear fault
        return false;
    }
    
    switch (state) {
        case States::SAFE :
            Motor::setMotor(0,0);
            // Starts the dwell that heartbeat() makes an escalation wait out.
            // Only reached on a real transition - a request for the state we
            // are already in returns above, so re-requesting SAFE cannot keep
            // pushing the dwell back.
            safeEntryMs = millis();
            currentState = state;
            return true;            

        case States::READY_ARMED :
            currentState = state;
            return true;
        
        case States::MAN_FLIGHT :
            // Zero motor output before transition. Masked for the same
            // reason as setTarget(): the control tick must not read a
            // half-zeroed setpoint.
            noInterrupts();
            activeTarget.bottomMotor = 0;
            activeTarget.topMotor = 0;
            interrupts();

            // Update state
            currentState = state;
            return true;
    
        case States::AUTO_FLIGHT :
            return false; // Not implemented
    default:
        return false;
        break;
    }
}


} // namespace Drone
