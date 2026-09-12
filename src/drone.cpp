#include "drone.h"

#include "radio.h"
#include "usb.h"
#include "gyro.h"
#include "gimbal.h"
#include "motor.h"

static constexpr int CONTROL_LOOP_HZ = 1000;
static constexpr int CONTROL_LOOP_US = (1000000 / CONTROL_LOOP_HZ);

// Higher priority (lower number) than the default (128) so the control tick
// isn't delayed behind lower-priority peripheral interrupts.
static constexpr uint8_t CONTROL_TIMER_PRIORITY = 64;

static constexpr int STATUS_LED = -1; // TODO wire LED on flight computer

namespace Drone {

// Initialize state to BOOT
static volatile States state = States::BOOT;

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

static Target_t activeTarget;

// MARK: Helpers

States getState() {return state;}

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
void onControlTick() {
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
void startControlTimer() {
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

void ledFader() {
    // The total time in milliseconds for one full breathe cycle (inhale + exhale)
    const float breathePeriodMs = 2000.0f; 
    
    // Convert the current time into a continuous radian angle
    float radians = (2.0f * PI * (float)millis()) / breathePeriodMs;
    
    // Calculate a smooth sine wave scaled between 0.0 and 255.0
    float smoothValue = 127.5f * (1.0f + sin(radians));
    
    // Output a true hardware PWM duty cycle to your fade pin
    // Note: Make sure your chosen LED pin supports PWM! (On Teensy 4.1, almost all pins do)
    analogWrite(STATUS_LED, (int)smoothValue);
    return; // Exit early so standard blinking code below doesn't override this
}

void doubleFlash() {
    const uint32_t cycleDuration = 1200; // Total duration of the pattern in ms
        
    // This collapses the infinite timeline of millis() into a repeating 0-1199ms window
    uint32_t currentCycleTime = millis() % cycleDuration;

    if (currentCycleTime < 100) {
        // 0ms to 99ms -> First Strobe
        digitalWrite(STATUS_LED, HIGH);
    } 
    else if (currentCycleTime >= 100 && currentCycleTime < 250) {
        // 100ms to 249ms -> Dark gap
        digitalWrite(STATUS_LED, LOW);
    } 
    else if (currentCycleTime >= 250 && currentCycleTime < 350) {
        // 250ms to 349ms -> Second Strobe
        digitalWrite(STATUS_LED, HIGH);
    } 
    else {
        // 350ms to 1199ms -> Long dark pause before cycle resets
        digitalWrite(STATUS_LED, LOW);
    }
    
}


/**
 * Helper class to update status LEDS to inform us of current state
 */
void updateLEDS() {
    static bool ledOn = false;
    static uint32_t lastLEDToggle = 0;

    uint32_t blinkInterval = 500; // Nominal blink interval every 500ms

    switch (state) {
        case States::RADIO_SETUP :
            blinkInterval = 100; // Blink every 100 ms during radio setup
            break;
        
        case States::SENSOR_SETUP :
            blinkInterval = 300; // Blink every 300 ms during sensor setup
            break;
        case States::READY_ARMED :
            ledFader();
            return;

        case States::FLIGHT :
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
        digitalWrite(STATUS_LED, ledOn ? HIGH : LOW);
        lastLEDToggle = millis();
    }
}

void setTarget(Target_t target) {
    activeTarget.gimbalX = target.gimbalX;
    activeTarget.gimbalY = target.gimbalY;
    activeTarget.bottomMotor = target.bottomMotor;
    activeTarget.topMotor = target.topMotor;
}


// MARK: Startup

/**
 * Performs the startup sequence
 */
bool startup() {
    // Step 1 Radio
    USB::update(); // Update the USB stack to allow for prints
    updateLEDS(); // Update status LEDS

    switch (state)
    {
    case States::BOOT:
        // This state handles any internal initialization that the controller may need to do
        pinMode(STATUS_LED, OUTPUT);       
        
        // Transition to next state
        state = States::RADIO_SETUP;
        USB::sendText("DRONE: State progressing from BOOT to RADIO_SETUP");
        break;
    
    case States::RADIO_SETUP :
        if(!Radio::setup()) {
            state = States::FAULT_ERROR;
            USB::sendText("DRONE: SETUP FAILURE in stage RADIO_SETUP");
        }

        if (Radio::setupComplete()) {
            state = States::SENSOR_SETUP;
            USB::sendText("DRONE: State progressing from RADIO_SETUP to SENSOR_SETUP");
        }
        break;

    case States::SENSOR_SETUP :
        // Needs to init Gryo. Any other sensors can also go in here

        // Run setup functions here
        if (!Gyro::setup()) {
            state = States::FAULT_ERROR;
            USB::sendText("DRONE: SETUP FAILURE in stage SENSOR_SETUP -> GYRO");
        }

        // if (!GPS::setup()) {
        //     state = States::FAULT_ERROR;
        //     USB::sendText("DRONE: SETUP FAILURE in stage SENSOR_SETUP -> GPS")
        // }

        // Check for complete here. 
        if (Gyro::setupComplete()) { // Add && GPS::setupComplete()
            USB::sendText("DRONE: State progressing from SENSOR_SETUP to READY_ARMED");
            state = States::CONTROL_SETUP;
        }

        break;

    case States::CONTROL_SETUP :
        Gimbal::setup();
        Motor::setup();
        startControlTimer();
        state = States::READY_ARMED;
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
        break;
    }

    default:
        break;
    }

    if (state == States::READY_ARMED){
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
    telemetry.runtimeSec   = millis() / 1000;
    telemetry.state        = state;

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
    static Target_t activeSlot;
    static constexpr float GIMBAL_INT16_TO_FLOAT = 1638.0f;

    memcpy(&activeSlot, &activeTarget, sizeof(Target_t));

    // Set gimbal. Scale by 1638. Gives +- 20 degrees of range
    Gimbal::set(activeSlot.gimbalX / GIMBAL_INT16_TO_FLOAT, 
                activeSlot.gimbalY / GIMBAL_INT16_TO_FLOAT);

    // Set motor set points
    Motor::setMotor(activeSlot.bottomMotor, activeSlot.topMotor);

    recordTelemetry();
}

} // namespace Drone
