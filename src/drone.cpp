#include "drone.h"

#include "radio.h"
#include "usb.h"
#include "gyro.h"
#include "gimbal.h"
#include "error.h"
#include "motor.h"

static constexpr int CONTROL_LOOP_HZ = 1000;
static constexpr int CONTROL_LOOP_US = (1000000 / CONTROL_LOOP_HZ);

// Higher priority (lower number) than the default (128) so the control tick
// isn't delayed behind lower-priority peripheral interrupts.
static constexpr uint8_t CONTROL_TIMER_PRIORITY = 64;

static constexpr int STATUS_LED = -1; // TODO wire LED on flight computer

using namespace Drone;

// Initialize state to BOOT
static volatile States state = States::BOOT;

// Initialize loop timers
static uint16_t lastLoopTime = 0;
static uint16_t worstTime = 0;
static uint16_t bestTime = -1;
static uint16_t drone_rollAvg = 0;

// Set by the hardware timer ISR every CONTROL_LOOP_US. loop() polls this
// and clears it before running the flight control algorithm, so the
// algorithm itself always executes in normal (non-ISR) context.
volatile bool controlTick = false;

// Counts ticks where the previous one hadn't been serviced by loop() yet,
// i.e. the flight control algorithm is taking longer than CONTROL_LOOP_US.
volatile uint32_t missedTicks = 0;

// Hardware timer driving the control loop tick
IntervalTimer controlTimer;

static Target_t drone_targ0, drone_targ1;

static bool drone_activeSlot = 0;

// MARK: Helpers

States getState() {return state;}

/**
 * ISR fired by the hardware timer at CONTROL_LOOP_HZ.
 *
 * @warning Runs in interrupt context. Do not add I2C/SPI/Serial calls, heap
 * allocation, or anything else non-reentrant here - just flag the tick and
 * let loop() run the actual flight control algorithm.
 */
void onControlTick() {
    if (controlTick) {
        // loop() hasn't serviced the previous tick yet - the control
        // algorithm is running long. Track it so it shows up in telemetry.
        missedTicks++;
    }
    controlTick = true;
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

void Drone::setTarget(Target_t target) {
    drone_targ0.gimbalX = target.gimbalX;
    drone_targ0.gimbalY = target.gimbalY;
    drone_targ0.bottomMotor = target.bottomMotor;
    drone_targ0.topMotor = target.topMotor;
}


// MARK: Startup

/**
 * Performs the startup sequence
 */
bool Drone::startup() {
    // Step 1 Radio
    usb_update(); // Update the USB stack to allow for prints
    updateLEDS(); // Update status LEDS

    switch (state)
    {
    case States::BOOT:
        // This state handles any internal initialization that the controller may need to do
        pinMode(STATUS_LED, OUTPUT);       
        
        // Transition to next state
        state = States::RADIO_SETUP;
        usb_send_text("DRONE: State progressing from BOOT to RADIO_SETUP");
        break;
    
    case States::RADIO_SETUP :
        if(!radio_setup()) {
            state = States::FAULT_ERROR;
            usb_send_text("DRONE: SETUP FAILURE in stage RADIO_SETUP");
        }

        if (radio_setupComplete()) {
            state = States::SENSOR_SETUP;
            usb_send_text("DRONE: State progressing from RADIO_SETUP to SENSOR_SETUP");
        }
        break;

    case States::SENSOR_SETUP :
        // Needs to init Gryo. Any other sensors can also go in here

        // Run setup functions here
        if (!Gyro::setup()) {
            state = States::FAULT_ERROR;
            usb_send_text("DRONE: SETUP FAILURE in stage SENSOR_SETUP -> GYRO");
        }

        // if (!GPS::setup()) {
        //     state = States::FAULT_ERROR;
        //     usb_send_text("DRONE: SETUP FAILURE in stage SENSOR_SETUP -> GPS")
        // }

        // Check for complete here. 
        if (Gyro::setupComplete()) { // Add && GPS::setupComplete()
            usb_send_text("DRONE: State progressing from SENSOR_SETUP to READY_ARMED");
            state = States::CONTROL_SETUP;
        }

        break;

    case States::CONTROL_SETUP :
        Gimbal::setup();
        motor_setup();
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
            usb_send_text("FAULT");
            lastFaultMs = nowMs;
            faultReported = true;
        }
        break;
    }

    default:
        break;
    }

    if (state == States::READY_ARMED){
        usb_send_text("Drone ARMED");
        return true;
    }

    return false;

}


/**
 * Main update loop
 * 
 * Runs at main loop speed and is not controlled by ISR
 */
void Drone::update() {
    static Target_t* slot;

    if (drone_activeSlot == 0) {
        slot = &drone_targ0;
    } else {
        slot = &drone_targ1;
    }

    // Set gimbal. Scale by 1638. Gives +- 20 degrees of range
    Gimbal::set(slot->gimbalX / 1638.0f, slot->gimbalY / 1638.0f);

    motor_setMotor(slot->bottomMotor, slot->topMotor);


    static uint32_t lastTelemetryMs = 0;
    constexpr uint32_t telemetryIntervalMs = 100;
    const uint32_t now = millis();
    if (now - lastTelemetryMs >= telemetryIntervalMs) {
        usb_send_telemetry();
        radio_sendStatus0();
        radio_sendStatus1();
        radio_sendStatus2();
        radio_sendStatus3();
        radio_sendStatus4();
        radio_sendStatus5();
        radio_sendStatus6();
        lastTelemetryMs = now;
    }




}




