#include "motor.h"
#include "Arduino.h"
#include "Servo.h"
#include "configs.h"
#include "drone.h"

namespace Motor {

static constexpr int BOTTOM_MOTOR_PIN = 29;
static constexpr int TOP_MOTOR_PIN = 28;

static constexpr int ESC_MIN_US = 1000;
static constexpr int ESC_MIN_RUNNING = 1333;
static constexpr int ESC_MAX_US = 2000;

static Servo topMotor;
static Servo bottomMotor;

static uint16_t bottomSetSpeed = 0;
static uint16_t topSetSpeed = 0;

//MARK: Helpers
uint16_t getBottomSpeed() {return bottomSetSpeed;}
uint16_t getTopSpeed() {return topSetSpeed;}



//MARK: Motor Logic

void setup() {
    bottomMotor.attach(BOTTOM_MOTOR_PIN);
    topMotor.attach(TOP_MOTOR_PIN);

    // Zero out the controls
    bottomMotor.writeMicroseconds(ESC_MIN_US); 
    topMotor.writeMicroseconds(ESC_MIN_US);
}

void setMotor(uint8_t bottomMotorSpeed, uint8_t topMotorSpeed ) {

    // Clamp in signed space. The trim offset is signed, so the sum must stay
    // signed until it is known to be in range - narrowing first would wrap a
    // small negative result up to a near-maximum throttle.
    int top = (int)topMotorSpeed + Configs::get().motor1offset;
    int bottom = (int)bottomMotorSpeed + Configs::get().motor2offset;

    if (top < 0) top = 0; else if (top > 255) top = 255;
    if (bottom < 0) bottom = 0; else if (bottom > 255) bottom = 255;

    // Scale from this call's trimmed values, not from the telemetry vars -
    // those are now assigned below, once it is known what actually reached the
    // ESCs, and reading them here would command the previous tick's throttle.
    uint16_t bottomSpeed =
        ESC_MIN_RUNNING +
        ((uint32_t)bottom * (ESC_MAX_US - ESC_MIN_RUNNING)) / 255;

    uint16_t topSpeed =
        ESC_MIN_RUNNING +
        ((uint32_t)top * (ESC_MAX_US - ESC_MIN_RUNNING)) / 255;

 
    // If motors are suppose to be off set to min armed value
    if (bottomMotorSpeed == 0) {
        bottomSpeed = ESC_MIN_US;
    }

    if (topMotorSpeed == 0) {
        topSpeed = ESC_MIN_US;
    }

    // Only allow motor movement when watchdog is fed and state = flight.
    // The telemetry vars are assigned inside each branch, so they report what
    // the ESCs were actually commanded rather than what was asked for - a
    // suppressed throttle must not show up on the dashboard as a live one.
    if (Drone::getFlightWatchdogStatus() && 
        Drone::getState() == Drone::States::MAN_FLIGHT) {
        bottomMotor.writeMicroseconds(bottomSpeed);
        topMotor.writeMicroseconds(topSpeed);

        topSetSpeed = (uint16_t)top;
        bottomSetSpeed = (uint16_t)bottom;

    } else {
        bottomMotor.writeMicroseconds(ESC_MIN_US);
        topMotor.writeMicroseconds(ESC_MIN_US);
    
        topSetSpeed = 0;
        bottomSetSpeed = 0;
    }
}  

} // namespace Motor
