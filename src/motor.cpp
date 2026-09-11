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

    topSetSpeed = (uint16_t)top;
    bottomSetSpeed = (uint16_t)bottom;


    uint16_t bottomSpeed =
        ESC_MIN_RUNNING +
        ((uint32_t)bottomSetSpeed * (ESC_MAX_US - ESC_MIN_RUNNING)) / 255;

    uint16_t topSpeed =
        ESC_MIN_RUNNING +
        ((uint32_t) topSetSpeed * (ESC_MAX_US - ESC_MIN_RUNNING)) / 255;

 
    // If motors are suppose to be off set to min armed value
    if (bottomMotorSpeed == 0) {
        bottomSpeed = ESC_MIN_US;
    }

    if (topMotorSpeed == 0) {
        topSpeed = ESC_MIN_US;
    }

    if (Drone::getFlightWatchdogStatus()) {
        bottomMotor.writeMicroseconds(bottomSpeed);
        topMotor.writeMicroseconds(topSpeed);
    } else {
        bottomMotor.writeMicroseconds(ESC_MIN_US);
        topMotor.writeMicroseconds(ESC_MIN_US);
    }
}

} // namespace Motor
