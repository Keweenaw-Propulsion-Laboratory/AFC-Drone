#include "motor.h"
#include "Arduino.h"
#include "Servo.h"
#include "configs.h"

static constexpr int BOTTOM_MOTOR_PIN = 29;
static constexpr int TOP_MOTOR_PIN = 28;

static constexpr int ESC_MIN_US = 1000;
static constexpr int ESC_MIN_RUNNING = 1333;
static constexpr int ESC_MAX_US = 2000;

static Servo topMotor;
static Servo bottomMotor;

uint16_t motor_bottomSetSpeed = 0;
uint16_t motor_topSetSpeed = 0;

void motor_setup() {
    bottomMotor.attach(BOTTOM_MOTOR_PIN);
    topMotor.attach(TOP_MOTOR_PIN);

    // Zero out the controls
    bottomMotor.writeMicroseconds(ESC_MIN_US); 
    topMotor.writeMicroseconds(ESC_MIN_US);
}

void motor_setMotor(uint8_t bottomMotorSpeed, uint8_t topMotorSpeed ) {

    // Clamp in signed space. The trim offset is signed, so the sum must stay
    // signed until it is known to be in range - narrowing first would wrap a
    // small negative result up to a near-maximum throttle.
    int top = (int)topMotorSpeed + config_get().motor1offset;
    int bottom = (int)bottomMotorSpeed + config_get().motor2offset;

    if (top < 0) top = 0; else if (top > 255) top = 255;
    if (bottom < 0) bottom = 0; else if (bottom > 255) bottom = 255;

    motor_topSetSpeed = (uint16_t)top;
    motor_bottomSetSpeed = (uint16_t)bottom;


    uint16_t bottomSpeed =
        ESC_MIN_RUNNING +
        ((uint32_t)motor_bottomSetSpeed * (ESC_MAX_US - ESC_MIN_RUNNING)) / 255;

    uint16_t topSpeed =
        ESC_MIN_RUNNING +
        ((uint32_t) motor_topSetSpeed * (ESC_MAX_US - ESC_MIN_RUNNING)) / 255;

 
    // If motors are suppose to be off set to min armed value
    if (bottomMotorSpeed == 0) {
        bottomSpeed = ESC_MIN_US;
    }

    if (topMotorSpeed == 0) {
        topSpeed = ESC_MIN_US;
    }


    bottomMotor.writeMicroseconds(bottomSpeed);
    topMotor.writeMicroseconds(topSpeed);
}