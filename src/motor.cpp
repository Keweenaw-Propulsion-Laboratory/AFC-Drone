#include "motor.h"
#include "Arduino.h"
#include "Servo.h"

static constexpr int bottomMotorPin = 29;
static constexpr int topMotorPin = 28;

static constexpr int ESC_MIN_US = 1000;
static constexpr int ESC_MAX_US = 2000;

static Servo topMotor;
static Servo bottomMotor;

uint8_t motor_bottomSetSpeed = 0;
uint8_t motor_topSetSpeed = 0;

void motor_setup() {
    bottomMotor.attach(bottomMotorPin);
    topMotor.attach(topMotorPin);

    // Zero out the controls
    bottomMotor.writeMicroseconds(ESC_MIN_US); 
    topMotor.writeMicroseconds(ESC_MIN_US);
}

void motor_setMotor(uint8_t bottomMotorSpeed, uint8_t topMotorSpeed ) {
    uint16_t bottomSpeed =
        ESC_MIN_US +
        ((uint32_t)bottomMotorSpeed * (ESC_MAX_US - ESC_MIN_US)) / 255;

    uint16_t topSpeed =
        ESC_MIN_US +
        ((uint32_t) topMotorSpeed * (ESC_MAX_US - ESC_MIN_US)) / 255;

    motor_topSetSpeed = topMotorSpeed;
    motor_bottomSetSpeed = bottomMotorSpeed;

    bottomMotor.writeMicroseconds(bottomSpeed);
    topMotor.writeMicroseconds(topSpeed);
}