#include "motor.h"
#include "Arduino.h"

static constexpr int bottomMotor = 4;
static constexpr int topMotor = 5;

uint8_t motor_bottomSetSpeed = 0;
uint8_t motor_topSetSpeed = 0;

void motor_setup() {
    pinMode(bottomMotor, OUTPUT);
    pinMode(topMotor, OUTPUT);

    analogWriteFrequency(bottomMotor, 20000);
    analogWriteFrequency(topMotor, 20000);

    analogWriteResolution(8);
}

void motor_setMotor(uint8_t bottomMotorSpeed, uint8_t topMotorSpeed ) {
    if (bottomMotorSpeed == motor_bottomSetSpeed && topMotorSpeed == motor_topSetSpeed) return; // Skip if speed is already set.
    analogWrite(bottomMotor, bottomMotorSpeed);
    analogWrite(topMotor, topMotorSpeed);
    motor_bottomSetSpeed = bottomMotorSpeed;
    motor_topSetSpeed = topMotorSpeed;
}