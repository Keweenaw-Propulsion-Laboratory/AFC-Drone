#include "motor.h"
#include "Arduino.h"

constexpr int bottomMotor = 4;
constexpr int topMotor = 5;

float bottomSetSpeed = 0.0f;
float topSetSpeed = 0.0f;

void motor_setup() {
    pinMode(bottomMotor, OUTPUT);
    pinMode(topMotor, OUTPUT);

    analogWriteFrequency(bottomMotor, 20000);
    analogWriteFrequency(topMotor, 20000);

    analogWriteResolution(8);
}

void motor_setMotor(float bottomMotorSpeed, float topMotorSpeed ) {
    if (bottomMotorSpeed == bottomSetSpeed && topMotorSpeed == topSetSpeed) return; // Skip if speed is already set.
    analogWrite(bottomMotor, bottomMotorSpeed * 256);
    analogWrite(topMotor, topMotorSpeed * 256);
    bottomSetSpeed = bottomMotorSpeed;
    topSetSpeed = topMotorSpeed;
}