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

    motor_topSetSpeed = (int)topMotorSpeed + config_get().motor1offset;
    motor_bottomSetSpeed = (int)bottomMotorSpeed + config_get().motor2offset;
    
    // Clamp the motor set speed to be between 0 and 255
    motor_topSetSpeed = motor_topSetSpeed <= 0 ? 0 : motor_topSetSpeed > 255 ? 255 : motor_topSetSpeed;
    motor_bottomSetSpeed = motor_bottomSetSpeed <= 0 ? 0 : motor_bottomSetSpeed > 255 ? 255 : motor_bottomSetSpeed;


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