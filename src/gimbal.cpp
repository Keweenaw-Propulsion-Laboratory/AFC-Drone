#include "gimbal.h"

#include "Servo.h"
#include "Arduino.h"

#define PITCH_ZERO 90 // Degrees. Some difference in these is normal to account for thooth placement. 
#define YAW_ZERO 89 // Degrees. Some difference in these is normal to account for tooth placement

Servo Gimbal::pitchServo;
Servo Gimbal::yawServo;


void Gimbal::setup() {
    pitchServo.attach(PITCH_SERVO_PIN);
    yawServo.attach(YAW_SERVO_PIN);
}

/**
 * Sets the pitch servo to the number of degrees off of zero.
 * 
 * @param pitch The number of degrees. Positive moves servo throw arm up.
 */
void Gimbal::setPitch(int pitch) {
    pitchServo.write(limitRange(pitch + PITCH_ZERO, 60 , 120));   
}

/**
 * Sets the yaw servo to the number of degrees off of zero.
 * 
 * @param yaw The number of degrees. Positive moves servo throw arm up.
 */
void Gimbal::setYaw(int yaw) {
    yawServo.write(limitRange( -yaw + YAW_ZERO, 60, 120));
}

void Gimbal::zero() {
    // Setting the servos to their mid point
    pitchServo.write(90);
    yawServo.write(90);
}

void Gimbal::selfTest() {
    setPitch(-30);
    delay(1000);

    setYaw(-30);
    delay(1000);

    setPitch(30);
    delay(1000);

    setYaw(30);
    delay(1000);
}

int Gimbal::limitRange(int val, int low, int high){
    if(val > high) {
        val = high;
    } else if (val < low) {
        val = low;
    }

    return val;
}