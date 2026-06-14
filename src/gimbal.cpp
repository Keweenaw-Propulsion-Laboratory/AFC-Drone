#include "gimbal.h"

#include "Servo.h"
#include "Arduino.h"

#define PITCH_ZERO 90 // Degrees. Some difference in these is normal to account for tooth placement. 
#define YAW_ZERO 89 // Degrees. Some difference in these is normal to account for tooth placement

Servo Gimbal::pitchServo;
Servo Gimbal::yawServo;


void Gimbal::setup() {
    pitchServo.attach(PITCH_SERVO_PIN);
    yawServo.attach(YAW_SERVO_PIN);
}

void Gimbal::set(float pitch, float yaw) {

    // Bilinear Interpolation
    // https://en.wikipedia.org/wiki/Bilinear_interpolation

    // Clamp the pitch inputs
    if (pitch < pitchValues[0]) pitch = pitchValues[0];
    if (pitch > pitchValues[PITCH_ROWS-1]) pitch = pitchValues[PITCH_ROWS-1];

    // Clamp the yaw inputs
    if (yaw < yawValues[0]) yaw = yawValues[0];
    if (yaw > yawValues[YAW_COLS-1]) yaw = yawValues[YAW_COLS-1];

    // Map the pitch and yaw to the nearest index

    // The row that the setpoint is in.
    uint8_t row = (uint8_t) (yaw + 20) / 5;
    // The column that the setpoint is in
    uint8_t column = (uint8_t) (pitch + 20) / 5;


    // Calculate how close the original command was to a precalculated command
    float y_frac = (pitch - pitchValues[column]) / (pitchValues[column + 1] - pitchValues[column]);
    float x_frac = (yaw - yawValues[row]) / (yawValues[row + 1] - yawValues[row]);

    // Get the surrounding calculated values
    float q11 = topServoMap[column]     [row];      // Q(1,1) Top left
    float q21 = topServoMap[column + 1] [row];      // Q(2,1) Top Right
    float q12 = topServoMap[column]     [row + 1];  // Q(1,2) Bottom Left
    float q22 = topServoMap[column + 1] [row + 1];  // Q(2,2) Bottom Right

    float topInterp = q11 + x_frac * (q21 - q11);
    float bottomInterp = q12 +x_frac * (q22 - q11);

    // Resulting top servo setpoint
    float topServo = topInterp + y_frac * (bottomInterp - topInterp);

    // Repeat interpolation for bottom servo
    // Get the surrounding calculated values
    q11 = bottomServoMap[column]     [row];      // Q(1,1) Top left
    q21 = bottomServoMap[column + 1] [row];      // Q(2,1) Top Right
    q12 = bottomServoMap[column]     [row + 1];  // Q(1,2) Bottom Left
    q22 = bottomServoMap[column + 1] [row + 1];  // Q(2,2) Bottom Right

    topInterp = q11 + x_frac * (q21 - q11);
    bottomInterp = q12 +x_frac * (q22 - q12);

    // Resulting top servo setpoint
    float bottomServo = topInterp + y_frac * (bottomInterp - topInterp);
    
    Serial.printf("Top %f \nBot %f ", topServo, bottomServo);

    // Set servos
    setTopServo(topServo);
    setBotServo(bottomServo);

}



/**
 * Sets the pitch servo to the number of degrees off of zero.
 * 
 * @param angle The number of degrees. Positive moves servo throw arm up.
 */
void Gimbal::setTopServo(float angle) {
    pitchServo.write(limitRange(angle + PITCH_ZERO, 60 , 120));   
}

/**
 * Sets the yaw servo to the number of degrees off of zero.
 * 
 * @param yaw The number of degrees. Positive moves servo throw arm up.
 */
void Gimbal::setBotServo(float angle) {
    yawServo.write(limitRange( -angle + YAW_ZERO, 60, 120));
}



void Gimbal::zero() {
    // Setting the servos to their mid point
    pitchServo.write(90);
    yawServo.write(90);
}

void Gimbal::selfTest() {
    // Test Servos independently 
    // setTopServo(-30);
    // delay(1000);

    // setBotServo(-30);
    // delay(1000);

    // setTopServo(30);
    // delay(1000);

    // setBotServo(30);
    // delay(1000);

    // setBotServo(0);
    // setTopServo(0);
    // delay(2000);
    // // Test reference 

    set(-30, 0);
    delay(1000);
    set(0, -30);
    delay(1000);
    set(30, 0);
    delay(1000);
    set(0, 30);
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