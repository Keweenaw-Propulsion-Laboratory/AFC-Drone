#include "gimbal.h"

#include "Servo.h"
#include "configs.h"
#include "Arduino.h"


#define PITCH_ZERO 90 // Degrees. Some difference in these is normal to account for tooth placement. 
#define YAW_ZERO 89 // Degrees. Some difference in these is normal to account for tooth placement

Servo Gimbal::pitchServo;
Servo Gimbal::yawServo;

float gimbal_botServo = 0.0f;
float gimbal_topServo = 0.0f;

float gimbal_pitch = 0.0f;
float gimbal_yaw = 0.0f;




void Gimbal::setup() {
    pitchServo.attach(PITCH_SERVO_PIN);
    yawServo.attach(YAW_SERVO_PIN);
}

void Gimbal::set(float pitch, float yaw) {

    // Update set points
    gimbal_pitch = pitch;
    gimbal_yaw = yaw;

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
    float q11 = topServoMap[row]     [column];      // Q(1,1) Top left
    float q21 = topServoMap[row + 1] [column];      // Q(2,1) Top Right
    float q12 = topServoMap[row]     [column + 1];  // Q(1,2) Bottom Left
    float q22 = topServoMap[row + 1] [column + 1];  // Q(2,2) Bottom Right

    float topInterp = q11 + y_frac * (q12 - q11);
    float bottomInterp = q21 + y_frac * (q22 - q21);

    // Resulting top servo setpoint
    float topServo = topInterp + x_frac * (bottomInterp - topInterp);

    // Repeat interpolation for bottom servo
    // Get the surrounding calculated values
    q11 = bottomServoMap[row]     [column];      // Q(1,1) Top left
    q21 = bottomServoMap[row + 1] [column];      // Q(2,1) Top Right
    q12 = bottomServoMap[row]     [column + 1];  // Q(1,2) Bottom Left
    q22 = bottomServoMap[row + 1] [column + 1];  // Q(2,2) Bottom Right

    topInterp = q11 + y_frac * (q12 - q11);
    bottomInterp = q21 + y_frac * (q22 - q21);

    // Resulting top servo setpoint
    float bottomServo = topInterp + x_frac * (bottomInterp - topInterp);
    
    // Serial.printf("Top %f \nBot %f ", topServo, bottomServo);

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
    gimbal_topServo = limitRange(angle + config_get().gimbalPitchOffset, 60 , 120);

    pitchServo.write(gimbal_topServo);   
}

/**
 * Sets the yaw servo to the number of degrees off of zero.
 * 
 * @param angle The number of degrees. Positive moves servo throw arm up.
 */
void Gimbal::setBotServo(float angle) {
    gimbal_botServo = limitRange( -angle + config_get().gimbalYawOffset, 60, 120);
    yawServo.write(gimbal_botServo);
}



void Gimbal::zero() {
    // Setting the servos to their mid point
    set(0,0);
    
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