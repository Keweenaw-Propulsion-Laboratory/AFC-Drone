/* This file will be used to define all functions relating to the gimbal. 
*/

#pragma once
#include <Servo.h>

// Servo GPIO pins
#define PITCH_SERVO_PIN 24
#define YAW_SERVO_PIN 25

// Extents of the servo lookup tables, which are transcribed straight from
// docs/ServoLookupTable.csv. That sheet is laid out one ROW per yaw setpoint
// and one COLUMN per pitch setpoint, so the maps are indexed [yaw][pitch].
#define YAW_ROWS 9
#define PITCH_COLS 9

namespace Gimbal {

    void setup();

    void set(float pitch, float yaw);

    /**
     * Sets the gimbal throw arms to their zero offset position.
     */
    void zero();

    /**
     * Runs a short test routine on the gimbal servos. 
     * 
     * This routine will move each servo to its maximum allowed deflection in
     * both directions.
     * 
     * @warning This call is blocking and takes approximatley 4 seconds to
     * complete. 
     * 
     * @param lookup When true self test will run the set points through the
     * gimbal lookup tables. When false, the test will set positions
     * explicitly with pos = setpoint + zero point
     * 
     */
    void selfTest(bool lookup);

    /**The current pitch of the gimbal mechanism */
    float getPitch();

    /**The current yaw of the gimbal mechanism */
    float getYaw();

};