/* This file will be used to define all functions relating to the gimbal. 
*/

#pragma once
#include <Servo.h>


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
     * @warning This call is blocking and takes approximately 4 seconds to
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

    uint16_t getTopServo();
    uint16_t getBottomServo();

};