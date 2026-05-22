/* This file will be used to define all functions relating to the gimbal. 
*/

#pragma once
#include <Servo.h>

#define PITCH_SERVO_PIN 24
#define YAW_SERVO_PIN 25

class Gimbal {
    public: 
        static void setup();

        static void setPitch(int pitch);
        static void setYaw(int yaw);

        static void zero();
        static void selfTest();


    private:
        static int pitch;
        static int yaw;

        static Servo pitchServo;
        static Servo yawServo;

        static int limitRange(int val, int low, int high);

};