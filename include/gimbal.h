/* This file will be used to define all functions relating to the gimbal. 
*/

#pragma once
#include <Servo.h>

// Servo GPIO pins
#define PITCH_SERVO_PIN 24
#define YAW_SERVO_PIN 25

// Number of measure pitch and yaw points in the loop up tables
#define PITCH_ROWS 9
#define YAW_COLS 9



class Gimbal {
    public: 
        static void setup();

        static void set(float pitch, float yaw);

        static void zero();
        static void selfTest();

        static float getPitch() {return pitch;}
        static float getYaw() {return yaw;}

        static float getTopServo() {return topServo;}
        static float getBottomServo() {return botServo;}


    private:
        static float pitch;
        static float yaw;

        static float topServo;
        static float botServo;

        static Servo pitchServo;
        static Servo yawServo;

        static int limitRange(int val, int low, int high);

        /**
         * INTERNAL USE ONLY. Manually sets the servo to a point.
         * 
         * @warning does not update pitch and yaw set points
         */
        static void setTopServo(float angle);
        
        /**
         * INTERNAL USE ONLY. Manually sets the servo to a point.
         * 
         * @warning does not update pitch and yaw set points
         */
        static void setBotServo(float angle);


        // MARK: Lookup table for gimbal correction

        static constexpr float pitchValues[PITCH_ROWS] = {-20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0};
        static constexpr float yawValues[YAW_COLS] = {-20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0};

        static constexpr float topServoMap[PITCH_ROWS][YAW_COLS] = {
            {-1.306221,     5.164407,   11.298533,  17.210218,  23.018482,  16.95216,   11.170965,  5.554293,   0},
            {-6.744026,     -0.423434,  5.579505,   11.3642,    16.95216,   11.3642,    5.711151,   -0.155048,  -5.272502},
            {-12.11768,     -5.909759,  0,          5.711151,   11.170965,  5.512334,   0,          -5.711151,  -10.928924},
            {-17.492678,    -11.368308, -5.512334,  0.155048,   5.541829,   -0.155048,  -5.711151,  -11.3642,   -16.753008},
            {-23.018482,    -16.753008, -10.928924, -5.272502,  0,          5.272502,   10.928924,   16.753008,   23.018482},
            {-16.753008,    -11.3642,   -5.711151,  -0.155048,  -5.554293,  -0.155048,  5.512334,   11.368308,   17.492678},
            {-11.170965,    -5.711151,  0,          5.512334,  -11.170965,  -5.711151,   0,           5.909759,    12.11768},
            {-5.554293,     0.155048,   5.711151,   11.3642,    -16.95216,  -11.3642,   -5.579505,  0.423434,   6.744026},
            {0,             5.554293,   11.170965,  16.95216,   -23.018482, -17.210218, -11.298533, -5.164407,  1.306221}
        };

        static constexpr float bottomServoMap[9][9] = {
            {-42.129333,    -36.747015, -31.198556,-25.576657,-20.467741,-15.836313,-10.753458,-5.420747,0},
            {-37.334843,    -32.062079, -26.586062,-21.008485,-15.836313,-21.008485,-15.937261,-10.598426,-5.498094},
            {-31.979404,    -26.824999, -21.443005,-15.937261,-10.753458,-21.443005,-21.443005,-15.937261,-10.949558},
            {-26.390838,    -21.336456, -16.038791,-10.598426,-5.420747,-10.598426,-15.937261,-21.008485,-16.268496},
            {-21.364376,    -16.268496, -10.949558,-5.498094,0,5.498094,10.949558,16.268496,21.364376},
            {-16.268496,    -21.008485, -15.937261,-10.598426,5.420747,10.598426,16.038791,21.336456,26.390838},
            {-10.949558,    -15.937261, -21.443005,-21.443005,10.753458,15.937261,21.443005,26.824999,31.979404},
            {-5.498094,     -10.598426, -15.937261,-21.008485,15.836313,21.008485,26.586062,32.062079,37.334843},
            {0,-5.420747,   -10.753458, -15.836313,20.467741,25.576657,31.198556,36.747015,42.129333}
        };


};