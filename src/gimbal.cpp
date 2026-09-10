#include "gimbal.h"

#include "Servo.h"
#include "configs.h"
#include "Arduino.h"

namespace Gimbal {

// Servo GPIO pins
static constexpr int PITCH_SERVO_PIN = 24;
static constexpr int YAW_SERVO_PIN = 25;

// Degrees. Some difference in these is normal to account for tooth placement.
static constexpr int PITCH_ZERO = 90;
static constexpr int YAW_ZERO = 89;

static Servo pitchServo;
static Servo yawServo;

static float bottomServo = 0.0f;
static float topServo = 0.0f;

static float currentPitch = 0.0f;
static float currentYaw = 0.0f;

// MARK: Lookup table for gimbal correction

// Extents of the servo lookup tables, which are transcribed straight from
// docs/ServoLookupTable.csv. That sheet is laid out one ROW per yaw setpoint
// and one COLUMN per pitch setpoint, so the maps are indexed [yaw][pitch].
static constexpr int YAW_ROWS = 9;
static constexpr int PITCH_COLS = 9;


static constexpr float pitchValues[PITCH_COLS] = {-20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0};
static constexpr float yawValues[YAW_ROWS] = {-20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0};

static constexpr float topServoMap[YAW_ROWS][PITCH_COLS] = {
    { -1.306221,   5.164407,  11.298533,  17.210218,  23.018482,  28.964504,  35.107468,  41.307199,  47.574315},
    { -6.744026,  -0.423434,   5.579505,  11.364200,  16.952160,  23.106720,  29.067197,  35.066980,  41.111088},
    {-12.117680,  -5.909759,   0.000000,   5.711151,  11.170965,  17.252006,  23.077774,  28.932407,  34.818812},
    {-17.492678, -11.368308,  -5.512334,   0.155048,   5.554293,  11.437110,  17.176252,  22.937908,  28.724942},
    {-23.018482, -16.753008, -10.928924,  -5.272502,   0.000000,   5.272502,  10.928924,  16.753008,  23.018482},
    {-28.724942, -22.937908, -17.176252, -11.437110,  -5.554293,  -0.155048,   5.512334,  11.368308,  17.492678},
    {-34.818812, -28.932407, -23.077774, -17.252006, -11.170965,  -5.711151,   0.000000,   5.909759,  12.117680},
    {-41.111088, -35.066980, -29.067197, -23.106720, -16.952160, -11.364200,  -5.579505,   0.423434,   6.744026},
    {-47.574315, -41.307199, -35.107468, -28.964504, -23.018482, -17.210218, -11.298533,  -5.164407,   1.306221}
};

static constexpr float bottomServoMap[YAW_ROWS][PITCH_COLS] = {
    {-42.129333, -36.747015, -31.198556, -25.576657, -20.467741, -15.181597,  -9.390883,  -3.672455,   2.003914},
    {-37.334843, -32.062079, -26.586062, -21.008485, -15.836313,  -9.940778,  -4.361664,   1.184972,   6.715591},
    {-31.979404, -26.824999, -21.443005, -15.937261, -10.753458,  -4.804774,   0.643552,   6.080286,  11.513122},
    {-26.390838, -21.336456, -16.038791, -10.598426,  -5.420747,   0.306039,   5.683913,  11.059438,  16.433658},
    {-21.364376, -16.268496, -10.949558,  -5.498094,   0.000000,   5.498094,  10.949558,  16.268496,  21.364376},
    {-16.433658, -11.059438,  -5.683913,  -0.306039,   5.420747,  10.598426,  16.038791,  21.336456,  26.390838},
    {-11.513122,  -6.080286,  -0.643552,   4.804774,  10.753458,  15.937261,  21.443005,  26.824999,  31.979404},
    { -6.715591,  -1.184972,   4.361664,   9.940778,  15.836313,  21.008485,  26.586062,  32.062079,  37.334843},
    { -2.003914,   3.672455,   9.390883,  15.181597,  20.467741,  25.576657,  31.198556,  36.747015,  42.129333}
};

int limitRange(int val, int low, int high){
    if(val > high) {
        val = high;
    } else if (val < low) {
        val = low;
    }

    return val;
}

void setup() {
    pitchServo.attach(PITCH_SERVO_PIN);
    yawServo.attach(YAW_SERVO_PIN);
}

/**
 * Sets the pitch servo to the number of degrees off of zero.
 * 
 * @param angle The number of degrees. Positive moves servo throw arm up.
 */
void setTopServo(float angle) {
    topServo = limitRange(angle + Configs::get().gimbalPitchOffset, 60 , 120);

    pitchServo.write(topServo);   
}

/**
 * Sets the yaw servo to the number of degrees off of zero.
 * 
 * @param angle The number of degrees. Positive moves servo throw arm up.
 */
void setBotServo(float angle) {
    bottomServo = limitRange( -angle + Configs::get().gimbalYawOffset, 60, 120);
    yawServo.write(bottomServo);
}

void set(float pitch, float yaw) {

    // Update set points
    currentPitch = pitch;
    currentYaw = yaw;

    // Bilinear Interpolation
    // https://en.wikipedia.org/wiki/Bilinear_interpolation

    // Clamp the pitch inputs
    if (pitch < pitchValues[0]) pitch = pitchValues[0];
    if (pitch > pitchValues[PITCH_COLS-1]) pitch = pitchValues[PITCH_COLS-1];

    // Clamp the yaw inputs
    if (yaw < yawValues[0]) yaw = yawValues[0];
    if (yaw > yawValues[YAW_ROWS-1]) yaw = yawValues[YAW_ROWS-1];

    // Map the pitch and yaw to the nearest index

    // The row that the setpoint is in.
    uint8_t row = (uint8_t) ((yaw + 20) / 5);
    // The column that the setpoint is in
    uint8_t column = (uint8_t) ((pitch + 20) / 5);

    // Clamp the row index to not go out of bounds
    if (row > YAW_ROWS - 2) row = YAW_ROWS - 2;
    if (column > PITCH_COLS - 2) column = PITCH_COLS - 2;

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

void zero() {
    set(0,0);
}

void selfTest(bool lookup) {
    if (lookup) {
        set(-30, 0);
        delay(1000);
        set(0, -30);
        delay(1000);
        set(30, 0);
        delay(1000);
        set(0, 30);
        delay(1000);
    } else {    
        // Test Servos independently 
        setTopServo(-30);
        delay(1000);

        setBotServo(-30);
        delay(1000);

        setTopServo(30);
        delay(1000);

        setBotServo(30);
        delay(1000);

        setBotServo(0);
        setTopServo(0);
        delay(2000);
    } 
}

float getPitch() {return currentPitch;}
float getYaw() {return currentYaw;}
uint16_t getTopServo() {return topServo;}
uint16_t getBottomServo() {return bottomServo;}

} // namespace Gimbal
