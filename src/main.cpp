#include <Arduino.h>

#include "drone.h"
#include "gimbal.h"
#include "radio.h"
#include "gyro.h"

#define onboard 13

#define LOOPTIME 1000

void setup() {
    // put your setup code here, to run once:
    Gimbal::setup();
    Gimbal::zero();

    delay(1000);

    Gimbal::selfTest();

    Gimbal::zero();

    Radio::setup();

    Gyro::setup();


}

void loop() {
    static uint32_t lastLoopTime = 0; // How long did the last loop take.
    static uint32_t worstTime = 0; // Keep track of our worst case loop time
    static uint32_t bestTime = -1; // Keep track of our best case loop time



    static uint32_t startTime = micros(); // What time is it currently?
  
    Drone::update(); // Perform flight logic


    static uint32_t currentLoopCost = micros() - startTime; // How long did the loop take

    static float rollingAverage = 0.0f;
    // Alpha controls smoothing. 0.01 means the average changes smoothly over ~100 loops.
    const float alpha = 0.01f; 
    
    if (rollingAverage == 0.0f) {
        rollingAverage = (float)currentLoopCost; // Initialize on the very first boot loop
    } else {
        rollingAverage = (alpha * (float)currentLoopCost) + ((1.0f - alpha) * rollingAverage);
    }
  
    if (lastLoopTime > worstTime) {worstTime = lastLoopTime;}
    if (lastLoopTime < bestTime) {bestTime = lastLoopTime;}

    // Last time that a status message has been sent
    static uint16_t lastStatus = 0;
    // Send a status message every 2 seconds
    if (millis() - lastStatus > 2000) {
        static uint8_t statusMessage[8];

        statusMessage[7] = ((bestTime >> 8) & 0xFF); // Upper 8 bits of best time
        statusMessage[6] = (bestTime & 0xFF); // Lower 8 bits of best time
        statusMessage[5] = ((worstTime >> 8) & 0xFF); // Upper 8 bits of the worst time
        statusMessage[4] = (worstTime & 0xFF);

        static uint16_t rollAvgInt;

        rollAvgInt = ((int) rollingAverage * 100); 

        statusMessage[3] = ((rollAvgInt >> 8) & 0xFF);
        statusMessage[2] = (rollAvgInt & 0xFF);
        statusMessage[1] = ((lastLoopTime >> 8) & 0xFF);
        statusMessage[0] = (lastLoopTime & 0xFF);
        
        Radio::sendMessage(statusMessage, 8, Radio::MessageType::LOOPTIMES );
    } 

    while(micros() - startTime < LOOPTIME) ; // Wait until the looptime has elapsed 
}

