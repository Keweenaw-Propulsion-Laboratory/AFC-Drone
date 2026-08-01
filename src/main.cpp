#include <Arduino.h>

#include "drone.h"
#include "gimbal.h"
#include "radio.h"
#include "gyro.h"

#define onboard 13

#define LOOPTIME 1000
#define AlPHA_SMOOTHING 0.1f

#define LOOP_STATUS_INTERVAL 2000 // 2000 ms

void setup() {

    while (!Drone::startup()) {}

}

void loop() {
    static uint32_t startTime = micros(); // What time is it currently?
  
    // MARK: Control Loop Logic
    Drone::update(); // Perform flight logic

    // End of Control Loop Logic

    static uint32_t currentLoopCost = micros() - startTime; // How long did the loop take

    static float rollingAverage = 0.0f;
    // Alpha controls smoothing. 0.01 means the average changes smoothly over ~100 loops.
    const float alpha = AlPHA_SMOOTHING; 
    
    if (rollingAverage == 0.0f) {
        rollingAverage = (float)currentLoopCost; // Initialize on the very first boot loop
    } else {
        rollingAverage = (alpha * (float)currentLoopCost) + ((1.0f - alpha) * rollingAverage);
    }
  
    if (Drone::lastLoopTime > Drone::worstTime) {Drone::worstTime = Drone::lastLoopTime;}
    if (Drone::lastLoopTime < Drone::bestTime) {Drone::bestTime = Drone::lastLoopTime;} 

    while(micros() - startTime < LOOPTIME) ; // Wait until the looptime has elapsed 
}

