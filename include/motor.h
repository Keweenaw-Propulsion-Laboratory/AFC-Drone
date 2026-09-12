#pragma once

#include <cstdint>

namespace Motor {

uint16_t getBottomSpeed();
uint16_t getTopSpeed();


/** 
 * Sets up required resources
 */
void setup();

/**
 * Sets the output from 0.0 (stopped) to 1.0 (Full speed)
 */
void setMotor(uint8_t bottomMotorSpeed, uint8_t topMotorSpeed);
}
