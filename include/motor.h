#include <cstdint>

extern uint8_t motor_topSetSpeed;
extern uint8_t motor_bottomSetSpeed;

/** 
 * Sets up required reasources
 */
void motor_setup();

/**
 * Sets the output from 0.0 (stopped) to 1.0 (Full speed)
 */
void motor_setMotor(uint8_t bottomMotorSpeed, uint8_t topMotorSpeed);
