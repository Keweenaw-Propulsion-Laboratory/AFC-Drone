#include <Arduino.h>

#include "gimbal.h"

#define onboard 13;


void setup() {
  // put your setup code here, to run once:
  Gimbal::setup();
  Gimbal::zero();

  delay(1000);

  Gimbal::selfTest();

  Gimbal::zero();


}

void loop() {

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}