#include <Arduino.h>

#include "gimbal.h"
#include "radio.h"

#define onboard 13;


void setup() {
  // put your setup code here, to run once:
  Gimbal::setup();
  Gimbal::zero();

  delay(1000);

  Gimbal::selfTest();

  Gimbal::zero();

  Radio::setup();


}

void loop() {

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}