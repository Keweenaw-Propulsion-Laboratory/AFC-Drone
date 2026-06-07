#include "debug.h"
#include "Arduino.h"

bool Debug::hasSerial = false;

void Debug::checkSerial() {
    if(Serial) {
        hasSerial = true;
    } else {
        hasSerial = false;
    }


}