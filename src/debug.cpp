#include "debug.h"
#include "Arduino.h"

bool Debug::hasSerial = false;
bool Debug::waitForSerial = true; // TODO read from EEPROM

bool Debug::checkSerial() {    
    // Check if USB serial connection has been made. 
    if(Serial) {
        hasSerial = true;
    } else {
        hasSerial = false;
    }

    // If waitForSerial is true then check if serial is true.
    if (waitForSerial && !hasSerial) {
        // If serial has not been established return false
        return false;
    }

    // Return true to move on with setup. 
    return true;
}