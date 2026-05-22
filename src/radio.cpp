#include "radio.h"
#include "Arduino.h"

#include "error.h"

// Construct the radio driver
RH_RF95 Radio::radio = RH_RF95(RFM69_CS, RFM69_INT);

void Radio::setup() {
    pinMode(RFM69_RST, OUTPUT); // Define the reset pin

    if( !radio.init() ) {
        ErrorHandler::addError(ErrorHandler::radioInitFail);
    }

}

void Radio::sendMessage() {

}