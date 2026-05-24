#include "radio.h"
#include "Arduino.h"

#include "error.h"

// Initialize static variables
RH_RF69 Radio::radio = RH_RF69(RFM69_CS, RFM69_INT); // Construct the radio driver
uint8_t Radio::packetNum = 0; // Set packet number to zero;

/**
 * Keeps track of the different stages of Radio setup
 */
enum RadioSetupStates {
    RESET1,
    RESET2,
    RADIO_INIT,
    SET_FREQ,
    SEND_CONN,
    WAIT_ACK,
    COMPLETE
};

bool Radio::setup() {
    while(!Serial); // Wait for a serial connection
    Serial.println("Init Radio");

    pinMode(RFM69_RST, OUTPUT); // Define the reset pin
    // Run reset sequence
    digitalWrite(RFM69_RST, LOW);
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    if( !radio.init() ) {
        ErrorHandler::addError(ErrorHandler::radioInitFail);
        Serial.println("Radio start failed");
        return false;
    }

    if (!radio.setFrequency(RF69_FREQ)){
        ErrorHandler::addError(ErrorHandler::radioFreqSetFail);
        Serial.println("failed to set radio freq");
        return false;
    }

    // Encryption key must match the receiver (16 bytes exactly)
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    radio.setEncryptionKey(key);  


    radio.setTxPower(20, true); // 20 dbm , Enable high power antenna.
    // Power range is between 14 and 20dbm. 
    // This is the high power variant and we need to enable the high power antenna. 

    Serial.println("Radio Init Good");
    // No encryption at this time

    uint8_t testMessage[] = "Hello Houghton. This is AFCDrone";

    uint8_t message[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t messSize = sizeof(message);
    // Send a test message
        

    sendMessage(testMessage, sizeof(testMessage), SETUP);

    radio.waitPacketSent();



    // Wait for BaseStation to respond to confirm connection. 
    
    getMessage( message, messSize);
    delay(500);
    Serial.println("Waiting for ack");
    
    if (message[0] != ACK[0]) {

    }

    Serial.println("BaseStation connected");

    return true;
}

void Radio::sendMessage(uint8_t data[], uint8_t dataSize, MessageType type) {
    radio.waitPacketSent(); // Wait for any previous packet to be sent

    radio.setHeaderId(packetNum); // Set the packet Id to the current packet number
    radio.setHeaderFlags(type); // Set the flags to the type of message.

    radio.send(data, dataSize); // Send the data
}

/**
 * @return Will return true when there is a message. 
 * Will return false if there is no message available.
 */
bool Radio::getMessage(uint8_t (&buffer)[RH_RF69_MAX_MESSAGE_LEN]
                        , uint8_t& bufferLength ) {

    // If radio has no message return false
    if(!radio.available()) {return false;}

    // If the message was valid then put it in the given buffer
    if (!radio.recv(buffer, &bufferLength)) {return false;}

    // Message recieved successfully
    return true;

}