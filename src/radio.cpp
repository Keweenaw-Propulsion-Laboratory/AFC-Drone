#include "radio.h"
#include "Arduino.h"

#include "error.h"

// Initialize static variables
RH_RF69 Radio::radio = RH_RF69(RFM69_CS, RFM69_INT); // Construct the radio driver
uint8_t Radio::packetNum = 0; // Set packet number to zero;



Radio::RadioSetupStates Radio::setupState = RESET1;


/**
 * Performs setup on the radio module.
 * 
 * @return Will return true if stage completed successfully
 * Will return false if an error occured. All errors should
 * be treated as fatal
 */
bool Radio::setup() {

    // A variable to help with timing during the setup process
    static uint32_t setupTimmer;
    switch (setupState) {
        case RadioSetupStates::RESET1 :
                pinMode(RFM69_RST, OUTPUT); // Define the reset pin
                // Run reset sequence
                digitalWrite(RFM69_RST, HIGH);

                setupTimmer = millis();

                setupState = RESET2;
                return true;
            break;
        
        case RadioSetupStates::RESET2 :
            if (millis() > (setupTimmer + 10)){
                digitalWrite(RFM69_RST, LOW);
                return false;
            }

            if (millis() > setupTimmer + 20){
                setupState = RADIO_INIT;
            }

            break;
        
        case RadioSetupStates::RADIO_INIT :
            if( !radio.init() ) {
                ErrorHandler::addError(ErrorHandler::radioInitFail);
                Serial.println("Radio start failed");
                return false;
            }
            return true;
            break;

        case RadioSetupStates::SET_CONFIG : {
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

                return true;
        }
            break;
        
        case RadioSetupStates::SEND_CONN : {
            setupTimmer = millis(); // Record time of sent connection ping
            uint8_t testMessage[] = "Hello Houghton. This is AFCDrone";
            sendMessage(testMessage, sizeof(testMessage), MessageType::SETUP);
            return true;
        
        break;
        }
        case RadioSetupStates::WAIT_ACK : {
            
            // Check for ack
            uint8_t recvBuffer[RH_RF69_MAX_MESSAGE_LEN];
            uint8_t buffLength;
            if (!getMessage(recvBuffer, buffLength)) {
                // If no message was ready in the buffer return to loop
                return true;
            }

            if ((buffLength == 8) && (recvBuffer[0] == ACK[0])){
                setupState = COMPLETE;
                return true;
            }

            // Go back to sending a message if the ack hasnt been received
            if(millis() < setupTimmer + 1000) {
                setupState = SEND_CONN;
                }
            }
            break;

        default :
            break;
    }

    return false;

}

bool Radio::setupComplete() {
    return setupState == COMPLETE;
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