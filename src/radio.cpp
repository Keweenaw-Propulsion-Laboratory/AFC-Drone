#include "radio.h"
#include "Arduino.h"

#include "error.h"
#include "debug.h"

// Initialize static variables
RH_RF69 Radio::radio = RH_RF69(RFM69_CS, RFM69_INT); // Construct the radio driver
uint8_t Radio::packetNum = 0; // Set packet number to zero;



Radio::RadioSetupStates Radio::setupState = RESET1;

int retryCounter = 0;

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
            }

            if (millis() > setupTimmer + 20){
                setupState = RADIO_INIT;
                Debug::println("Radio Reset");
            }
            return true;

            break;
        
        case RadioSetupStates::RADIO_INIT :
            if( !radio.init() ) {
                ErrorHandler::addError(ErrorHandler::radioInitFail);
                Debug::println("Radio start failed");
                return false;
            }

            setupState = SET_CONFIG;
            return true;
            break;

        case RadioSetupStates::SET_CONFIG : {
                if (!radio.setFrequency(RF69_FREQ)){
                    ErrorHandler::addError(ErrorHandler::radioFreqSetFail);
                    Debug::println("failed to set radio freq");
                    return false;
                }

                // Encryption key must match the receiver (16 bytes exactly)
                uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
                radio.setEncryptionKey(key);  

                radio.setTxPower(20, true); // 20 dbm , Enable high power antenna.
                // Power range is between 14 and 20dbm. 
                // This is the high power variant and we need to enable the high power antenna. 
                
                setupState = SEND_CONN;
                return true;
        }
            break;
        
        case RadioSetupStates::SEND_CONN : {
            setupTimmer = millis(); // Record time of sent connection ping
            uint8_t testMessage[] = "Hello Houghton. This is AFCDrone";
            sendMessage(testMessage, sizeof(testMessage), MessageType::SETUP);

            setupState = WAIT_ACK;
            return true;
        
        break;
        }
        case RadioSetupStates::WAIT_ACK : {
            
            // Check for ack
            uint8_t recvBuffer[RH_RF69_MAX_MESSAGE_LEN];
            uint8_t buffLength;

            // Check if there is an ack waiting
            if (!getMessage(recvBuffer, buffLength)) {
                // Go back to sending a message if the ack hasnt been received after 1 second
                if(millis() > setupTimmer + 1000) {
                    setupTimmer = millis();
                    Debug::print("BaseStation not connected. Retrying... #");
                    Debug::println(retryCounter++);

                    setupState = SEND_CONN;
                }
                return true; // Return back to loop
            }

            // Check if correct ack was recieved 
            if ((buffLength == 8) && (recvBuffer[0] == ACK[0])){
                // Update state to complete the radio init
                Debug::println("BaseStation CONNECTED");
                
                setupState = COMPLETE;
                return true;
            }
            // Return to loop
            return true;
            break;
        }
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