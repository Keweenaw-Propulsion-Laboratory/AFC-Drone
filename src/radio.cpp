#include "radio.h"
#include "Arduino.h"
#include <cstring>
#include "circular_buffer.h"

#include "error.h"
#include "debug.h"
#include "drone.h"
#include "gimbal.h"
#include "gyro.h"

/** Bool value to skip radio handshake
 * This should only be used for testing
 * and debugging.
 */
constexpr bool SKIP_HANDSHAKE = true;

/**Minimum time to wait in ms between transmissions */
constexpr uint32_t RX_WINDOW_MIN = 10;

// Initialize static variables
RH_RF69 radio = RH_RF69(RFM69_CS, RFM69_INT); // Construct the radio driver
uint8_t globalPacketNum = 0; // Set packet number to zero;
int16_t lastRssi = 0;
uint32_t lastTxTime = 0; /** Last transmission time */


struct __attribute__((packed)) RadioPacket {
    Radio::RadioMessage message;
    Radio::MessageType type;

    // 1. Constructor allowing implicit conversion from '0' (fixes the Circular_Buffer fallback)
    RadioPacket(int = 0) 
        : message{0}, type(Radio::MessageType::SETUP) {}

    // 2. Multi-argument constructor for initializing packets cleanly
    RadioPacket(Radio::RadioMessage msg, Radio::MessageType t) 
        : message(msg), type(t) {}
};

static Circular_Buffer<RadioPacket, 16> radio_tx_buffer; // 16 message tx buffer
static Circular_Buffer<RadioPacket, 16> radio_rx_buffer; // 16 message rx buffer

Radio::RadioSetupStates setupState = Radio::RadioSetupStates::RESET1;

int retryCounter = 0;

void radio_handleCommand(Radio::RadioMessage msg);
void radio_handleConfig(Radio:: RadioMessage msg);
void radio_handleSetup(Radio::RadioMessage msg);


// MARK: Setup
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

                setupState = RadioSetupStates::RESET2;
                return true;
            break;
        
        case RadioSetupStates::RESET2 :
            if (millis() > (setupTimmer + 10)){
                digitalWrite(RFM69_RST, LOW);
            }

            if (millis() > setupTimmer + 20){
                setupState = RadioSetupStates::RADIO_INIT;
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

            setupState = RadioSetupStates::SET_CONFIG;
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
                
                setupState = RadioSetupStates::SEND_CONN;
                return true;
        }
            break;
        
        case RadioSetupStates::SEND_CONN : {
            
            if (SKIP_HANDSHAKE) {
                setupState = RadioSetupStates::COMPLETE;
                return true;
            }

            setupTimmer = millis(); // Record time of sent connection ping
            
            RadioMessage conn;
            memcpy(conn.textArray, "AFCDrone", 8);

            sendMessage(conn, MessageType::SETUP);
            
            setupState = RadioSetupStates::WAIT_ACK;
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

                    setupState = RadioSetupStates::SEND_CONN;
                }
                return true; // Return back to loop
            }

            // Check if correct ack was recieved 
            if ((buffLength == 8) && (recvBuffer[0] == ACK[0])){
                // Update state to complete the radio init
                Debug::println("BaseStation CONNECTED");
                
                setupState = RadioSetupStates::COMPLETE;
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
    return setupState == RadioSetupStates::COMPLETE;
}

// MARK: Periodic Update
void Radio::update() {
    // Get current time;
    uint32_t now = millis();

    // Only run radio if setup has been completed. 
    if (setupComplete()) {

        // Check if radio has available packets
        if (radio.available()) {
            uint8_t buffer[RH_RF69_MAX_MESSAGE_LEN];
            uint8_t len = sizeof(buffer);

            if( radio.recv(buffer, &len) ) { // Get message from radio
                // Save the headers
                uint8_t currentPacketNum = radio.headerId();
                uint8_t messageType = radio.headerFlags();

                // TODO implement packet counting and error checking
                if (currentPacketNum != globalPacketNum + 1){

                }

                // Copy the data from the message
                header_t header = {currentPacketNum, messageType};
                RadioMessage msg;
                memcpy(&msg, buffer + sizeof(header_t), sizeof(RadioMessage));

                switch (static_cast<MessageType>(header.packetType))
                {
                case MessageType::SETUP :
                    /* code */
                    break;
                
                case MessageType::COMMAND :
                    radio_handleCommand(msg);
                    break;

                case MessageType::CONFIG :
                
                default:
                    break;
                }






                // // Convert to a RadioPacket
                // RadioPacket packet = { msg, static_cast<MessageType>(header.packetType)};

                // // Save to rx buffer
                // radio_rx_buffer.push_back(packet);


            }

        }

        // Check if radio is busy. If yes wait
        if (radio.mode() == RHGenericDriver::RHModeTx) return;

        // Check if listen window has been open long enough
        if (now - lastTxTime < RX_WINDOW_MIN) {
            if (radio.mode() != RHGenericDriver::RHModeRx) {
                radio.setModeRx();
            }
            return;
        }

        // Send any messages in the outgoing buffer
        RadioPacket packet;
        if (radio_tx_buffer.size() != 0) {
            packet = radio_tx_buffer.pop_front();

            header_t header{ globalPacketNum++, static_cast<uint8_t>(packet.type) };

            uint8_t frame[sizeof(header_t) + sizeof(RadioMessage)];
            memcpy(frame, &header, sizeof(header_t));
            memcpy(frame + sizeof(header_t), &packet.message, sizeof(RadioMessage));

            radio.send(frame, sizeof(frame)); // Non-blocking transmit start
            lastTxTime = now;
        }


    }
}

/** Adds message to radio queue */
void Radio::sendMessage(RadioMessage data, MessageType type) {
    radio_tx_buffer.push_back({data, type});
}

// MARK: Status Senders

void Radio::sendStatus0() {
    RadioMessage msg;

    msg.status0.loopTimeAvg = Drone::rollAvg;
    msg.status0.loopTimeMax = Drone::worstTime;
    msg.status0.RunTime = millis() / 1000;
    msg.status0.rssi = lastRssi;
    msg.status0.currentMode = (uint8_t) Drone::state;

    sendMessage( msg, MessageType::STATUS0);
}

void Radio::sendStatus1() {
    RadioMessage msg;

    msg.status1.gimbalPitchNorm = Gimbal::getPitch();
    msg.status1.gimbalYawNorm = Gimbal::getYaw();
    msg.status1.topServoSet = Gimbal::getTopServo();
    msg.status1.bottomServoSet = Gimbal::getBottomServo();

    sendMessage(msg, MessageType::STATUS1);
}

void Radio::sendStatus2() {
[[maybe_unused]]    RadioMessage msg;



}

void Radio::sendStatus3() {
[[maybe_unused]]    RadioMessage msg;

    

}

void Radio::sendStatus4() {
[[maybe_unused]]    RadioMessage msg;

    

}

void Radio::sendStatus5() {
[[maybe_unused]]    RadioMessage msg;

    

}

void Radio::sendStatus6() {
[[maybe_unused]]    RadioMessage msg;

    

}

// MARK: Message Handlers

void radio_handleCommand(Radio::RadioMessage msg) {

// Sets a target position
    if (msg.command.flags.targSlot == 0) {
        drone_targ0.gimbalX = msg.command.gimbalX;
        drone_targ0.gimbalY = msg.command.gimbalY;
        drone_targ0.motor0Speed = msg.command.motor0Speed;
        drone_targ0.motor1Speed = msg.command.motor1Speed;
    } else {
        drone_targ1.gimbalX = msg.command.gimbalX;
        drone_targ1.gimbalY = msg.command.gimbalY;
        drone_targ1.motor0Speed = msg.command.motor0Speed;
        drone_targ1.motor1Speed = msg.command.motor1Speed;
    }

    drone_activeSlot = msg.command.flags.activeSlot;




}

void radio_handleConfig(Radio::RadioMessage msg) {


}

void radio_handleSetup(Radio::RadioMessage msg) {


}

// MARK: Radio helpers

bool Radio::getMessage(uint8_t (&buffer)[RH_RF69_MAX_MESSAGE_LEN]
                        , uint8_t& bufferLength ) {

    // If radio has no message return false
    if(!radio.available()) {return false;}

    // If the message was valid then put it in the given buffer
    if (!radio.recv(buffer, &bufferLength)) {return false;}

    // Message recieved successfully
    return true;

}