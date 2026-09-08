#include "radio.h"
#include "Arduino.h"
#include <cstring>
#include "circular_buffer.h"
#include <cstdint>

#include "drone.h"
#include "gimbal.h"
#include "gyro.h"
#include "usb.h"
#include "motor.h"
#include "configs.h"

using namespace Radio;

/**Minimum time to wait in ms between transmissions */
static constexpr uint32_t RX_WINDOW_MIN = 10;

static constexpr float RF69_FREQ  = 915.0f;

static constexpr int RFM69_CS = 10;
static constexpr int RFM69_INT = 40;
static constexpr int RFM69_RST = 41;  

// Initialize static variables
RH_RF69 radio = RH_RF69(RFM69_CS, RFM69_INT); // Construct the radio driver
uint8_t radioPacketNum = 0; // Set packet number to zero;

int16_t Radio::radio_avgRSSI = 0;
static float rollingRssi = 0.0f;
static bool rollingRssiInitialized = false;

constexpr float RSSI_ALPHA = 0.1f;

uint32_t lastTxTime = 0; /** Last transmission time */

static constexpr uint8_t TX_SIZE = 16;
static constexpr uint8_t RX_SIZE = 16;

static Circular_Buffer<DataPacket, TX_SIZE> radio_tx_buffer; // 16 message tx buffer
static Circular_Buffer<DataPacket, RX_SIZE> radio_rx_buffer; // 16 message rx buffer

static uint16_t tx_dropped = 0; /** Number of dropped tx packets */

SetupStates setupState = SetupStates::RESET1;

/** Stage 2 state: whether the base station has answered our connection ping. */
static LinkStates linkState = LinkStates::DISCONNECTED;
static uint32_t lastLinkPingTime = 0;

/** How long to wait for an ACK before re-sending the connection ping. */
static constexpr uint32_t LINK_RETRY_MS = 1000;

static void updateLink();

void radio_handleCommand(Message msg);
void radio_handleConfig(Message msg);
void radio_handleSetup(Message msg);

/** Adds message to radio queue */
void radio_sendMessage(Message data, MessageType type);


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
        case SetupStates::RESET1 :
                pinMode(RFM69_RST, OUTPUT); // Define the reset pin
                // Run reset sequence
                digitalWrite(RFM69_RST, HIGH);

                setupTimmer = millis();

                setupState = SetupStates::RESET2;
                return true;
            break;
        
        case SetupStates::RESET2 :
            if (millis() - setupTimmer >= 10){
                digitalWrite(RFM69_RST, LOW);
            }

            if (millis() - setupTimmer >= 20){
                setupState = SetupStates::RADIO_INIT;
                USB::sendText("Radio Reset");
            }
            return true;

            break;
        
        case SetupStates::RADIO_INIT :
            if( !radio.init() ) {
                // ErrorHandler::addError(ErrorHandler::radioInitFail);
                USB::sendText("Radio start failed");
                return false;
            }

            setupState = SetupStates::SET_CONFIG;
            return true;
            break;

        case SetupStates::SET_CONFIG : {
                if (!radio.setFrequency(RF69_FREQ)){
                    // ErrorHandler::addError(ErrorHandler::radioFreqSetFail);
                    USB::sendText("failed to set radio freq");
                    return false;
                }

                // Encryption key must match the receiver (16 bytes exactly)
                uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
                radio.setEncryptionKey(key);  

                radio.setTxPower(Configs::get().txPowerDbm, true); // 20 dbm , Enable high power antenna.
                // Power range is between 14 and 20dbm. 
                // This is the high power variant and we need to enable the high power antenna. 
                
                // Hardware is configured, which is as far as the boot state
                // machine needs to get. Finding the base station is stage 2 and
                // continues in the background from radio_update().
                setupState = SetupStates::COMPLETE;
                return true;
        }
            break;

        default :
            break;
    }

    return false;

}

bool Radio::setupComplete() {
    return setupState == SetupStates::COMPLETE;
}

bool radio_linkConnected() {
    return linkState == LinkStates::CONNECTED;
}

/**
 * Stage 2 of bring-up: poll for the base station without blocking anything.
 *
 * Sends a connection ping, then re-sends it every LINK_RETRY_MS until the base
 * station answers with the ACK pattern. The vehicle arms and flies regardless of
 * whether this ever succeeds, and a link that drops later is retried from here
 * rather than requiring a reboot.
 */
static void updateLink() {
    // The handshake is opt-out; treat it as already satisfied when skipped so
    // radio_linkConnected() still reports something meaningful to telemetry.
    if (Configs::get().skipRadioHandshake) {
        linkState = LinkStates::CONNECTED;
        return;
    }

    if (linkState == LinkStates::CONNECTED) {
        return;
    }

    // Only the first ping is immediate; after that we retry on a fixed cadence.
    if (linkState == LinkStates::AWAITING_ACK &&
        millis() - lastLinkPingTime < LINK_RETRY_MS) {
        return;
    }

    Message conn{};
    memcpy(conn.textArray, "AFCDrone", 8);
    radio_sendMessage(conn, MessageType::SETUP);

    lastLinkPingTime = millis();
    linkState = LinkStates::AWAITING_ACK;
}

// MARK: Periodic Update
void Radio::update() {
    if (!Configs::get().radioEnabled)
        return;

    // Get current time;
    uint32_t now = millis();

    // Only run radio if setup has been completed. 
    if (Radio::setupComplete()) {

        // Stage 2: keep looking for the base station. Runs alongside normal
        // traffic and never gates arming or the control loop.
        updateLink();

        // Check if radio has available packets
        if (radio.available()) {
            uint8_t buffer[RH_RF69_MAX_MESSAGE_LEN];
            uint8_t len = sizeof(buffer);

            if( radio.recv(buffer, &len) ) { // Get message from radio
                int16_t newRssi = radio.lastRssi();

                if (!rollingRssiInitialized) {
                    rollingRssi = static_cast<float>(newRssi);
                    rollingRssiInitialized = true;
                } else {
                    rollingRssi += RSSI_ALPHA *
                                (static_cast<float>(newRssi) - rollingRssi);
                }

                radio_avgRSSI = static_cast<int16_t>(roundf(rollingRssi));
                
                // Save the headers
                uint8_t currentPacketNum = radio.headerId();
                uint8_t messageType = radio.headerFlags();

                // TODO implement packet counting and error checking
                if (currentPacketNum != radioPacketNum + 1){

                }

                // Copy the data from the message
                PacketHeader header = {currentPacketNum, messageType};
                Message msg{};
                if (len != sizeof(msg)) {
                    return;
                }
                memcpy(&msg, buffer, sizeof(msg));

                if (Configs::get().usbRelayEnabled) {
                    USB::radioRelay(msg, static_cast<MessageType>(header.packetType),
                                    header.msgNum, USB::RadioDirection::RECEIVED);
                }

                switch (static_cast<MessageType>(header.packetType))
                {
                case MessageType::SETUP :
                    // The base station answers our connection ping with the ACK
                    // pattern. Accept it whenever it arrives, so a link that
                    // comes back after a dropout reconnects on its own.
                    if (msg.raw == ack.raw &&
                        linkState != LinkStates::CONNECTED) {
                        linkState = LinkStates::CONNECTED;
                        USB::sendText("BaseStation CONNECTED");
                    }
                    break;
                
                case MessageType::COMMAND :
                    radio_handleCommand(msg);
                    break;

                case MessageType::CONFIG :
                    radio_handleConfig(msg);
                    break;
                
                default:
                    break;
                }

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

        // Send one message from the outgoing buffer
        DataPacket packet;
        if (radio_tx_buffer.size() != 0) {
            packet = radio_tx_buffer.pop_front();

            PacketHeader header{ radioPacketNum++, static_cast<uint8_t>(packet.type) };

            uint8_t frame[sizeof(Message)];
            memcpy(frame, &packet.message, sizeof(Message));

            radio.setHeaderId(header.msgNum);
            radio.setHeaderFlags(header.packetType);

            radio.send(frame, sizeof(frame)); // Non-blocking transmit start
            if (Configs::get().usbRelayEnabled) {
                USB::radioRelay(packet.message, packet.type, header.msgNum,
                                USB::RadioDirection::SENT);
            }
            lastTxTime = now;
        }


    }
}

/** Adds message to radio queue */
void radio_sendMessage(Message data, MessageType type) {
    if (radio_tx_buffer.size() >= TX_SIZE)
        tx_dropped++;

    radio_tx_buffer.push_back({data, type});
 
}

// MARK: Status Senders

void Radio::sendStatus0(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status0.loopTimeAvg = t.loopTimeAvg;
    msg.status0.loopTimeMax = t.loopTimeMax;
    msg.status0.RunTime = t.runtimeSec;
    msg.status0.currentMode = (uint8_t) t.state;

    radio_sendMessage( msg, MessageType::STATUS0);
}

void Radio::sendStatus1(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status1.gimbalPitchNorm = t.gimbalPitch;
    msg.status1.gimbalYawNorm = t.gimbalYaw;
    msg.status1.topServoSet = t.topServoSet;
    msg.status1.bottomServoSet = t.bottomServoSet;

    radio_sendMessage(msg, MessageType::STATUS1);
}

void Radio::sendStatus2(const Drone::Telemetry_t& t) {
    Message msg{};
    
    msg.status2.bottomMotorSet = t.bottomMotorSet;
    msg.status2.topMotorSet = t.topMotorSet;
    msg.status2.voltage = 0; // TODO connect to battery monitor @crheilma-code
    // RSSI is owned by loop() context, not the control tick, so it is read
    // live rather than coming from the snapshot.
    msg.status2.rssi = radio_avgRSSI;

    radio_sendMessage(msg, MessageType::STATUS2);

}

void Radio::sendStatus3(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status3.qR = floatToFixed(t.qR, RADIO_QUAT_SCALE);
    msg.status3.qI = floatToFixed(t.qI, RADIO_QUAT_SCALE);
    msg.status3.qJ = floatToFixed(t.qJ, RADIO_QUAT_SCALE);
    msg.status3.qK = floatToFixed(t.qK, RADIO_QUAT_SCALE);

    radio_sendMessage(msg, MessageType::STATUS3);
}

void Radio::sendStatus4(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status4.accelX = floatToFixed(t.accelX, RADIO_ACCEL_SCALE);
    msg.status4.accelY = floatToFixed(t.accelY, RADIO_ACCEL_SCALE);
    msg.status4.accelZ = floatToFixed(t.accelZ, RADIO_ACCEL_SCALE);
    msg.status4.empty = 0;

    radio_sendMessage(msg, MessageType::STATUS4);

}

void Radio::sendStatus5(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status5.velX = floatToFixed(t.velX, RADIO_VEL_SCALE);
    msg.status5.velY = floatToFixed(t.velY, RADIO_VEL_SCALE);
    msg.status5.velZ = floatToFixed(t.velZ, RADIO_VEL_SCALE);
    msg.status5.empty = 0;

    radio_sendMessage(msg, MessageType::STATUS5);

}

void Radio::sendStatus6(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status6.posX = floatToFixed(t.posX, RADIO_POS_SCALE);
    msg.status6.posY = floatToFixed(t.posY, RADIO_POS_SCALE);
    msg.status6.posZ = floatToFixed(t.posZ, RADIO_POS_SCALE);
    msg.status6.empty = 0;

    radio_sendMessage(msg, MessageType::STATUS6);
}

// MARK: Message Handlers

/**
 * Handle incoming flight commands from the base station. 
 */
void radio_handleCommand(Message msg) {

    Drone::Target_t target;
    target.gimbalX = msg.command.gimbalX;
    target.gimbalY = msg.command.gimbalY;
    target.bottomMotor = msg.command.bottomMotor;
    target.topMotor = msg.command.topMotor;

    Drone::setTarget(target);

}

void radio_handleConfig(Message msg) {
    Message response{};
    
    if (msg.config.version != Configs::CONFIG_VERSION){
        response.config.version = Configs::CONFIG_VERSION;
        response.config.state.result = Configs::ConfigResult::UNKNOWN_VERSION;
        response.config.configKey = msg.config.configKey;
        radio_sendMessage(response, MessageType::CONFIG);
        return;
    }

    switch (msg.config.state.operation)
    {
    case Configs::ConfigOp::READ :
        response.config.version = Configs::CONFIG_VERSION;
        response.config.configKey = msg.config.configKey;
        response.config.value = Configs::read(msg.config.configKey,
                                            response.config.state.result);
        radio_sendMessage(response, MessageType::CONFIG);
        break;
    case Configs::ConfigOp::SET :
        response.config.version = Configs::CONFIG_VERSION;
        response.config.configKey = msg.config.configKey;
        response.config.state.result =
            Configs::set(msg.config.configKey, msg.config.value);
        radio_sendMessage(response, MessageType::CONFIG);
        break;
    default:
        response.config.version = Configs::CONFIG_VERSION;
        response.config.configKey = (Configs::ConfigKey) -1;
        response.config.state.result = Configs::ConfigResult::UNKNOWN_OP;
        radio_sendMessage(response, MessageType::CONFIG);
        break;
    }

}

void radio_handleSetup(Message msg) {


}

// MARK: Radio helpers
[[maybe_unused]]
static bool radio_getMessage(uint8_t (&buffer)[RH_RF69_MAX_MESSAGE_LEN]
                        , uint8_t& bufferLength ) {

    // If radio has no message return false
    if(!radio.available()) {return false;}

    // If the message was valid then put it in the given buffer
    if (!radio.recv(buffer, &bufferLength)) {return false;}

    // Message recieved successfully
    return true;

}
