#include "radio.h"
#include "Arduino.h"
#include <cstring>
#include "circular_buffer.h"
#include <cstdint>

#include "error.h"
#include "drone.h"
#include "gimbal.h"
#include "gyro.h"
#include "usb.h"
#include "motor.h"
#include "configs.h"

/**Minimum time to wait in ms between transmissions */
constexpr uint32_t RX_WINDOW_MIN = 10;

// Initialize static variables
RH_RF69 radio = RH_RF69(RFM69_CS, RFM69_INT); // Construct the radio driver
uint8_t radioPacketNum = 0; // Set packet number to zero;

int16_t radio_avgRSSI = 0;
static float rollingRssi = 0.0f;
static bool rollingRssiInitialized = false;

constexpr float RSSI_ALPHA = 0.1f;

uint32_t lastTxTime = 0; /** Last transmission time */

static constexpr uint8_t TX_SIZE = 16;
static constexpr uint8_t RX_SIZE = 16;

static Circular_Buffer<radio_Packet, TX_SIZE> radio_tx_buffer; // 16 message tx buffer
static Circular_Buffer<radio_Packet, RX_SIZE> radio_rx_buffer; // 16 message rx buffer

static uint16_t tx_dropped = 0; /** Number of dropped tx packets */

radio_SetupStates setupState = radio_SetupStates::RESET1;

/** Stage 2 state: whether the base station has answered our connection ping. */
static radio_LinkStates linkState = radio_LinkStates::DISCONNECTED;
static uint32_t lastLinkPingTime = 0;

/** How long to wait for an ACK before re-sending the connection ping. */
static constexpr uint32_t LINK_RETRY_MS = 1000;

static void radio_updateLink();

void radio_handleCommand(radio_Message msg);
void radio_handleConfig(radio_Message msg);
void radio_handleSetup(radio_Message msg);

/** Adds message to radio queue */
void radio_sendMessage(radio_Message data, radio_MessageType type);


// MARK: Setup
/**
 * Performs setup on the radio module.
 * 
 * @return Will return true if stage completed successfully
 * Will return false if an error occured. All errors should
 * be treated as fatal
 */
bool radio_setup() {

    // A variable to help with timing during the setup process
    static uint32_t setupTimmer;

    switch (setupState) {
        case radio_SetupStates::RESET1 :
                pinMode(RFM69_RST, OUTPUT); // Define the reset pin
                // Run reset sequence
                digitalWrite(RFM69_RST, HIGH);

                setupTimmer = millis();

                setupState = radio_SetupStates::RESET2;
                return true;
            break;
        
        case radio_SetupStates::RESET2 :
            if (millis() - setupTimmer >= 10){
                digitalWrite(RFM69_RST, LOW);
            }

            if (millis() - setupTimmer >= 20){
                setupState = radio_SetupStates::RADIO_INIT;
                usb_send_text("Radio Reset");
            }
            return true;

            break;
        
        case radio_SetupStates::RADIO_INIT :
            if( !radio.init() ) {
                ErrorHandler::addError(ErrorHandler::radioInitFail);
                usb_send_text("Radio start failed");
                return false;
            }

            setupState = radio_SetupStates::SET_CONFIG;
            return true;
            break;

        case radio_SetupStates::SET_CONFIG : {
                if (!radio.setFrequency(RF69_FREQ)){
                    ErrorHandler::addError(ErrorHandler::radioFreqSetFail);
                    usb_send_text("failed to set radio freq");
                    return false;
                }

                // Encryption key must match the receiver (16 bytes exactly)
                uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
                radio.setEncryptionKey(key);  

                radio.setTxPower(config_get().txPowerDbm, true); // 20 dbm , Enable high power antenna.
                // Power range is between 14 and 20dbm. 
                // This is the high power variant and we need to enable the high power antenna. 
                
                // Hardware is configured, which is as far as the boot state
                // machine needs to get. Finding the base station is stage 2 and
                // continues in the background from radio_update().
                setupState = radio_SetupStates::COMPLETE;
                return true;
        }
            break;

        default :
            break;
    }

    return false;

}

bool radio_setupComplete() {
    return setupState == radio_SetupStates::COMPLETE;
}

bool radio_linkConnected() {
    return linkState == radio_LinkStates::CONNECTED;
}

/**
 * Stage 2 of bring-up: poll for the base station without blocking anything.
 *
 * Sends a connection ping, then re-sends it every LINK_RETRY_MS until the base
 * station answers with the ACK pattern. The vehicle arms and flies regardless of
 * whether this ever succeeds, and a link that drops later is retried from here
 * rather than requiring a reboot.
 */
static void radio_updateLink() {
    // The handshake is opt-out; treat it as already satisfied when skipped so
    // radio_linkConnected() still reports something meaningful to telemetry.
    if (config_get().skipRadioHandshake) {
        linkState = radio_LinkStates::CONNECTED;
        return;
    }

    if (linkState == radio_LinkStates::CONNECTED) {
        return;
    }

    // Only the first ping is immediate; after that we retry on a fixed cadence.
    if (linkState == radio_LinkStates::AWAITING_ACK &&
        millis() - lastLinkPingTime < LINK_RETRY_MS) {
        return;
    }

    radio_Message conn{};
    memcpy(conn.textArray, "AFCDrone", 8);
    radio_sendMessage(conn, radio_MessageType::SETUP);

    lastLinkPingTime = millis();
    linkState = radio_LinkStates::AWAITING_ACK;
}

// MARK: Periodic Update
void radio_update() {
    if (!config_get().radioEnabled)
        return;

    // Get current time;
    uint32_t now = millis();

    // Only run radio if setup has been completed. 
    if (radio_setupComplete()) {

        // Stage 2: keep looking for the base station. Runs alongside normal
        // traffic and never gates arming or the control loop.
        radio_updateLink();

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
                radio_Header header = {currentPacketNum, messageType};
                radio_Message msg{};
                if (len != sizeof(msg)) {
                    return;
                }
                memcpy(&msg, buffer, sizeof(msg));

                if (config_get().usbRelayEnabled) {
                    usb_radio_relay(msg, static_cast<radio_MessageType>(header.packetType),
                                    header.msgNum, usb_radio_direction::RECEIVED);
                }

                switch (static_cast<radio_MessageType>(header.packetType))
                {
                case radio_MessageType::SETUP :
                    // The base station answers our connection ping with the ACK
                    // pattern. Accept it whenever it arrives, so a link that
                    // comes back after a dropout reconnects on its own.
                    if (msg.raw == ack.raw &&
                        linkState != radio_LinkStates::CONNECTED) {
                        linkState = radio_LinkStates::CONNECTED;
                        usb_send_text("BaseStation CONNECTED");
                    }
                    break;
                
                case radio_MessageType::COMMAND :
                    radio_handleCommand(msg);
                    break;

                case radio_MessageType::CONFIG :
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
        radio_Packet packet;
        if (radio_tx_buffer.size() != 0) {
            packet = radio_tx_buffer.pop_front();

            radio_Header header{ radioPacketNum++, static_cast<uint8_t>(packet.type) };

            uint8_t frame[sizeof(radio_Message)];
            memcpy(frame, &packet.message, sizeof(radio_Message));

            radio.setHeaderId(header.msgNum);
            radio.setHeaderFlags(header.packetType);

            radio.send(frame, sizeof(frame)); // Non-blocking transmit start
            if (config_get().usbRelayEnabled) {
                usb_radio_relay(packet.message, packet.type, header.msgNum,
                                usb_radio_direction::SENT);
            }
            lastTxTime = now;
        }


    }
}

/** Adds message to radio queue */
void radio_sendMessage(radio_Message data, radio_MessageType type) {
    if (radio_tx_buffer.size() >= TX_SIZE)
        tx_dropped++;

    radio_tx_buffer.push_back({data, type});
 
}

// MARK: Status Senders

void radio_sendStatus0() {
    radio_Message msg{};

    msg.status0.loopTimeAvg = drone_rollAvg;
    msg.status0.loopTimeMax = Drone::worstTime;
    msg.status0.RunTime = millis() / 1000;
    msg.status0.currentMode = (uint8_t) Drone::state;

    radio_sendMessage( msg, radio_MessageType::STATUS0);
}

void radio_sendStatus1() {
    radio_Message msg{};

    msg.status1.gimbalPitchNorm = gimbal_pitch;
    msg.status1.gimbalYawNorm = gimbal_yaw;
    msg.status1.topServoSet = gimbal_topServo;
    msg.status1.bottomServoSet = gimbal_botServo;

    radio_sendMessage(msg, radio_MessageType::STATUS1);
}

void radio_sendStatus2() {
    radio_Message msg{};
    
    msg.status2.motor1set = motor_bottomSetSpeed;
    msg.status2.motor2set = motor_topSetSpeed;
    msg.status2.voltage = 0; // TODO connect to battery monitor @crheilma-code
    msg.status2.rssi = radio_avgRSSI;

    radio_sendMessage(msg, radio_MessageType::STATUS2);

}

void radio_sendStatus3() {
    radio_Message msg{};

    msg.status3.qR = radio_floatToFixed(Gyro::droneQuatReal, RADIO_QUAT_SCALE);
    msg.status3.qI = radio_floatToFixed(Gyro::droneQuatI, RADIO_QUAT_SCALE);
    msg.status3.qJ = radio_floatToFixed(Gyro::droneQuatJ, RADIO_QUAT_SCALE);
    msg.status3.qK = radio_floatToFixed(Gyro::droneQuatK, RADIO_QUAT_SCALE);

    radio_sendMessage(msg, radio_MessageType::STATUS3);
}

void radio_sendStatus4() {
    radio_Message msg{};

    msg.status4.accelX = radio_floatToFixed(Gyro::worldAccelX, RADIO_ACCEL_SCALE);
    msg.status4.accelY = radio_floatToFixed(Gyro::worldAccelY, RADIO_ACCEL_SCALE);
    msg.status4.accelZ = radio_floatToFixed(Gyro::worldAccelZ, RADIO_ACCEL_SCALE);
    msg.status4.empty = 0;

    radio_sendMessage(msg, radio_MessageType::STATUS4);

}

void radio_sendStatus5() {
    radio_Message msg{};

    msg.status5.velX = radio_floatToFixed(Gyro::droneState.velocity.x, RADIO_VEL_SCALE);
    msg.status5.velY = radio_floatToFixed(Gyro::droneState.velocity.y, RADIO_VEL_SCALE);
    msg.status5.velZ = radio_floatToFixed(Gyro::droneState.velocity.z, RADIO_VEL_SCALE);
    msg.status5.empty = 0;

    radio_sendMessage(msg, radio_MessageType::STATUS5);

}

void radio_sendStatus6() {
    radio_Message msg{};

    msg.status6.posX = radio_floatToFixed(Gyro::droneState.position.x, RADIO_POS_SCALE);
    msg.status6.posY = radio_floatToFixed(Gyro::droneState.position.y, RADIO_POS_SCALE);
    msg.status6.posZ = radio_floatToFixed(Gyro::droneState.position.z, RADIO_POS_SCALE);
    msg.status6.empty = 0;

    radio_sendMessage(msg, radio_MessageType::STATUS6);
}

// MARK: Message Handlers

void radio_handleCommand(radio_Message msg) {

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

void radio_handleConfig(radio_Message msg) {
    radio_Message response{};
    
    if (msg.config.version != CONFIG_VERSION){
        response.config.version = CONFIG_VERSION;
        response.config.state.result = ConfigResult::UNKNOWN_VERSION;
        response.config.configKey = msg.config.configKey;
        radio_sendMessage(response, radio_MessageType::CONFIG);
        return;
    }

    switch (msg.config.state.operation)
    {
    case ConfigOp::READ :
        response.config.version = CONFIG_VERSION;
        response.config.configKey = msg.config.configKey;
        response.config.value = config_read(msg.config.configKey,
                                            response.config.state.result);
        radio_sendMessage(response, radio_MessageType::CONFIG);
        break;
    case ConfigOp::SET :
        response.config.version = CONFIG_VERSION;
        response.config.configKey = msg.config.configKey;
        response.config.state.result =
            config_set(msg.config.configKey, msg.config.value);
        radio_sendMessage(response, radio_MessageType::CONFIG);
        break;
    default:
        response.config.version = CONFIG_VERSION;
        response.config.configKey = (ConfigKey) -1;
        response.config.state.result = ConfigResult::UNKNOWN_OP;
        radio_sendMessage(response, radio_MessageType::CONFIG);
        break;
    }

}

void radio_handleSetup(radio_Message msg) {


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
