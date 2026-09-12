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

namespace Radio {

/**Minimum time to wait in ms between transmissions */
static constexpr uint32_t RX_WINDOW_MIN = 10;

static constexpr float RF69_FREQ  = 915.0f;

static constexpr int RFM69_CS = 10;
static constexpr int RFM69_INT = 40;
static constexpr int RFM69_RST = 41;  

// Initialize static variables
static RH_RF69 radio = RH_RF69(RFM69_CS, RFM69_INT); // Construct the radio driver
static uint8_t radioPacketNum = 0; // Set packet number to zero;

static int16_t avgRSSI = 0;

/** Rolling average RSSI of received packets, in dBm. */
int16_t getAvgRSSI() {return avgRSSI;}
static float rollingRssi = 0.0f;
static bool rollingRssiInitialized = false;

constexpr float RSSI_ALPHA = 0.1f;

static uint32_t lastTxTime = 0; /** Last transmission time */

static constexpr uint8_t TX_SIZE = 16;
static constexpr uint8_t RX_SIZE = 16;

static Circular_Buffer<DataPacket, TX_SIZE> txBuffer; // 16 message tx buffer
static Circular_Buffer<DataPacket, RX_SIZE> rxBuffer; // 16 message rx buffer

static uint16_t txDropped = 0; /** Number of dropped tx packets */

static SetupStates setupState = SetupStates::RESET1;

/** Stage 2 state: whether the base station has answered our connection ping. */
static LinkStates linkState = LinkStates::DISCONNECTED;
static uint32_t lastLinkPingTime = 0;

/** How long to wait for an ACK before re-sending the connection ping. */
static constexpr uint32_t LINK_RETRY_MS = 1000;

static void updateLink();

static void handleCommand(Message msg);
static void handleConfig(Message msg);
// TODO: not yet wired into the update() dispatch - the SETUP case currently
// handles the base-station ACK inline. Kept as the hook for the rest of the
// handshake; [[maybe_unused]] so internal linkage does not trip -Werror.
[[maybe_unused]] static void handleSetup(Message msg);

/** Adds message to radio queue */
static void sendMessage(Message data, MessageType type);


// MARK: Setup
/**
 * Performs setup on the radio module.
 * 
 * @return Will return true if stage completed successfully
 * Will return false if an error occured. All errors should
 * be treated as fatal
 */
bool setup() {

    // A variable to help with timing during the setup process
    static uint32_t setupTimer;

    switch (setupState) {
        case SetupStates::RESET1 :
                pinMode(RFM69_RST, OUTPUT); // Define the reset pin
                // Run reset sequence
                digitalWrite(RFM69_RST, HIGH);

                setupTimer = millis();

                setupState = SetupStates::RESET2;
                return true;
            break;
        
        case SetupStates::RESET2 :
            if (millis() - setupTimer >= 10){
                digitalWrite(RFM69_RST, LOW);
            }

            if (millis() - setupTimer >= 20){
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
                // continues in the background from update().
                setupState = SetupStates::COMPLETE;
                return true;
        }
            break;

        default :
            break;
    }

    return false;

}

bool setupComplete() {
    return setupState == SetupStates::COMPLETE;
}

bool linkConnected() {
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
    // linkConnected() still reports something meaningful to telemetry.
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
    sendMessage(conn, MessageType::SETUP);

    lastLinkPingTime = millis();
    linkState = LinkStates::AWAITING_ACK;
}

// MARK: Periodic Update
void update() {
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

                avgRSSI = static_cast<int16_t>(roundf(rollingRssi));
                
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
                    handleCommand(msg);
                    break;

                case MessageType::CONFIG :
                    handleConfig(msg);
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
        if (txBuffer.size() != 0) {
            packet = txBuffer.pop_front();

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
static void sendMessage(Message data, MessageType type) {
    if (txBuffer.size() >= TX_SIZE)
        txDropped++;

    txBuffer.push_back({data, type});
 
}

// MARK: Status Senders

void sendStatus0(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status0.loopTimeAvg = t.loopTimeAvg;
    msg.status0.loopTimeMax = t.loopTimeMax;
    msg.status0.RunTime = t.runtimeSec;
    msg.status0.currentMode = (uint8_t) t.state;

    sendMessage( msg, MessageType::STATUS0);
}

void sendStatus1(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status1.gimbalPitchNorm = t.gimbalPitch;
    msg.status1.gimbalYawNorm = t.gimbalYaw;
    msg.status1.topServoSet = t.topServoSet;
    msg.status1.bottomServoSet = t.bottomServoSet;

    sendMessage(msg, MessageType::STATUS1);
}

void sendStatus2(const Drone::Telemetry_t& t) {
    Message msg{};
    
    msg.status2.bottomMotorSet = t.bottomMotorSet;
    msg.status2.topMotorSet = t.topMotorSet;
    msg.status2.voltage = 0; // TODO connect to battery monitor @crheilma-code
    // RSSI is owned by loop() context, not the control tick, so it is read
    // live rather than coming from the snapshot.
    msg.status2.rssi = avgRSSI;

    sendMessage(msg, MessageType::STATUS2);

}

void sendStatus3(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status3.qR = floatToFixed(t.qR, RADIO_QUAT_SCALE);
    msg.status3.qI = floatToFixed(t.qI, RADIO_QUAT_SCALE);
    msg.status3.qJ = floatToFixed(t.qJ, RADIO_QUAT_SCALE);
    msg.status3.qK = floatToFixed(t.qK, RADIO_QUAT_SCALE);

    sendMessage(msg, MessageType::STATUS3);
}

void sendStatus4(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status4.accelX = floatToFixed(t.accelX, RADIO_ACCEL_SCALE);
    msg.status4.accelY = floatToFixed(t.accelY, RADIO_ACCEL_SCALE);
    msg.status4.accelZ = floatToFixed(t.accelZ, RADIO_ACCEL_SCALE);
    msg.status4.empty = 0;

    sendMessage(msg, MessageType::STATUS4);

}

void sendStatus5(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status5.velX = floatToFixed(t.velX, RADIO_VEL_SCALE);
    msg.status5.velY = floatToFixed(t.velY, RADIO_VEL_SCALE);
    msg.status5.velZ = floatToFixed(t.velZ, RADIO_VEL_SCALE);
    msg.status5.empty = 0;

    sendMessage(msg, MessageType::STATUS5);

}

void sendStatus6(const Drone::Telemetry_t& t) {
    Message msg{};

    msg.status6.posX = floatToFixed(t.posX, RADIO_POS_SCALE);
    msg.status6.posY = floatToFixed(t.posY, RADIO_POS_SCALE);
    msg.status6.posZ = floatToFixed(t.posZ, RADIO_POS_SCALE);
    msg.status6.empty = 0;

    sendMessage(msg, MessageType::STATUS6);
}

// MARK: Message Handlers

/**
 * Handle incoming flight commands from the base station. 
 */
static void handleCommand(Message msg) {

    Drone::Target_t target;
    target.gimbalX = msg.command.gimbalX;
    target.gimbalY = msg.command.gimbalY;
    target.bottomMotor = msg.command.bottomMotor;
    target.topMotor = msg.command.topMotor;

    Drone::setTarget(target);

}

static void handleConfig(Message msg) {
    Message response{};
    
    if (msg.config.version != Configs::CONFIG_VERSION){
        response.config.version = Configs::CONFIG_VERSION;
        response.config.state.result = Configs::ConfigResult::UNKNOWN_VERSION;
        response.config.configKey = msg.config.configKey;
        sendMessage(response, MessageType::CONFIG);
        return;
    }

    switch (msg.config.state.operation)
    {
    case Configs::ConfigOp::READ :
        response.config.version = Configs::CONFIG_VERSION;
        response.config.configKey = msg.config.configKey;
        response.config.value = Configs::read(msg.config.configKey,
                                            response.config.state.result);
        sendMessage(response, MessageType::CONFIG);
        break;
    case Configs::ConfigOp::SET :
        response.config.version = Configs::CONFIG_VERSION;
        response.config.configKey = msg.config.configKey;
        response.config.state.result =
            Configs::set(msg.config.configKey, msg.config.value);
        sendMessage(response, MessageType::CONFIG);
        break;
    default:
        response.config.version = Configs::CONFIG_VERSION;
        response.config.configKey = (Configs::ConfigKey) -1;
        response.config.state.result = Configs::ConfigResult::UNKNOWN_OP;
        sendMessage(response, MessageType::CONFIG);
        break;
    }

}

static void handleSetup(Message msg) {


}

// MARK: Radio helpers
[[maybe_unused]]
static bool getMessage(uint8_t (&buffer)[RH_RF69_MAX_MESSAGE_LEN]
                        , uint8_t& bufferLength ) {

    // If radio has no message return false
    if(!radio.available()) {return false;}

    // If the message was valid then put it in the given buffer
    if (!radio.recv(buffer, &bufferLength)) {return false;}

    // Message recieved successfully
    return true;

}

} // namespace Radio
