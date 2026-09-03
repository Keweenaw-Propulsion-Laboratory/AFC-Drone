#define STATUS_LED 10
#include <Arduino.h>
#include <cstdint>

struct Target_t {
    int16_t gimbalX;
    int16_t gimbalY;
    uint8_t motor0Speed;
    uint8_t motor1Speed;
};

extern Target_t drone_targ0;
extern Target_t drone_targ1;
extern bool drone_activeSlot;

extern uint16_t drone_rollAvg;

class Drone {
    public:
    enum class DroneStates: uint8_t {
        BOOT, 
        RADIO_SETUP,  
        SENSOR_SETUP,
        CONTROL_SETUP,
        READY_ARMED,
        FLIGHT,
        FAULT_ERROR
    };
    
    /**
     * This function contains the setup state machine. Any systems that have a non blocking setup
     * routine should be placed here in the appropriate stage. 
     * 
     * @warning No physical setup should occur here. This should only be electronics. 
     */
    static bool startup();

    /**
     * This function contains all of the functions that the drone is expected to do periodically at a cycle of 1 KHz.
     * 
     * 
     */
    static void update();

        static uint16_t lastLoopTime; // How long did the last loop take.
        static uint16_t worstTime; // Keep track of our worst case loop time
        static uint16_t bestTime; // Keep track of our best case loop time
        static DroneStates state; // The current state of the Drone

        // Set by the hardware timer ISR every CONTROL_LOOP_US. loop() polls this
        // and clears it before running the flight control algorithm, so the
        // algorithm itself always executes in normal (non-ISR) context.
        static volatile bool controlTick;

        // Counts ticks where the previous one hadn't been serviced by loop() yet,
        // i.e. the flight control algorithm is taking longer than CONTROL_LOOP_US.
        static volatile uint32_t missedTicks;

    private:
        static bool hasSerial; // Is there a USB Serial connection to debug with

        static IntervalTimer controlTimer; // Hardware timer driving the control loop tick

        static void updateLEDS();
        static void ledFader();
        static void doubleFlash();

        static void startControlTimer();

        // ISR: keep this minimal. No I2C/SPI/Serial calls or heap use here -
        // it only flags that a tick occurred; loop() does the real work.
        static void onControlTick();
};