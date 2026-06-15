#define STATUS_LED 10


class Drone {
    public:
    enum class DroneStates: uint8_t {
        BOOT, 
        RADIO_SETUP,  
        SENSOR_SETUP,
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
        static uint16_t rollAvg;
        static DroneStates state; // The current state of the Drone

    private:
        static bool hasSerial; // Is there a USB Serial connection to debug with


        static void updateLEDS();
        static void ledFader();
        static void doubleFlash();
};