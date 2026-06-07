#define STATUS_LED 10


class Drone {
    public:
    enum class DroneStates {
        BOOT, 
        RADIO_SETUP,  
        SENSOR_SETUP,
        READY_ARMED,
        FLIGHT,
        FAULT_ERROR
    };
    
    static bool startup();
    static void update();


    private:
        static DroneStates state;
        static bool hasSerial;

        static void updateLEDS();
        static void ledFader();
        static void doubleFlash();
};