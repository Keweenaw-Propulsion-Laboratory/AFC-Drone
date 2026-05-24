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
    
    static void startup();
    static void update();


    private:
        static DroneStates state;

        static void updateLEDS();

};