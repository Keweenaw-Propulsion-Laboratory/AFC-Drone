#include "Arduino.h"

class Debug {

    public:
        static void checkSerial();

        template <typename type>
        static void println(type message) {
            if (!hasSerial) {return;}

            Serial.println(message);
        }

    private:
        static bool hasSerial;

};