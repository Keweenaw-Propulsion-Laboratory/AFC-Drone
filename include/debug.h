#include "Arduino.h"

class Debug {

    public:
        /**
         * Checks to see if a USB serial connection has been esatblished.
         * 
         * Can be set to pause until a serial connection has been made 
         * using the config option waitForSerial
         * 
         * @return True when ready to move on with loop. 
         * False when waiting for Serial.
         */
        static bool checkSerial();

        /**
         * Wrapper function for Serial.println()
         * 
         * Writes to USB serial when a connection is detected
         * at start up. 
         */
        template <typename type>
        static void println(type message) {
            if (!hasSerial) {return;}

            Serial.println(message);
        }

        /**
         * Wrapper function for Serial.print()
         * 
         * Writes to USB serial when a connection is detected
         * at start up. 
         */
        template <typename type>
        static void print(type message) {
            if (!hasSerial) {return;}

            Serial.print(message);
        }


    private:
        /**
         * True if a serial connection was established in checkSerial.
         */
        static bool hasSerial;

        /**
         * Config variable stored in EEPROM and recalled on power up. 
         */
        static bool waitForSerial; // TODO implement to be stored in EEPROM 

};