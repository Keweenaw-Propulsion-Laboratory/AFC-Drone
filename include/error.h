#include "circular_buffer.h"

/**
 * This class will be used to define the different errors that could occur.
 * At the time of writing errors will be shown on the builtin led and send 
 * by radio. 
 */
class ErrorHandler {
    /**
     * A inner class of ErrorHanlder
     * Will define what an actual error is.
     */
    class Error {
        public:
        short code;
        using ErrorSeverity = char;
        /**
         * Severity level of the error. 
         * 
         * 0 = Low
         * 1 = Medium
         * 2 = High
         * 3 = Critical / fatal
         */
        ErrorSeverity severity; 
    };
    
    public:
        static constexpr Error radioInitFail{1, 1};        

        static void addError(Error error);
        static bool hasError();
  
    private:
        Circular_Buffer<Error, 256> errorBuffer;
};
