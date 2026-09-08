// #pragma once

// #include "Arduino.h"
// #include "circular_buffer.h"

// /**
//  * This namespace will be used to define the different errors that could occur.
//  * At the time of writing errors will be shown on the builtin led and send 
//  * by radio. 
//  */
// namespace ErrorHandler {
//     /**
//      * A inner class of ErrorHanlder
//      * Will define what an actual error is.
//      */
//     namespace Error {
        
//         short code;
//         using ErrorSeverity = char;
//         /**
//          * Severity level of the error. 
//          * 
//          * 0 = Low
//          * 1 = Medium
//          * 2 = High
//          * 3 = Critical / fatal
//          */
//         ErrorSeverity severity; 
//     };
    
//     // constexpr Error radioInitFail{1, 1};        
//     // constexpr Error radioFreqSetFail{2, 1};

//     // void addError(Error error);
//     // bool hasError();
//     // void serialOut();

//     // static Circular_Buffer<Error, 256> errorBuffer;
//     // static bool serialConn;

// };
