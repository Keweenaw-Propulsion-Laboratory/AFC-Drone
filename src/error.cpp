#include "error.h"
#include "Arduino.h"

bool ErrorHandler::serialConn{false};
Circular_Buffer<ErrorHandler::Error, 256> ErrorHandler::errorBuffer;

void ErrorHandler::addError(Error error) {
    errorBuffer.push_back(error);

    // TODO print to radio and internal log
}

bool ErrorHandler::hasError() {
    return errorBuffer.size() > 0;
}
