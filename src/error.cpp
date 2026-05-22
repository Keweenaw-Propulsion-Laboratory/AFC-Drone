#include "error.h"

void ErrorHandler::addError(Error error) {
    errorBuffer.push_back(error);

    // TODO print to radio and internal log
}

bool ErrorHandler::hasError() {
    return errorBuffer.size() > 0;
}