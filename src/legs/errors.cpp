#include "legs/errors.h"

double validate_port_smart(int port) {
    return port > 0 && port < 22;
}

double validate_port_adi(int port) {
    return port > 0 && port < 9;
}