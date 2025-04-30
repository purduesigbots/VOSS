#include "VOSS/utils/debug.hpp"

namespace voss {

bool debug;

void enable_debug() {
    debug = true;
}

void disable_debug() {
    debug = false;
}

bool get_debug() {
    return debug;
}
}