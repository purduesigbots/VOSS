#pragma once

#include <cstdint>

namespace voss {

enum class Flags {
    NONE = 0b0000,
    ASYNC = 0b0001,
    REVERSE = 0b0010,
    RELATIVE = 0b0100,
    THRU = 0b1000
};

auto inline operator|(Flags flag1, Flags flag2) {
    return static_cast<Flags>(static_cast<uint8_t>(flag1) |
                              static_cast<uint8_t>(flag2));
}

auto inline operator&(Flags flag1, Flags flag2) {
    return static_cast<bool>(static_cast<uint8_t>(flag1) &
                             static_cast<uint8_t>(flag2));
}

enum class AngularDirection {
    AUTO,
    COUNTERCLOCKWISE,
    CCW = COUNTERCLOCKWISE,
    CLOCKWISE,
    CW = CLOCKWISE
};
} // namespace voss