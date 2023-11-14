#pragma once

#include <cstdint>

namespace voss {

const uint8_t NONE = 0b0000;
const uint8_t ASYNC = 0b0001;
const uint8_t REVERSE = 0b0010;
const uint8_t RELATIVE = 0b0100;
const uint8_t THRU = 0b1000;

} // namespace voss