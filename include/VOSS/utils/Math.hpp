#pragma once
#include "cmath"

namespace voss {

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

}; // namespace voss