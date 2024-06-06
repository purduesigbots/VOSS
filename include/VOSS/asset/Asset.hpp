#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {

typedef struct __attribute__((__packed__)) _asset {
    uint8_t* buf;
    size_t size;
} asset;
}

#define ASSET(x)                                                               \
    extern "C" {                                                               \
    extern uint8_t _binary_static_##x##_start[], _binary_static_##x##_size[];  \
    static asset x = {_binary_static_##x##_start,                              \
                      (size_t)_binary_static_##x##_size};                      \
    }
// fun fact this comment disabled the warning cursed by macro and auto format
// (don't delete it)