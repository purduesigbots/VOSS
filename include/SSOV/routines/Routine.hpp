#pragma once

#include "SSOV/chassis/ChassisCommand.hpp"

namespace ssov {

class Routine {
    public:
        virtual void start() = 0;
        virtual ChassisCommand update() = 0;
        virtual bool finished() = 0;
};

}