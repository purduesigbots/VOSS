#pragma once

#include <vector>

#include "api.h"
#include "legs/chassis/basic_chassis.hpp"

namespace legs {

/* Forward declarations so that both classes are aware of each other*/
class DiffChassisBuilder;
class DiffChassis;

class DiffChassis : public BasicChassis
{

};

class DiffChassisBuilder
{
    
};

} // namespace legs