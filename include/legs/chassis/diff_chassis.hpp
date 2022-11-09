#pragma once

#include <vector>

#include <legs/chassis/basic_chassis.hpp>


class DiffChassis : public BasicChassis 
{
public:
    
private:

};

class DiffChassisBuilder 
{
public:
    DiffChassisBuilder& leftWheels(std::vector<int> motors);
    DiffChassisBuilder& rightWheels(std::vector<int> motors);

    DiffChassis build();
};