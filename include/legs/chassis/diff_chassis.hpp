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

    explicit DiffChassisBuilder();

    DiffChassisBuilder &withLeftMotors(const pros::MotorGroup &leftMotors);
    DiffChassisBuilder &withRightMotors(const pros::MotorGroup &rightMotors);

    DiffChassisBuilder &withHorizontalEncoders(const pros::ADIEncoder &leftEncoder, const pros::ADIEncoder &rightEncoder);
    DiffChassisBuilder &withLeftEncoder(const pros::ADIEncoder &leftEncoder);
    DiffChassisBuilder &withRightEncoder(const pros::ADIEncoder &rightEncoder);
    DiffChassisBuilder &withPerpendicularEncoders(const pros::ADIEncoder &perpEncoder);

    DiffChassisBuilder &withHorizontalRotation(const pros::ADIEncoder &rotationEncoder);
    DiffChassisBuilder &withLeftRotation(const pros::ADIEncoder &rotationEncoder);
    DiffChassisBuilder &withRightRotation(const pros::ADIEncoder &rotationEncoder);

    DiffChassisBuilder &withImu(const pros::Imu &imu);
};

} // namespace legs