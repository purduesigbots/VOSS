#pragma once

#include <vector>

#include "api.h"
#include "legs/chassis/basic_chassis.hpp"
#include "legs/chassis/distance_tracker.hpp"
namespace legs {

/* Forward declarations so that both classes are aware of each other*/
class DiffChassisBuilder;
class DiffChassis;

class DiffChassis : public BasicChassis
{
        friend class DiffChassisBuilder;
        private:
            std::shared_ptr<pros::MotorGroup> leftMotors;
            std::shared_ptr<pros::MotorGroup> rightMotors;

        public:
            virtual void setForwardVelocity(double velocity) override;
            virtual void setAngularVelocity(double velocity) override;
            virtual void setLeftVelocity(double velocity);
            virtual void setRightVelocity(double velocity);

};

class DiffChassisBuilder
{

    explicit DiffChassisBuilder();

    DiffChassisBuilder &withLeftMotors(const pros::MotorGroup &leftMotors);
    DiffChassisBuilder &withRightMotors(const pros::MotorGroup &rightMotors);
    DiffChassis build();

    private:
        DiffChassis chassis;
};

} // namespace legs