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
    
            std::shared_ptr<legs::BaseDistanceTracker> leftEncoder;
            std::shared_ptr<legs::BaseDistanceTracker> rightEncoder;
            std::shared_ptr<legs::BaseDistanceTracker> perpEncoder;

            std::shared_ptr<pros::Imu> imu;

        public:
            virtual void setForwardVelocity(double velocity) override;
};

class DiffChassisBuilder
{

    explicit DiffChassisBuilder();

    DiffChassisBuilder &withLeftMotors(const pros::MotorGroup &leftMotors);
    DiffChassisBuilder &withRightMotors(const pros::MotorGroup &rightMotors);

    DiffChassisBuilder &withEncoders(std::vector<legs::BaseDistanceTracker> encoders);
    DiffChassisBuilder &withImu(const pros::Imu &imu);
    DiffChassisBuilder &withImu(const int port);

    DiffChassis build();

    private:
        DiffChassis chassis;
        short withImuCount;
        short
};

} // namespace legs