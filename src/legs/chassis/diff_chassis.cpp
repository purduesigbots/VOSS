#include "legs/chassis/diff_chassis.hpp"

#include "api.h"

namespace legs {

    // make a builder class for the chassis
    
    DiffChassisBuilder::DiffChassisBuilder()
    {
            
        // initialize the builder
    }

    DiffChassisBuilder &DiffChassisBuilder::withLeftMotors(const pros::MotorGroup &leftMotors) {
        this->chassis.leftMotors=std::make_shared<pros::MotorGroup>(leftMotors);
        return *this;
    }

    DiffChassisBuilder &DiffChassisBuilder::withRightMotors(const pros::MotorGroup &rightMotors) {
        this->chassis.rightMotors=std::make_shared<pros::MotorGroup>(rightMotors);
        return *this;
    }

    DiffChassis DiffChassisBuilder::build() {
        return this->chassis;
    }
    // set left motors function

    void DiffChassis::setForwardVelocity(double velocity) {
        this->leftMotors->move_velocity(velocity);
        this->rightMotors->move_velocity(velocity);
    }

    void DiffChassis::setAngularVelocity(double velocity) {
        this->leftMotors->move_velocity(velocity);
        this->rightMotors->move_velocity(-velocity);
    }

    void DiffChassis::setLeftVelocity(double velocity) {
        this->leftMotors->move_velocity(velocity);
    }

    void DiffChassis::setRightVelocity(double velocity) {
        this->rightMotors->move_velocity(velocity);
    }

    

}

