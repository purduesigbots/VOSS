#include "legs/chassis/diff_chassis.hpp"

#include "api.h"

namespace legs {

    // make a builder class for the chassis
    
    DiffChassisBuilder::DiffChassisBuilder()
    {
            
        // initialize the builder
    }

    DiffChassisBuilder &DiffChassisBuilder::withImu(const pros::Imu &imu)
    {
        this->chassis.imu=std::make_shared<pros::Imu>(imu);
    }

    DiffChassisBuilder &DiffChassisBuilder::withImu(const int port)
    {
        std::shared_ptr<pros::Imu> tmp = std::make_shared<pros::Imu>(port);
        return this->withImu(*tmp);
    }

    DiffChassis DiffChassisBuilder::build() {
        if(this->chassis.leftEncoder==nullptr) {
            if(this->chassis.leftMotors) {
                
            }
        }
        return this->chassis;
    }


    // set left motors function





}