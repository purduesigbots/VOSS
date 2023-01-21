#pragma once

#include "Eigen/Core"
#include "legs/motion_models/basic_model.hpp"
#include "legs/motion_controllers/flags.h"

namespace legs {
class BasicController
{
    public:
        virtual void move(std::vector<double> target, MoveFlags = NONE) = 0;
        virtual void turn(double target, MoveFlags = NONE)              = 0;

    protected:
        BasicController() {}
        std::shared_ptr<BasicChassis> chassis = nullptr;
        std::shared_ptr<BasicModel> model = nullptr;
    
    private:
};
}