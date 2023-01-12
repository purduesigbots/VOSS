#pragma once

#include "api.h"
#include "legs/motion_controllers/basic_controller.hpp"

namespace legs {

class BoomerangControllerBuilder;
class BoomerangController;

class BoomerangController : public BasicController
{
    void move(Eigen::Vector3d& target);
    void turn(double target);
};

class BoomerangControllerBuilder
{

};

}