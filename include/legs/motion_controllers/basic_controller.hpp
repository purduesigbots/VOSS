#pragma once

#include "Eigen/Core"

class BasicController
{
    public:
        virtual void move(Eigen::Vector3d& target) {}
        virtual void turn(double target) {}

    protected:
        BasicController() {}
    
    private:
};