#pragma once

#include "Eigen/Core"

class BasicModel
{
public:
    virtual Eigen::Vector3d& getPose()       = 0;
    virtual Eigen::Vector2d& getPosition()   = 0;
    virtual double           getHeading()    = 0;

};