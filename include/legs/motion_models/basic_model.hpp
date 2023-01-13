#pragma once

#include "Eigen/Core"

class BasicModel
{
public:
    virtual Eigen::Vector3d getPose() {}
    virtual Eigen::Vector2d getPosition() {}
    virtual double          getHeading() {}
protected:
    BasicModel() {}
};