#pragma once

#include "Eigen/Core"

class BasicChassis
{
public:
    virtual void setCartesianVelocity(Eigen::Vector2d& velocity) {}

    virtual void setForwardVelocity(double velocity) {}
    virtual void setHorizontalVelocity(double velocity) {}
    virtual void setAngularVelocity(double velocity) {}

protected:
    // We want the constructor to be protected so that no objects are made
    // directly from this class, only by being inherited and implemented
    BasicChassis() {}

private:
    /* 
    At the moment, there's nothing about the basic chassis that needs to be
    private since it is just an interface.
    */
};
