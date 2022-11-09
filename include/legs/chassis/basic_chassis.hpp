#pragma once

#include <Eigen/Core>

class BasicChassis
{
public:

    virtual void forwardVelocity(double velocity) = 0;
    virtual void angularVelocity(double velocity) = 0;

protected:
    // We want the constructor to be protected so that no objects are made
    // directly from this class
    BasicChassis() {}

private:
    /* 
    At the moment, there's nothing about the basic chassis that needs to be
    private since it is just an interface.
    */
};
