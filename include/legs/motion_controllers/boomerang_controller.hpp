#pragma once

#include "api.h"
#include "legs/api.hpp"

namespace legs {

class BoomerangControllerBuilder;
class BoomerangController;

class BoomerangController : public BasicController
{
    #define DISABLE 0
    #define TRANSLATIONAL 1
    #define ANGULAR 2

    public:
        void move(Eigen::Vector3d& target);
        void turn(double target);
    
    protected:
        BoomerangController();
    
    private:
        int mode = DISABLE;
        BasicChassis chassis;
        BasicModel model;
        Pid linearPid;
        Pid angularPid;
        double linear_exit_error;
        double angular_exit_error;
        double linear_settle_thresh;
        double angular_settle_thresh;
        double settle_time;
        double max_speed;
        double lead_pct;
        bool reverse;
        bool thru;
        bool can_reverse;

        friend class BoomerangControllerBuilder;
};

class BoomerangControllerBuilder
{
    public:
        BoomerangControllerBuilder& withChassis(BasicChassis chassis);
        BoomerangControllerBuilder& withModel(BasicModel model);
        BoomerangControllerBuilder& withLinearPid(double p, double i, double d);
        BoomerangControllerBuilder& withAngularPid(double p, double i, double d);
        BoomerangControllerBuilder& withExitErrors(double linear, double angular);
        BoomerangControllerBuilder& withSettleThresh(double linear, double angular);
        BoomerangControllerBuilder& withSettleTime(double time);

        BoomerangController build();

        BoomerangControllerBuilder();
    
    private:
        BoomerangController controller;
};

}