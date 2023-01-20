#pragma once

#include "api.h"
#include "legs/api.hpp"
#include "legs/pid.hpp"

namespace legs {

class BoomerangControllerBuilder;
class BoomerangController;

class BoomerangController : public BasicController
{
    #define DISABLE 0
    #define TRANSLATIONAL 1
    #define ANGULAR 2

    public:
        void move(std::vector<double> target, double lead_pct, double max, double exit_error,
                  MoveFlags = NONE);
        void move(std::vector<double> target, MoveFlags = NONE);
        void turn(double target);
        void waitUntilFinished();
        bool settled();
    
    protected:
        BoomerangController();
    
    private:
        int mode = DISABLE;
        Pid linear_pid;
        Pid angular_pid;
        Eigen::Vector2d linear_target;
        double angular_target;
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
        BoomerangControllerBuilder& withChassis(BasicChassis& chassis);
        BoomerangControllerBuilder& withModel(BasicModel& model);
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