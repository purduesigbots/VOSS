#pragma once

#include "SSOV/routines/Routine.hpp"
#include "SSOV/controller/TurnController.hpp"
#include "SSOV/exit_condition/ExitCondition.hpp"
#include "SSOV/localizer/Localizer.hpp"
#include "SSOV/common/Math.hpp"
#include <memory>

namespace ssov {

class TurnToHeading: public Routine {
    public:
    struct Params {
        std::shared_ptr<TurnController> controller;
        std::shared_ptr<ExitCondition> exit;
        std::shared_ptr<Localizer> localizer;
        double slew;
        TurnDirection direction;
        bool thru;
    };
    private:
        std::shared_ptr<TurnController> controller;
        std::shared_ptr<ExitCondition> exit;
        std::shared_ptr<Localizer> localizer;
        double target;
        DriveSignal prev_speeds;
        double slew;
        TurnDirection direction;
        bool thru;
        bool done = false;
    public:
        TurnToHeading(double target, Params params, DriveSignal initial_speeds):
            target(target),
            controller(params.controller),
            exit(params.exit),
            localizer(params.localizer),
            slew(params.slew),
            direction(params.direction),
            thru(params.thru),
            prev_speeds(initial_speeds) {};
        void start() override {
            controller->reset();
            exit->reset();
        }
        bool finished() override {
            return done;
        }
        ChassisCommand update() {
            DriveSignal result;
            Pose current_pose = localizer->get_pose();
            done = exit->is_met(current_pose, {current_pose.x, current_pose.y, target}, thru);
            if (done && !thru) {
                result = {0, 0, 0};
            } else {
                double turn_speed = controller->compute(current_pose.theta, target, direction, thru);
                result = {0, 0, turn_speed};
                result.x = ssov::slew(result.x, prev_speeds.x, slew);
                result.y = ssov::slew(result.y, prev_speeds.y, slew);
                result.theta = ssov::slew(result.theta, prev_speeds.theta, slew);
                prev_speeds = result;
            }
            return result;
        }
};

}