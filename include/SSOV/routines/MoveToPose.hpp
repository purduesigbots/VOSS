#pragma once

#include "SSOV/routines/Routine.hpp"
#include "SSOV/controller/PoseController.hpp"
#include "SSOV/exit_condition/ExitCondition.hpp"
#include "SSOV/localizer/Localizer.hpp"
#include "SSOV/common/Math.hpp"
#include <memory>

namespace ssov {

class MoveToPose: public Routine {
    public:
        struct Params {
            std::shared_ptr<PoseController> controller;
            std::shared_ptr<ExitCondition> ec;
            std::shared_ptr<Localizer> localizer;
            double slew;
            bool reverse;
            bool thru;
            bool holonomic;
            int strafe_angle;
        };
    private:
        std::shared_ptr<PoseController> controller;
        std::shared_ptr<ExitCondition> exit;
        std::shared_ptr<Localizer> localizer;
        Pose target;
        DriveSignal prev_speeds;
        double slew;
        bool reverse;
        bool thru;
        bool done = false;
        bool holonomic = false;
        int strage_angle = 0;
    public:
        MoveToPose(Pose target, Params params, DriveSignal initial_speeds):
            target(target),
            controller(params.controller),
            exit(params.ec),
            localizer(params.localizer),
            slew(params.thru),
            reverse(params.reverse),
            thru(params.thru),
            holonomic(params.holonomic),
            strage_angle(params.strafe_angle),
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
            done = exit->is_met(localizer->get_pose(), target, thru);
            if (done && !thru) {
                result = {0, 0, 0};
            } else {
                result = controller->compute(localizer->get_pose(), target, reverse, thru, holonomic, strage_angle);
                result.x = ssov::slew(result.x, prev_speeds.x, slew);
                result.y = ssov::slew(result.y, prev_speeds.y, slew);
                result.theta = ssov::slew(result.theta, prev_speeds.theta, slew);
                prev_speeds = result;
            }
            return result;
        }
};

}