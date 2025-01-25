#pragma once

#include "SSOV/routines/Routine.hpp"
#include "SSOV/controller/PointController.hpp"
#include "SSOV/exit_condition/ExitCondition.hpp"
#include "SSOV/localizer/Localizer.hpp"
#include "SSOV/common/Math.hpp"
#include <memory>

namespace ssov {

class MoveToPoint: public Routine {
    public:
    struct Params {
        std::shared_ptr<PointController> controller;
        std::shared_ptr<ExitCondition> exit;
        std::shared_ptr<Localizer> localizer;
        double slew;
        bool reverse;
        bool thru;
    };
    private:
        std::shared_ptr<PointController> controller;
        std::shared_ptr<ExitCondition> exit;
        std::shared_ptr<Localizer> localizer;
        Point target;
        DriveSignal prev_speeds;
        double slew;
        bool reverse;
        bool thru;
        bool done;
    public:
        MoveToPoint(Point target, MoveToPoint::Params params, DriveSignal initial_speeds):
            target(target),
            controller(params.controller),
            exit(params.exit),
            localizer(params.localizer),
            slew(params.slew),
            reverse(params.reverse),
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
            done = exit->is_met(localizer->get_pose(), {target.x, target.y, localizer->get_pose().theta}, thru);
            if (done && !thru) {
                result = {0, 0, 0};
            } else {
                result = controller->compute(localizer->get_pose(), target, reverse, thru);
                result.x = ssov::slew(result.x, prev_speeds.x, slew);
                result.y = ssov::slew(result.y, prev_speeds.y, slew);
                result.theta = ssov::slew(result.theta, prev_speeds.theta, slew);
                prev_speeds = result;
            }
            return result;
        }
};

}