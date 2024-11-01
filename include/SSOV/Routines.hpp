#pragma once

#include "SSOV/Defaults.hpp"

namespace ssov {
struct PointMoveParams {
    const std::shared_ptr<PointController> controller = defaults::point_controller;
    const std::shared_ptr<ExitCondition> exit_condition = defaults::exit_condition;
    const std::shared_ptr<Localizer> localizer = defaults::localizer;
    const std::shared_ptr<DiffChassis> chassis = defaults::chassis;
    const double slew = defaults::slew;
    const bool reverse = false;
    const bool thru = false;
};
void move(Point target, const PointMoveParams &params = {});
}