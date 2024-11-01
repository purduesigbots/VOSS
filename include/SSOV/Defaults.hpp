#pragma once

#include <memory>

#include "SSOV/chassis/DiffChassis.hpp"
#include "SSOV/controller/PointController.hpp"
#include "SSOV/controller/PoseController.hpp"
#include "SSOV/exit_condition/ExitCondition.hpp"
#include "SSOV/localizer/Localizer.hpp"

namespace ssov::defaults {
    // default pointers
    extern std::shared_ptr<DiffChassis> chassis;
    extern std::shared_ptr<PointController> point_controller;
    extern std::shared_ptr<PoseController> pose_controller;
    extern std::shared_ptr<ExitCondition> exit_condition;
    extern std::shared_ptr<Localizer> localizer;

    // default constants
    const double slew = 8;
}