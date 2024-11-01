#include "SSOV/Defaults.hpp"

namespace ssov::defaults {
    std::shared_ptr<DiffChassis> chassis = nullptr;
    std::shared_ptr<PointController> point_controller = nullptr;
    std::shared_ptr<PoseController> pose_controller = nullptr;
    std::shared_ptr<ExitCondition> exit_condition = nullptr;
    std::shared_ptr<Localizer> localizer = nullptr;
}