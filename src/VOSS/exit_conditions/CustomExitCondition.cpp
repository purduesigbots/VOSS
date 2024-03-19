#include "VOSS/exit_conditions/CustomExitCondition.hpp"

#include <utility>

namespace voss::controller {
CustomExitCondition::CustomExitCondition(std::function<bool()> callback)
    : callback(std::move(callback)){};

bool CustomExitCondition::is_met(voss::Pose current_pose, bool thru) {
    return this->callback();
}

} // namespace voss::controller