#pragma once

#include <variant>

namespace voss::chassis {

struct Stop {};

namespace diff_commands {
struct Voltages {
    double left;
    double right;
};
struct Chained {
    double left;
    double right;
};
struct Swing {
    double left;
    double right;
};
} // namespace diff_commands

namespace holo_commands {
struct Voltages {
    double v_x;
    double v_y;
    double v_theta;
};
} // namespace holo_commands

using DiffChassisCommand =
    std::variant<Stop, diff_commands::Voltages, diff_commands::Chained,
                 diff_commands::Swing>;
using HoloChassisCommand = std::variant<Stop, holo_commands::Voltages>;

template <class... Ts> struct overload : Ts... { using Ts::operator()...; };
template <class... Ts> overload(Ts...) -> overload<Ts...>;

} // namespace voss::chassis