#pragma once

#include <variant>

namespace voss::chassis {

struct Stop {};
struct Voltages {
	double linear;
	double angular;
};

using ChassisCommand = std::variant<Stop, Voltages>;

template <class... Ts> struct overload : Ts... {
	using Ts::operator()...;
};
template <class... Ts> overload(Ts...) -> overload<Ts...>;

} // namespace voss::chassis