#pragma once

#include <variant>

namespace legs::chassis {

struct Stop {};
struct Voltages {
	double left;
	double right;
};

using ChassisCommand = std::variant<Stop, Voltages>;

template <class... Ts> struct overload : Ts... {
	using Ts::operator()...;
};
template <class... Ts> overload(Ts...) -> overload<Ts...>;

} // namespace legs::chassis