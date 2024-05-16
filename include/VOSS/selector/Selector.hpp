/**
 * @file Selector.hpp
 * @brief
 * @version 0.1.2
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

namespace voss::selector {

void init(int default_auton, const char** autons);
int get_auton();

} // namespace voss::selector
