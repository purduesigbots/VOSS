#pragma once
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "AbstractChassis.hpp"

#include <cmath>

namespace voss::chassis {

AbstractChassis::AbstractChassis(controller_ptr default_controller, ec_ptr ec) {
    this->default_controller = std::move(default_controller);
    this->default_ec = std::move(ec);
}

void AbstractChassis::move_task(controller_ptr controller, ec_ptr ec,
                                double max, voss::Flags flags) {

    this->task =
        std::make_unique<pros::Task>([&, controller, ec, flags, max]() {
            ec->reset();
            controller->reset();
            while (!this->execute(
                controller->get_command(flags & voss::Flags::REVERSE,
                                        flags & voss::Flags::THRU, ec),
                max)) {
                if (pros::competition::is_disabled()) {
                    this->task_running = false;
                    return;
                }

                pros::delay(10);
            }
            this->task_running = false;
        });

    // Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }

    this->task->join();
}

void AbstractChassis::turn_task(controller_ptr controller, ec_ptr ec,
                                double max, voss::Flags flags,
                                voss::AngularDirection direction) {

    this->task = std::make_unique<pros::Task>(
        [&, controller, ec, flags, direction, max]() {
            ec->reset();
            controller->reset();
            while (!this->execute(controller->get_angular_command(
                                      flags & voss::Flags::REVERSE,
                                      flags & voss::Flags::THRU, direction, ec),
                                  max)) {
                if (pros::competition::is_disabled()) {
                    this->task_running = false;
                    return;
                }

                pros::delay(10);
            }
            this->task_running = false;
        });

    // Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }
    this->task->join();
}

void AbstractChassis::move(Pose target, double max, voss::Flags flags) {
    this->move(target, this->default_controller, this->default_ec, max, flags);
}

template <Is_Controller T>
void AbstractChassis::move(Pose target, std::shared_ptr<T> controller,
                           double max, Flags flags) {
    static_assert(voss::controller::can_move<T>(),
                  "\nThe controller passed in cannot move!");
    this->move(target, std::move(controller), this->default_ec, max, flags);
}

template <Is_Controller T>
void AbstractChassis::move(Pose target, std::shared_ptr<T> controller,
                           ec_ptr ec, double max, voss::Flags flags) {
    static_assert(voss::controller::can_move<T>(),
                  "\nThe controller passed in cannot move!");
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;
    controller->set_target(target, flags & voss::Flags::RELATIVE, ec);

    this->move_task(std::move(controller), std::move(ec), max, flags);
}

void AbstractChassis::turn(double target, double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    this->turn(target, this->default_controller, this->default_ec, max, flags,
               direction);
}

template <Is_Controller T>
void AbstractChassis::turn(double target, std::shared_ptr<T> controller,
                           double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    static_assert(voss::controller::can_turn<T>(),
                  "\nThe controller passed in cannot turn!");
    this->turn(target, std::move(controller), this->default_ec, max, flags,
               direction);
}

template <Is_Controller T>
void AbstractChassis::turn(double target, std::shared_ptr<T> controller,
                           ec_ptr ec, double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    static_assert(voss::controller::can_turn<T>(),
                  "\nThe controller passed in cannot turn!");
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    controller->set_target({NAN, NAN, target}, flags & voss::Flags::RELATIVE,
                           ec);
    controller->set_angular_target(target, flags & voss::Flags::RELATIVE);

    this->turn_task(std::move(controller), std::move(ec), max, flags,
                    direction);
}

void AbstractChassis::turn_to(Point target, double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    this->turn_to(target, this->default_controller, this->default_ec, max,
                  flags, direction);
}

template <Is_Controller T>
void AbstractChassis::turn_to(Point target, std::shared_ptr<T> controller,
                              double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    static_assert(voss::controller::can_turn<T>(),
                  "\nThe controller passed in cannot turn!");
    this->turn_to(target, std::move(controller), this->default_ec, max, flags,
                  direction);
}

template <Is_Controller T>
void AbstractChassis::turn_to(Point target, std::shared_ptr<T> controller,
                              ec_ptr ec, double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    static_assert(voss::controller::can_turn<T>(),
                  "\nThe controller passed in cannot turn!");
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    controller->set_target({target.x, target.y, std::nullopt},
                           flags & voss::Flags::RELATIVE, ec);

    this->turn_task(std::move(controller), std::move(ec), max, flags,
                    direction);
}

} // namespace voss::chassis