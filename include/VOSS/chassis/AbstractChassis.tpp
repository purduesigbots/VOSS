#pragma once
#include "../utils/flags.hpp"
#include "AbstractChassis.hpp"

#include <cmath>
#include <memory>

namespace voss::chassis {

AbstractChassis::AbstractChassis(
    std::shared_ptr<controller::PIDController> default_controller, ec_ptr ec) {
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

void AbstractChassis::move(double target, double max, voss::Flags flags) {
    this->move({target, 0}, this->default_controller, this->default_ec, max,
               flags | voss::Flags::RELATIVE);
}

void AbstractChassis::move(double target, ec_ptr ec, double max,
                           voss::Flags flags) {
    this->move({target, 0}, this->default_controller, std::move(ec), max,
               flags | voss::Flags::RELATIVE);
}

template <Is_Controller T>
void AbstractChassis::move(double target, std::shared_ptr<T> controller,
                           double max, voss::Flags flags) {
    static_assert(voss::controller::Is_linear_motion<T>(),
                  "\nThe controller passed in cannot move!");
    this->move({target, 0}, std::move(controller), this->default_ec, max,
               flags | voss::Flags::RELATIVE);
}

template <Is_Controller T>
void AbstractChassis::move(double target, std::shared_ptr<T> controller,
                           ec_ptr ec, double max, voss::Flags flags) {
    static_assert(voss::controller::Is_linear_motion<T>(),
                  "\nThe controller passed in cannot move!");
    this->move({target, 0}, std::move(controller), std::move(ec), max,
               flags | voss::Flags::RELATIVE);
}

void AbstractChassis::move(Pose target, double max, voss::Flags flags) {
    this->move(target, this->default_controller, this->default_ec, max, flags);
}

void AbstractChassis::move(Pose target, ec_ptr ec, double max,
                           voss::Flags flags) {
    this->move(target, this->default_controller, std::move(ec), max, flags);
}

template <Is_Controller T>
void AbstractChassis::move(Pose target, std::shared_ptr<T> controller,
                           double max, Flags flags) {
    static_assert(voss::controller::Is_linear_motion<T>(),
                  "\nThe controller passed in cannot move!");
    this->move(target, std::move(controller), this->default_ec, max, flags);
}

template <Is_Controller T>
void AbstractChassis::move(Pose target, std::shared_ptr<T> controller,
                           ec_ptr ec, double max, voss::Flags flags) {
    static_assert(voss::controller::Is_linear_motion<T>(),
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

void AbstractChassis::turn(double target, ec_ptr ec, double max,
                           voss::Flags flags,
                           voss::AngularDirection direction) {
    this->turn(target, this->default_controller, std::move(ec), max, flags,
               direction);
}

template <Is_Controller T>
void AbstractChassis::turn(double target, std::shared_ptr<T> controller,
                           double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    static_assert(voss::controller::Is_angular_motion<T>(),
                  "\nThe controller passed in cannot turn!");
    this->turn(target, std::move(controller), this->default_ec, max, flags,
               direction);
}

template <Is_Controller T>
void AbstractChassis::turn(double target, std::shared_ptr<T> controller,
                           ec_ptr ec, double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    static_assert(voss::controller::Is_angular_motion<T>(),
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

void AbstractChassis::turn_to(Point target, ec_ptr ec, double max,
                              voss::Flags flags,
                              voss::AngularDirection direction) {
    this->turn_to(target, this->default_controller, std::move(ec), max, flags,
                  direction);
}

template <Is_Controller T>
void AbstractChassis::turn_to(Point target, std::shared_ptr<T> controller,
                              double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    static_assert(voss::controller::Is_angular_motion<T>(),
                  "\nThe controller passed in cannot turn!");
    this->turn_to(target, std::move(controller), this->default_ec, max, flags,
                  direction);
}

template <Is_Controller T>
void AbstractChassis::turn_to(Point target, std::shared_ptr<T> controller,
                              ec_ptr ec, double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    static_assert(voss::controller::Is_angular_motion<T>(),
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

template <Is_Controller T>
void AbstractChassis::follow(std::initializer_list<Pose> target_path,
                             std::shared_ptr<T> controller, double max,
                             voss::Flags flags) {
    static_assert(voss::controller::Is_path_follow_motion<T>(),
                  "\nThe controller passed in cannot follow path!");
    this->follow(target_path, std::move(controller), this->default_ec, max,
                 flags);
}

template <Is_Controller T>
void AbstractChassis::follow(std::initializer_list<Pose> target_path,
                             std::shared_ptr<T> controller, ec_ptr ec,
                             double max, voss::Flags flags) {
    static_assert(voss::controller::Is_path_follow_motion<T>(),
                  "\nThe controller passed in cannot follow path!");

    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;
    // set target
    (std::shared_ptr<controller::AbstractController>(controller))
        ->set_target_path(target_path, voss::Flags::RELATIVE & flags);
    controller->set_target(*target_path.end(), flags & voss::Flags::RELATIVE,
                           ec);

    this->move_task(std::move(controller), std::move(ec), max, flags);
}

template <Is_Controller T>
void AbstractChassis::follow(const asset& target_file,
                             std::shared_ptr<T> controller, double max,
                             voss::Flags flags) {
    static_assert(voss::controller::Is_path_follow_motion<T>(),
                  "\nThe controller passed in cannot follow path!");
    this->follow(target_file, std::move(controller), this->default_ec, max, flags);
}

template <Is_Controller T>
void AbstractChassis::follow(const asset& target_file,
                             std::shared_ptr<T> controller, ec_ptr ec,
                             double max, voss::Flags flags) {
    static_assert(voss::controller::Is_path_follow_motion<T>(),
                  "\nThe controller passed in cannot follow path!");

    /**
     * Decode .json or .txt file
     */
    auto target_path = voss::utils::decode_traj_txt(target_file);

    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;
    // set target
    (std::shared_ptr<controller::AbstractController>(controller))
        ->set_target_path(target_path, voss::Flags::RELATIVE & flags);
    (std::shared_ptr<controller::AbstractController>(controller))
        ->set_target(*target_path.end(), flags & voss::Flags::RELATIVE, ec);

    this->move_task(std::move(controller), std::move(ec), max, flags);
}

} // namespace voss::chassis