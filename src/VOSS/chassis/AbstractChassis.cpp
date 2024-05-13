#include "VOSS/chassis/AbstractChassis.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "VOSS/constants.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/utils/angle.hpp"

#include <cmath>

namespace voss::chassis {

AbstractChassis::AbstractChassis(
    std::shared_ptr<voss::controller::PIDController> default_controller,
    std::shared_ptr<voss::localizer::AbstractLocalizer> l, ec_ptr ec) {
    this->l = std::move(l);
    this->default_controller = std::move(default_controller);
    this->default_ec = std::move(ec);
}

Pose AbstractChassis::process_target_pose(Pose target, bool relative) {
    if (target.theta.has_value()) {
        target.theta = voss::to_radians(target.theta.value());
    }
    if (relative) {
        return Pose::get_relative(target, this->l->get_pose());
    }
    return target;
}

double AbstractChassis::process_target_angle(double angle, bool relative) {
    angle = voss::to_radians(angle);
    if (relative) {
        return voss::norm(angle + this->l->get_orientation_rad());
    }
    return voss::norm(angle);
}

void AbstractChassis::move_task(move_controller_ptr controller, ec_ptr ec,
                                double max, voss::Flags flags) {

    this->task = std::make_unique<pros::Task>([&, this, flags, max]() {
        ec->reset();
        controller->reset();
        while (!this->execute(
            controller->get_command(this->l->get_pose(),
                                    flags & voss::Flags::REVERSE,
                                    flags & voss::Flags::THRU, ec),
            max)) {
            if (pros::competition::is_disabled()) {
                this->task_running = false;
                return;
            }

            pros::delay(constants::MOTOR_UPDATE_DELAY);
        }
        this->task_running = false;
    });

    // Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }

    this->task->join();
}

void AbstractChassis::turn_task(turn_controller_ptr controller, ec_ptr ec,
                                double max, voss::Flags flags,
                                voss::AngularDirection direction) {

    this->task =
        std::make_unique<pros::Task>([&, this, flags, direction, max]() {
            ec->reset();
            controller->reset();
            while (!this->execute(controller->get_angular_command(
                                      this->l->get_pose(),
                                      flags & voss::Flags::REVERSE,
                                      flags & voss::Flags::THRU, direction, ec),
                                  max)) {
                if (pros::competition::is_disabled()) {
                    this->task_running = false;
                    return;
                }

                pros::delay(constants::MOTOR_UPDATE_DELAY);
            }
            this->task_running = false;
        });

    // Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }
    this->task->join();
}

void AbstractChassis::move(double distance, double max, voss::Flags flags) {
    this->move({distance, 0}, this->default_controller, this->default_ec, max,
               flags | voss::Flags::RELATIVE);
}

void AbstractChassis::move(double distance, move_controller_ptr controller,
                           double max, voss::Flags flags) {

    this->move({distance, 0}, std::move(controller), this->default_ec, max,
               flags | voss::Flags::RELATIVE);
}

void AbstractChassis::move(double distance, move_controller_ptr controller,
                           ec_ptr ec, double max, voss::Flags flags) {

    this->move({distance, 0}, std::move(controller), std::move(ec), max,
               flags | Flags::RELATIVE);
}

void AbstractChassis::move(Pose target, double max, voss::Flags flags) {
    this->move(target, this->default_controller, this->default_ec, max, flags);
}

void AbstractChassis::move(Pose target, move_controller_ptr controller,
                           double max, voss::Flags flags) {

    this->move(target, std::move(controller), this->default_ec, max, flags);
}

void AbstractChassis::move(Pose target, move_controller_ptr controller,
                           ec_ptr ec, double max, voss::Flags flags) {

    while (this->task_running) {
        pros::delay(constants::MOTOR_UPDATE_DELAY);
    }
    this->task_running = true;

    Pose processed_target =
        this->process_target_pose(target, flags & voss::Flags::RELATIVE);

    controller->set_target(processed_target);
    ec->set_target(processed_target);

    this->move_task(std::move(controller), std::move(ec), max, flags);
}

void AbstractChassis::turn(double target, double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    this->turn(target, this->default_controller, this->default_ec, max, flags,
               direction);
}

void AbstractChassis::turn(double target, turn_controller_ptr controller,
                           double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    this->turn(target, std::move(controller), this->default_ec, max, flags,
               direction);
}

void AbstractChassis::turn(double target, turn_controller_ptr controller,
                           ec_ptr ec, double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    while (this->task_running) {
        pros::delay(constants::MOTOR_UPDATE_DELAY);
    }
    this->task_running = true;

    double processed_target =
        this->process_target_angle(target, flags & voss::Flags::RELATIVE);

    controller->set_target({NAN, NAN, processed_target});
    controller->set_angular_target(processed_target);
    ec->set_target({NAN, NAN, processed_target});

    this->turn_task(std::move(controller), std::move(ec), max, flags,
                    direction);
}

void AbstractChassis::turn_to(Point target, double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    this->turn_to(target, this->default_controller, this->default_ec, max,
                  flags, direction);
}

void AbstractChassis::turn_to(Point target, turn_controller_ptr controller,
                              double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    this->turn_to(target, std::move(controller), this->default_ec, max, flags,
                  direction);
}

void AbstractChassis::turn_to(Point target, turn_controller_ptr controller,
                              ec_ptr ec, double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    Pose processed_target = process_target_pose(
        {target.x, target.y, std::nullopt}, flags & voss::Flags::RELATIVE);

    controller->set_target(processed_target);
    ec->set_target(processed_target);

    this->turn_task(std::move(controller), std::move(ec), max, flags,
                    direction);
}

} // namespace voss::chassis