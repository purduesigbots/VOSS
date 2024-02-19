#include "VOSS/chassis/AbstractChassis.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"

namespace voss::chassis {

AbstractChassis::AbstractChassis(controller_ptr default_controller, ec_ptr ec) {
    this->default_controller = default_controller;
    this->default_ec = ec;
}

void AbstractChassis::move_task(controller_ptr controller, ec_ptr ec,
                                double max, voss::Flags flags) {
    int t = 0;
    pros::Task running_t([&, controller]() {
        ec->reset();
        controller->reset();
        // Loops until movement is complete or robot is disabled
        while (!this->execute(
            controller->get_command(flags & voss::Flags::REVERSE,
                                    flags & voss::Flags::THRU, ec),
            max)) {
            if (pros::competition::is_disabled()) {
                return;
            }

            t += 10;
            pros::delay(10);
        }
        // this->m.give();
    });

    // Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }

    running_t.join();

    // this->m.take();
    // this->m.give();
}

void AbstractChassis::turn_task(controller_ptr controller, ec_ptr ec,
                                double max, voss::Flags flags) {
    int t = 0;
    pros::Task running_t([&, controller]() {
        ec->reset();
        controller->reset();
        // Loops until movement is complete or robot is disabled
        while (!this->execute(
            controller->get_angular_command(flags & voss::Flags::REVERSE,
                                            flags & voss::Flags::THRU, ec),
            max)) {
            if (pros::competition::is_disabled()) {
                return;
            }

            t += 10;
            pros::delay(10);
        }
    });

    // Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }

    running_t.join();
}

// Overloaded constructors move functions to allow for different parameters
void AbstractChassis::move(Point target, double max, voss::Flags flags) {
    this->move(target, this->default_controller, max, flags);
}

void AbstractChassis::move(Pose target, double max, voss::Flags flags) {
    this->move(target, this->default_controller, max, flags);
}

void AbstractChassis::move(Point target, controller_ptr controller, double max,
                           voss::Flags flags) {
    Pose pose_target = Pose{target.x, target.y, 361};
    this->move(pose_target, controller, max, flags);
}

void AbstractChassis::move(Point target, controller_ptr controller, ec_ptr ec,
                           double max, voss::Flags flags) {
    Pose pose_target = Pose{target.x, target.y, 361};
    this->move(pose_target, controller, ec, max, flags);
}

void AbstractChassis::move(Pose target, controller_ptr controller, double max,
                           voss::Flags flags) {

    this->move(target, controller, this->default_ec, max, flags);
}

void AbstractChassis::move(Pose target, controller_ptr controller, ec_ptr ec,
                           double max, voss::Flags flags) {
    // this->m.take();

    controller->set_target(target, flags & voss::Flags::RELATIVE, ec);

    this->move_task(controller, ec, max, flags);
}

void AbstractChassis::turn(double target, double max, voss::Flags flags) {
    this->turn(target, this->default_controller, max, flags);
}

void AbstractChassis::turn(double target, controller_ptr controller, double max,
                           voss::Flags flags) {
    this->turn(target, controller, this->default_ec, max, flags);
}

void AbstractChassis::turn(double target, controller_ptr controller, ec_ptr ec,
                           double max, voss::Flags flags) {
    // this->m.take();

    controller->set_target({0, 0, 0}, false, ec);
    controller->set_angular_target(target, flags & voss::Flags::RELATIVE);

    this->turn_task(controller, ec, max, flags);
}

void AbstractChassis::turn_to(Point target, double max, voss::Flags flags) {
    this->turn_to(target, this->default_controller, max, flags);
}

void AbstractChassis::turn_to(Point target, controller_ptr controller,
                              double max, voss::Flags flags) {
    this->turn_to(target, controller, this->default_ec, max, flags);
}

void AbstractChassis::turn_to(Point target, controller_ptr controller,
                              ec_ptr ec, double max, voss::Flags flags) {
    // this->m.take();

    controller->set_target({target.x, target.y, 361},
                           flags & voss::Flags::RELATIVE, ec);

    this->turn_task(controller, ec, max, flags);
}

} // namespace voss::chassis