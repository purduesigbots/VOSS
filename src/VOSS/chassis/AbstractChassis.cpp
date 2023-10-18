#include "voss/chassis/AbstractChassis.hpp"
#include "pros/rtos.hpp"

namespace voss::chassis {

    AbstractChassis::AbstractChassis(
            controller::AbstractController &default_controller) {
        this->default_controller = &default_controller;
    }

    void AbstractChassis::move_task(controller::AbstractController* controller,
                                    double max, uint8_t flags) {

        pros::Task running_t([&]() {
            controller->reset();
            while (!this->execute(
                    controller->get_command(flags & voss::REVERSE, flags & voss::THRU),
                    max)) {
                pros::delay(10);
            }
            this->m.give();
        });

        if (flags & voss::ASYNC) {
            return;
        }

        this->m.take();
        this->m.give();
    }

    void AbstractChassis::turn_task(controller::AbstractController* controller,
                                    double max, uint8_t flags) {

        pros::Task running_t([&]() {
            controller->reset();
            while (!this->execute(
                    controller->get_angular_command(flags & voss::REVERSE, flags & voss::THRU),
                    max)) {
                pros::delay(10);
            }
            this->m.give();
        });

        if (flags & voss::ASYNC) {
            return;
        }

        this->m.take();
        this->m.give();
    }


    void AbstractChassis::move(Point target, double max, uint8_t flags) {
        this->move(target, this->default_controller, flags);
    }

    void AbstractChassis::move(Pose target, double max, uint8_t flags) {
        this->move(target, this->default_controller, flags);
    }

    void AbstractChassis::move(Point target,
                               controller::AbstractController *controller,
                               double max, uint8_t flags) {

        Pose pose_target = Pose{target.x, target.y, 361};
        this->move(pose_target, controller, flags);
    }

    void AbstractChassis::move(Pose target,
                               controller::AbstractController *controller,
                               double max, uint8_t flags) {

        this->m.take();

        controller->set_target(target, flags & voss::RELATIVE);

        this->move_task(controller, max, flags);
    }

    void AbstractChassis::turn(double target, double max, uint8_t flags) {
        this->turn(target, this->default_controller, max, flags);
    }

    void AbstractChassis::turn(double target, controller::AbstractController *controller, double max, uint8_t flags) {
        this->m.take();

        controller->set_angular_target(target, flags & voss::RELATIVE);

        this->turn_task(controller, max, flags);
    }

    void AbstractChassis::turnTo(Point target, double max, uint8_t flags) {
        this->turnTo(target, this->default_controller, max, flags);
    }

    void AbstractChassis::turnTo(Point target, controller::AbstractController *controller, double max, uint8_t flags) {
        this->m.take();

        controller->set_target({target.x, target.y, 361}, flags & voss::RELATIVE);

        this->turn_task(controller, max, flags);
    }

} // namespace voss::chassis