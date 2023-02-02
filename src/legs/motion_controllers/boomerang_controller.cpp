#include "legs/motion_controllers/boomerang_controller.hpp"

#include "Eigen/Geometry"
#include "api.h"

namespace legs {

void BoomerangController::move(std::vector<double> target, double lead,
                               double max, double exit_error, MoveFlags flags) {
	mode = TRANSLATIONAL;
	linear_target[0] = target[0];
	linear_target[1] = target[1];
	if (target.size() == 3) {
		angular_target = (fmod(target.at(2) * (M_PI / 180), M_PI * 2));
	} else {
		angular_target = 6.9; // sentinel value
	}

	if (flags & RELATIVE) {
		Eigen::Rotation2Dd rotation_matrix(model->getHeading());
		linear_target = model->getPosition() + rotation_matrix * linear_target;
		if (target.size() == 3)
			angular_target += fmod(model->getHeading(), M_PI * 2);
	}

	lead_pct = lead;
	max_speed = max;
	BoomerangController::exit_error = exit_error;
	thru = flags & THRU;
	reverse = flags & REVERSE;
	can_reverse = false;

	if (!(flags & ASYNC)) {
		waitUntilFinished();
		mode = DISABLE;
		if (!(flags & THRU)) {
			chassis->setAngularVelocity(0);
			chassis->setForwardVelocity(0);
			chassis->setHorizontalVelocity(0);
		}
	}
}

BoomerangController::BoomerangController() {
}

void BoomerangController::move(std::vector<double> target, MoveFlags flags) {
	move(target, lead_pct, max_speed, linear_exit_error, flags);
}

void BoomerangController::turn(double target, double max, double exit_error,
                               MoveFlags flags) {
	mode = ANGULAR;
	double bounded_heading = fmod(model->getHeading(), M_PI * 2);
	double diff = (target * (M_PI / 180)) - bounded_heading;

	if (diff > M_PI)
		diff -= 2 * M_PI;
	else if (diff < -M_PI)
		diff += 2 * M_PI;

	if (flags & RELATIVE) {
		diff = target;
	}

	angular_target = diff + model->getHeading();
	max_speed = max;

	if (!(flags & ASYNC)) {
		waitUntilFinished();
		mode = DISABLE;
		if (!(flags & THRU)) {
			chassis->setAngularVelocity(0);
			chassis->setForwardVelocity(0);
			chassis->setHorizontalVelocity(0);
		}
	}
}

void BoomerangController::turn(double target, MoveFlags flags) {
	turn(target, 100, angular_exit_error, flags);
}

void BoomerangController::waitUntilFinished() {
	pros::delay(400); // minimum movement time
	switch (mode) {
	case TRANSLATIONAL:
		while ((model->getPosition() - linear_target).norm() > exit_error &&
		       !settled()) {
			pros::delay(10);
		}

		// if doing a pose movement, make sure we are at the target theta
		if (angular_target != 6.9) {
			while (fabs(model->getHeading() - angular_target) > exit_error &&
			       !settled()) {
				pros::delay(10);
			}
		}

		break;
	case ANGULAR:
		while (fabs(model->getHeading() - angular_target) > exit_error &&
		       !settled()) {
			pros::delay(10);
		}
		break;
	}
}

bool BoomerangController::settled() {
	Eigen::Vector2d position = model->getPosition();
	double angle = model->getHeading();

	if ((position - prev_position).norm() > linear_settle_thresh) {
		prev_position = position;
		settle_count = 0;
	} else if (fabs(angle - prev_angle) > angular_settle_thresh) {
		prev_angle = angle;
		settle_count = 0;
	} else {
		settle_count += 10;
	}

	return settle_count > settle_time;
}

std::array<double, 2> BoomerangController::translational() {
	// an angular targer > 2π indicates no desired final pose angle
	bool no_pose = (angular_target > 2 * M_PI);

	double lin_error = (model->getPosition() - linear_target).norm();

	Eigen::Vector2d carrot_point;
	if (no_pose) {
		carrot_point = linear_target;
	} else {
		carrot_point = {
		    linear_target[0] - lin_error * cos(angular_target) * lead_pct,
		    linear_target[1] - lin_error * sin(angular_target) * lead_pct};
	}

	carrot_point -= model->getPosition();
	double angle_error;
	if (reverse) {
		angle_error =
		    atan2(-carrot_point[1], -carrot_point[0]) - model->getHeading();
	} else {
		angle_error = atan2(carrot_point[1], carrot_point[0]) - model->getHeading();
	}

	while (fabs(angle_error) > M_PI) {
		angle_error -= 2 * M_PI * angle_error / fabs(angle_error);
	}

	double lin_speed;
	if (thru)
		lin_speed = max_speed;
	else {
		lin_speed = linear_pid.apply(0, lin_error);
		if (lin_speed > max_speed)
			lin_speed = max_speed;
	}

	if (reverse)
		lin_speed = -lin_speed;

	double ang_speed;
	if (lin_error < exit_error) {
		can_reverse = true;

		if (no_pose) {
			// disable turning when close to the point to prevent spinning
			ang_speed = 0;
		} else {
			// turn to face the final pose angle if executing a pose movement
			double pose_error = angular_target - model->getHeading();
			while (fabs(pose_error) > M_PI)
				pose_error -= 2 * M_PI * pose_error / fabs(pose_error);
			ang_speed = angular_pid.apply(0, pose_error);
		}

		// reduce the linear speed if the bot is tangent to the target
		lin_speed *= cos(angle_error);
	} else {
		// reverse on overshoot
		if (fabs(angle_error) > M_PI_2 && can_reverse) {
			angle_error = angle_error - (angle_error / fabs(angle_error)) * M_PI;
			lin_speed = -lin_speed;
		}

		ang_speed = angular_pid.apply(0, angle_error);
	}

	// overturn
	double overturn = fabs(ang_speed) + fabs(lin_speed) - max_speed;
	if (overturn > 0)
		lin_speed -= lin_speed > 0 ? overturn : -overturn;

	return {lin_speed, ang_speed};
}

double BoomerangController::angular() {
	return angular_pid.apply(angular_target, model->getHeading());
}

void BoomerangController::update() {
	while (true) {
		pros::delay(10);
		if (mode == TRANSLATIONAL) {
			std::array<double, 2> speeds = translational();
			chassis->setForwardVelocity(speeds[0]);
			chassis->setAngularVelocity(speeds[1]);
		} else if (mode == ANGULAR) {
			chassis->setForwardVelocity(0);
			chassis->setAngularVelocity(angular());
		}
	}
}

void BoomerangController::init() {
	pros::Task boomerang_task([=]() { update(); });
}

BoomerangControllerBuilder::BoomerangControllerBuilder() {
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withChassis(BasicChassis& chassis) {
	this->controller.chassis = std::shared_ptr<BasicChassis>(&chassis);
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withModel(BasicModel& model) {
	this->controller.model = std::shared_ptr<BasicModel>(&model);
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withLinearPid(double p, double i, double d) {
	this->controller.linear_pid = Pid(p, i, d);
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withAngularPid(double p, double i, double d) {
	this->controller.angular_pid = Pid(p, i, d);
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withExitErrors(double linear, double angular) {
	this->controller.linear_exit_error = linear;
	this->controller.angular_exit_error = angular;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withSettleThresh(double linear, double angular) {
	this->controller.linear_settle_thresh = linear;
	this->controller.angular_settle_thresh = angular;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withSettleTime(double time) {
	this->controller.settle_time = time;
	return *this;
}

BoomerangController BoomerangControllerBuilder::build() {
	return this->controller;
}

} // namespace legs