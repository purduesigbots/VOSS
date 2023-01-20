#include "legs/motion_models/odometry.hpp"
#include "api.h"

namespace legs {

Eigen::Vector2d OdometryModel::getPosition() {
    Eigen::Vector2d position(this->pose.x(), this->pose.y());
    return position;
}

Eigen::Vector3d OdometryModel::getPose() {
    return this->pose;
}

double OdometryModel::getHeading() {
    return this->pose.z();
}

int OdometryModel::odomTask() {
    this->pose = Eigen::Vector3d(0.0, 0.0, 0.0);

    while(true) {
        bool hasMiddleTracker = this->middleTracker != nullptr;

		double left_pos = this->leftTracker->get_dist_traveled();
		double right_pos = this->rightTracker->get_dist_traveled();
		double middle_pos = hasMiddleTracker ? this->middleTracker->get_dist_traveled() : 0;

		// calculate change in each encoder
		double delta_left = (left_pos - prev_left_pos) / this->tpi;
		double delta_right = (right_pos - prev_right_pos) / this->tpi;
		double delta_middle = hasMiddleTracker
		                          ? (middle_pos - prev_middle_pos) / this->middle_tpi
		                          : 0;

		// calculate new heading
		double delta_angle;
		if (this->imu != nullptr) {
			this->pose(2) = -this->imu->get_rotation() * M_PI / 180.0;
			delta_angle = this->pose.z() - prev_heading;
		} else {
			delta_angle = (delta_right - delta_left) / this->track_width;

			this->pose(2) += delta_angle;
		}

		// store previous positions
		prev_left_pos = left_pos;
		prev_right_pos = right_pos;
		prev_middle_pos = middle_pos;
		prev_heading = this->pose.z();

		// calculate local displacement
		double local_x;
		double local_y;

		if (delta_angle) {
			double i = sin(delta_angle / 2.0) * 2.0;
			local_x = (delta_right / delta_angle - left_right_distance) * i;
			local_y = (delta_middle / delta_angle + middle_distance) * i;
		} else {
			local_x = delta_right;
			local_y = delta_middle;
		}

		double p = this->pose.z() - delta_angle / 2.0; // global angle

		// convert to absolute displacement
		this->pose(0) += cos(p) * local_x - sin(p) * local_y;
		this->pose(1) += cos(p) * local_y + sin(p) * local_x;

		if (this->debug)
			printf("%.2f, %.2f, %.2f \n", this->pose.x(), this->pose.y(), this->pose.z());

		pros::delay(10);
    }
}

void OdometryModel::begin() {
    this->task = pros::Task([=](){
        this->odomTask();
    });
}

OdometryModelBuilder::OdometryModelBuilder() {
}

OdometryModelBuilder& OdometryModelBuilder::withImu(int port) {
    this->odom.imu = std::make_shared<pros::Imu>(port);
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withDebug(bool debug) {
    this->odom.debug = debug;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withExpander(int port) {
    this->odom.expanderPort = port;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withLeftEncoder(int port) {
    if(this->odom.tpi == 0) {
        //throw error
    }
    if(this->odom.expanderPort == 0) {
        this->odom.leftTracker = std::make_shared<LegsDistanceTracker>(port, LEGS_ADI_ENCODER, port < 0, this->odom.tpi);
    } else {
        this->odom.leftTracker = std::make_shared<LegsDistanceTracker>(port, this->odom.expanderPort, port < 0, this->odom.tpi);
    }
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withRightEncoder(int port) {
    if(this->odom.tpi == 0) {
        //throw error
    }
    if(this->odom.expanderPort == 0) {
        this->odom.rightTracker = std::make_shared<LegsDistanceTracker>(port, LEGS_ADI_ENCODER, port < 0, this->odom.tpi);
    } else {
        this->odom.rightTracker = std::make_shared<LegsDistanceTracker>(port, this->odom.expanderPort, port < 0, this->odom.tpi);
    }
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleEncoder(int port) {
    if(this->odom.tpi == 0) {
        //throw error
    }
    if(this->odom.expanderPort == 0) {
        this->odom.middleTracker = std::make_shared<LegsDistanceTracker>(port, LEGS_ADI_ENCODER, port < 0, this->odom.tpi);
    } else {
        this->odom.middleTracker = std::make_shared<LegsDistanceTracker>(port, this->odom.expanderPort, port < 0, this->odom.tpi);
    }
    this->hasMiddleEncoder = true;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withLeftRotation(int port) {
    if(this->odom.tpi == 0) {
        //throw error
    }
    this->odom.leftTracker = std::make_shared<legs::LegsDistanceTracker>(port, LEGS_ROTATION, port < 0, this->odom.tpi);
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withRightRotation(int port) {
    if(this->odom.tpi == 0) {
        //throw error
    }
    this->odom.rightTracker = std::make_shared<legs::LegsDistanceTracker>(port, LEGS_ROTATION, port < 0, this->odom.tpi);
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleRotation(int port) {
    if(this->odom.tpi == 0) {
        //throw error
    }
    this->odom.middleTracker = std::make_shared<legs::LegsDistanceTracker>(port, LEGS_ROTATION, port < 0, this->odom.tpi);
    this->hasMiddleEncoder = true;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withTrackWidth(double track_width) {
    this->odom.track_width = track_width;
    this->odom.left_right_distance = 0.5 * track_width;
    this->hasTrackWidthOrLeftRightDistance = true;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withLeftRightDistance(double left_right_distance) {
    this->odom.left_right_distance = left_right_distance;
    this->odom.track_width = 2 * left_right_distance;
    this->hasTrackWidthOrLeftRightDistance = true;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleDistance(double middle_distance) {
    this->odom.middle_distance = middle_distance;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withLeftRightTPI(double tpi) {
    this->odom.tpi = tpi;
    this->hasTPI = true;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleTPI(double middle_tpi) {
    this->odom.middle_tpi = middle_tpi;
    this->hasMiddleTPI = true;
    return *this;
}

OdometryModel OdometryModelBuilder::build() {
    if(this->hasTrackWidthOrLeftRightDistance && 
       this->hasTPI &&
       (
        this->hasIMU || 
        (this->hasMiddleEncoder && this->hasMiddleTPI)
       )
       ) {
        this->odom.begin();
        return this->odom;
    } else {
        // throw error
    }
}

}