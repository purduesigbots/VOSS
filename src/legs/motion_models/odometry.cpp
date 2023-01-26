#include "legs/motion_models/odometry.hpp"
#include "api.h"

namespace legs {

//==============================================================================
//                          ODOMETRY taskEMENTATION
//
// For an explanation on why this class exists, look at the notes/Tasks.txt text
// file. 
//==============================================================================

int OdometryModel::Task::mainLoop() {
    this->pose[0] = 0;
    this->pose[1] = 0;
    this->pose[2] = 0;

    while(true) {
        bool hasMiddleTracker = this->middleTracker != nullptr;

		double left_pos = this->leftTracker->get_dist_traveled();
		double right_pos = this->rightTracker->get_dist_traveled();
		double middle_pos = hasMiddleTracker ? this->middleTracker->get_dist_traveled() : 0;

		// calculate change in each encoder
		double delta_left = (left_pos - prev_left_pos);
		double delta_right = (right_pos - prev_right_pos);
		double delta_middle = hasMiddleTracker
		                          ? (middle_pos - prev_middle_pos)
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

void OdometryModel::Task::begin() {
    pros::Task odom([=](){
        this->mainLoop();
    });
}

//==============================================================================
//                              ODOMETRY 
//==============================================================================

Eigen::Vector2d OdometryModel::getPosition() {
    Eigen::Vector2d position(task->pose.x(), task->pose.y());
    return position;
}

Eigen::Vector3d OdometryModel::getPose() {
    return task->pose;
}

void OdometryModel::setPose(double x, double y, double heading) {
    task->pose[0] = x;
    task->pose[1] = y;
    task->pose[2] = heading;
}

double OdometryModel::getHeading() {
    return task->pose.z();
}

OdometryModel::OdometryModel() 
: task{std::shared_ptr<task>(new task())}
{
}


//==============================================================================
//                          ODOMETRY BUILDER
//==============================================================================

OdometryModelBuilder::OdometryModelBuilder() {
}

OdometryModelBuilder& OdometryModelBuilder::withImu(int port) {
    odom.task->imu = std::make_shared<pros::Imu>(port);
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withDebug(bool debug) {
    odom.task->debug = debug;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withExpander(int port) {
    odom.task->expanderPort = port;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withLeftEncoder(int port) {
    if(odom.task->tpi == 0) {
        //throw error
    }
    if(odom.task->expanderPort == 0) {
        odom.task->leftTracker = std::make_shared<LegsDistanceTracker>(port, LEGS_ADI_ENCODER, port < 0, odom.task->tpi);
    } else {
        odom.task->leftTracker = std::make_shared<LegsDistanceTracker>(port, odom.task->expanderPort, port < 0, odom.task->tpi);
    }
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withRightEncoder(int port) {
    if(odom.task->tpi == 0) {
        //throw error
    }
    if(odom.task->expanderPort == 0) {
        odom.task->rightTracker = std::make_shared<LegsDistanceTracker>(port, LEGS_ADI_ENCODER, port < 0, odom.task->tpi);
    } else {
        odom.task->rightTracker = std::make_shared<LegsDistanceTracker>(port, odom.task->expanderPort, port < 0, odom.task->tpi);
    }
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleEncoder(int port) {
    if(odom.task->tpi == 0) {
        //throw error
    }
    if(odom.task->expanderPort == 0) {
        odom.task->middleTracker = std::make_shared<LegsDistanceTracker>(port, LEGS_ADI_ENCODER, port < 0, odom.task->tpi);
    } else {
        odom.task->middleTracker = std::make_shared<LegsDistanceTracker>(port, odom.task->expanderPort, port < 0, odom.task->tpi);
    }
    this->hasMiddleEncoder = true;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withLeftRotation(int port) {
    if(odom.task->tpi == 0) {
        //throw error
    }
    odom.task->leftTracker = std::make_shared<legs::LegsDistanceTracker>(port, LEGS_ROTATION, port < 0, odom.task->tpi);
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withRightRotation(int port) {
    if(odom.task->tpi == 0) {
        //throw error
    }
    odom.task->rightTracker = std::make_shared<legs::LegsDistanceTracker>(port, LEGS_ROTATION, port < 0, odom.task->tpi);
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleRotation(int port) {
    if(odom.task->tpi == 0) {
        //throw error
    }
    odom.task->middleTracker = std::make_shared<legs::LegsDistanceTracker>(port, LEGS_ROTATION, port < 0, odom.task->tpi);
    this->hasMiddleEncoder = true;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withTrackWidth(double track_width) {
    odom.task->track_width = track_width;
    odom.task->left_right_distance = 0.5 * track_width;
    this->hasTrackWidthOrLeftRightDistance = true;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withLeftRightDistance(double left_right_distance) {
    odom.task->left_right_distance = left_right_distance;
    odom.task->track_width = 2 * left_right_distance;
    this->hasTrackWidthOrLeftRightDistance = true;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleDistance(double middle_distance) {
    odom.task->middle_distance = middle_distance;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withLeftRightTPI(double tpi) {
    odom.task->tpi = tpi;
    this->hasTPI = true;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleTPI(double middle_tpi) {
    odom.task->middle_tpi = middle_tpi;
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
        odom.begin();
        return this->odom;
    } else {
        // throw error
    }
}

}