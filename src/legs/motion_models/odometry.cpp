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