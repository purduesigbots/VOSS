#include "legs/motion_models/odometry.hpp"

#include "api.h"

namespace legs {

Eigen::Vector2d OdometryModel::getPosition() {
    Eigen::Vector2d position(0.0, 0.0);

    return position;
}

Eigen::Vector3d OdometryModel::getPose() {
    Eigen::Vector3d pose(0.0, 0.0, 0.0);

    return pose;
}

double OdometryModel::getHeading() {
    return 0.0;
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
    if(this->odom.expanderPort == 0) {
        this->odom.leftADIEncoder = std::make_shared<pros::ADIEncoder>(port, port + 1, port < 0);
    } else {
        this->odom.leftADIEncoder = std::make_shared<pros::ADIEncoder>(std::tuple<int, int, int>(this->odom.expanderPort, port, port + 1), port < 0);
    }
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withRightEncoder(int port) {
    if(this->odom.expanderPort == 0) {
        this->odom.rightADIEncoder = std::make_shared<pros::ADIEncoder>(port, port + 1, port < 0);
    } else {
        this->odom.rightADIEncoder = std::make_shared<pros::ADIEncoder>(std::tuple<int, int, int>(this->odom.expanderPort, port, port + 1), port < 0);
    }
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleEncoder(int port) {
    if(this->odom.expanderPort == 0) {
        this->odom.middleADIEncoder = std::make_shared<pros::ADIEncoder>(port, port + 1, port < 0);
    } else {
        this->odom.middleADIEncoder = std::make_shared<pros::ADIEncoder>(std::tuple<int, int, int>(this->odom.expanderPort, port, port + 1), port < 0);
    }
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withLeftRotation(int port) {
    this->odom.leftRotation = std::make_shared<pros::Rotation>(port);
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withRightRotation(int port) {
    this->odom.rightRotation = std::make_shared<pros::Rotation>(port);
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleRotation(int port) {
    this->odom.middleRotation = std::make_shared<pros::Rotation>(port);
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withTrackWidth(double track_width) {
    this->odom.track_width = track_width;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withLeftRightDistance(double left_right_distance) {
    this->odom.left_right_distance = left_right_distance;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleDistance(double middle_distance) {
    this->odom.middle_distance = middle_distance;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withLeftRightTPI(double tpi) {
    this->odom.tpi = tpi;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withMiddleTPI(double middle_tpi) {
    this->odom.middle_tpi = middle_tpi;
    return *this;
}

OdometryModelBuilder& OdometryModelBuilder::withEncoderType(EncoderType_e_t type) {
    this->odom.encoderType = type;
    return *this;
}

OdometryModel OdometryModelBuilder::build() {
    return this->odom;
}

}