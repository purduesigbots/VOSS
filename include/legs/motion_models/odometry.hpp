#pragma once

#include "api.h"
#include "legs/motion_models/basic_model.hpp"

namespace legs {

typedef enum EncoderType { ENCODER_ADI, ENCODER_ROTATION } EncoderType_e_t;

/* Forward declarations so that both classes are aware of each other*/
class OdometryModelBuilder;
class OdometryModel;

class OdometryModel : public BasicModel
{
    private:

        int expanderPort = 0;

        EncoderType_e_t encoderType;

        std::shared_ptr<pros::Imu> imu = nullptr;
        std::shared_ptr<pros::Rotation> rightRotation = nullptr;
        std::shared_ptr<pros::Rotation> leftRotation = nullptr;
        std::shared_ptr<pros::Rotation> middleRotation = nullptr;
        std::shared_ptr<pros::ADIEncoder> rightADIEncoder = nullptr;
        std::shared_ptr<pros::ADIEncoder> leftADIEncoder = nullptr;
        std::shared_ptr<pros::ADIEncoder> middleADIEncoder = nullptr;

        double track_width;
        double left_right_distance;
        double middle_distance;
        double tpi;
        double middle_tpi;

        friend class OdometryModelBuilder;
    
    protected:
        OdometryModel();

    public:
        Eigen::Vector3d getPose();
        Eigen::Vector2d getPosition();
        double          getHeading();
};

class OdometryModelBuilder
{
    private:
        OdometryModel odom;

    public:
        OdometryModelBuilder& withImu(int port);
        OdometryModelBuilder& withExpander(int port);
        OdometryModelBuilder& withRightEncoder(int port);
        OdometryModelBuilder& withLeftEncoder(int port);
        OdometryModelBuilder& withMiddleEncoder(int port);
        OdometryModelBuilder& withRightRotation(int port);
        OdometryModelBuilder& withLeftRotation(int port);
        OdometryModelBuilder& withMiddleRotation(int port);
        OdometryModelBuilder& withTrackWidth(double track_width);
        OdometryModelBuilder& withLeftRightDistance(double left_right_distance);
        OdometryModelBuilder& withMiddleDistance(double middle_distance);
        OdometryModelBuilder& withLeftRightTPI(double left_right_tpi);
        OdometryModelBuilder& withMiddleTPI(double middle_tpi);
        OdometryModelBuilder& withEncoderType(EncoderType_e_t type);

        OdometryModel build();

        OdometryModelBuilder();
};

} // namespace legs