#pragma once

#include "api.h"

#include <memory>

#include "legs/motion_models/basic_model.hpp"
#include "legs/chassis/distance_tracker.hpp"


namespace legs {

typedef enum EncoderType { ENCODER_ADI, ENCODER_ROTATION } EncoderType_e_t;

/* Forward declarations so that both classes are aware of each other*/
class OdometryModelBuilder;
class OdometryModel;

class OdometryModel : public BasicModel
{
    private:
        // Please see the notes/Tasks.txt file for an explanation on why this
        // nested class exists.  
        class Task
        { 
        public:
            Task();

            int expanderPort = 0;

            EncoderType_e_t encoderType;

            std::shared_ptr<pros::Imu> imu = nullptr;
            std::shared_ptr<LegsDistanceTracker> rightTracker = nullptr;
            std::shared_ptr<LegsDistanceTracker> leftTracker = nullptr;
            std::shared_ptr<LegsDistanceTracker> middleTracker = nullptr;

            double track_width;
            double left_right_distance;
            double middle_distance;
            double tpi;
            double middle_tpi;

            double prev_left_pos;
            double prev_right_pos;
            double prev_middle_pos;
            double prev_heading;

            bool debug;

            Eigen::Vector3d pose;

            int mainLoop();
            void begin();
        };
        std::shared_ptr<Task> task;

        friend class OdometryModelBuilder;
    
    protected:
        OdometryModel();

    public:
        Eigen::Vector3d getPose();
        Eigen::Vector2d getPosition();
        double          getHeading();
        void setPose(double x, double y, double heading);
        inline void begin() { task->begin(); }
};

class OdometryModelBuilder
{
    private:
        OdometryModel odom;
        bool hasTrackWidthOrLeftRightDistance;
        bool hasTPI;
        bool hasIMU;
        bool hasMiddleEncoder;
        bool hasMiddleTPI;

    public:
        OdometryModelBuilder& withImu(int port);
        OdometryModelBuilder& withDebug(bool debug);
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

        OdometryModel build();

        OdometryModelBuilder();
};

} // namespace legs