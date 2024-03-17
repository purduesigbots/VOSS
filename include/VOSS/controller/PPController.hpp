#pragma once

#include "VOSS/controller/PIDController.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/flags.hpp"
#include "VOSS/utils/Point.hpp"

namespace voss::controller {
class PPController : public AbstractController {
  public:
    PPController(std::shared_ptr<voss::localizer::AbstractLocalizer> l);

    bool isArrived() const;
    chassis::DiffChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    void reset() override;

  protected:


    double kp, kv, ka;
    double track_width;
    double min_error;
    bool can_reverse;

    double min_vel;

    double prev_lin_err, total_lin_err, prev_ang_err, total_ang_err;
    double lookahead_dist;
    double chase_power;
    double max_accelration;
    double prev_lookahead_index;
    Point prev_lookahead_pt;
    double weight_data, weight_smooth, tolerance;
    std::vector<double> distances, distance_along_path;
    std::vector<double> curvatures;
    std::vector<double> profile;
    int prev_closest_index = 0;
    bool firstIter = true;

    std::vector<Point> inject_points();
    void process_path();

    int get_closest();
    double get_curvature(const Point& P, const Point& Q, const Point& R);
    double get_curvature(const Pose& robot, const Point& pt);
    void get_distances_each_pt();
    void get_curvature();
    void generate_profile();
    std::optional<Point> get_lookahead_pt();
    Point circleLineIntersect(Point p1, Point p2);


    template <typename T> static int sgn(T val) {
        return val >= 0 ? 1 : -1;
    }

    friend class PPControllerBuilder;
};
} // namespace voss::controller