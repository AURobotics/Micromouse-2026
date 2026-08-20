#pragma once

#include <cmath>
#include <vector>

struct Point {
    double x;
    double y;
};

struct Pose {
    double x;
    double y;
    double theta; // rad
};

struct wheelVelocity {
    double left;  // m/s
    double right; // m/s
};

class PurePursuitPD {
public:
    PurePursuitPD(double lookahead, double wheel_base, double kp, double kd);

    wheelVelocity computeControl(const Pose& current_pose, 
                                 double v_measured,
                                 double omega_measured,
                                 double target_v, 
                                 const std::vector<Point>& path,
                                 double dt);

    void reset();
    void setGains(double kp, double kd);
    void setLookahead(double lookahead);

private:
    Point findLookaheadPoint(const Pose& current_pose, const std::vector<Point>& path);

    double lookahead_dist_; // L_d 
    double wheel_base_;     // L (m)
    double kp_o_;           
    double kd_o_;           
    double prev_error_o_;   // omega error from previous timestep
    size_t last_target_idx_;// index of the last target point on the path
    bool is_first_run_;       // flag to check if it's the first run 
};