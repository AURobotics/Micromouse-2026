#include "pure_pursuit.h"
#include <algorithm>

PurePursuitPD::PurePursuitPD(double lookahead, double wheel_base, double kp, double kd)
    : lookahead_dist_(lookahead), 
      wheel_base_(wheel_base), 
      kp_o_(kp), 
      kd_o_(kd), 
      prev_error_o_(0.0),
      last_target_idx_(0) {}

Point PurePursuitPD::findLookaheadPoint(const Pose& current_pose, const std::vector<Point>& path) {
    if (path.empty()) {
        return {current_pose.x, current_pose.y};
    }

    Point lookahead_point = path.back();

    
    for (size_t i = last_target_idx_; i < path.size(); ++i) // Start searching from the last target index 
     {
        double dx = path[i].x - current_pose.x;
        double dy = path[i].y - current_pose.y;
        double dist = std::hypot(dx, dy);

        if (dist >= lookahead_dist_) {
            lookahead_point = path[i];
            last_target_idx_ = i;
            break;
        }
    }

    return lookahead_point;
}

wheelVelocity PurePursuitPD::computeControl(const Pose& current_pose, 
                                           double v_measured,
                                           double omega_measured,
                                           double target_v,
                                           const std::vector<Point>& path,
                                           double dt) {
                                    
    int size = path.size();
    if(current_pose.x == path[size-1].x && current_pose.y == path[size-1].y){
        wheelVelocity wheels;
        wheels.left  = 0;
        wheels.right = 0;
        return wheels;
    }
    
    Point lookahead_point = findLookaheadPoint(current_pose, path);
// bzbt el lookahead point ll current pose
    double dx = lookahead_point.x - current_pose.x;
    double dy = lookahead_point.y - current_pose.y;
    double y_body = -dx * std::sin(current_pose.theta) + dy * std::cos(current_pose.theta);

// generate curvature based on the lookahead point 
    double curvature = (2.0 * y_body) / (lookahead_dist_ * lookahead_dist_);

    // bzbt el omega ref based on generated curvature and target velocity
    double omega_ref = target_v * curvature;

// cntrl loop for omega using PD control
    double error_omega = omega_ref - omega_measured;
    double d_error_omega = (dt > 0.0) ? (error_omega - prev_error_o_) / dt : 0.0;
    prev_error_o_ = error_omega;

    double omega_cmd = omega_ref + (kp_o_ * error_omega) + (kd_o_ * d_error_omega);

// limit el omega command to prevent excessive turning rates
    const double max_omega = 12.0;
    omega_cmd = std::clamp(omega_cmd, -max_omega, max_omega);

    // convert el omega command to wheel velocities
    // bzbt el wheel velocities based on the commanded omega and target velocity
    wheelVelocity wheels;
    wheels.left  = target_v - (omega_cmd * wheel_base_ / 2.0);
    wheels.right = target_v + (omega_cmd * wheel_base_ / 2.0);

    return wheels;
}

void PurePursuitPD::reset() {
    prev_error_o_ = 0.0;
    last_target_idx_ = 0;
}

void PurePursuitPD::setGains(double kp, double kd) {
    kp_o_ = kp; 
    kd_o_ = kd;
}

void PurePursuitPD::setLookahead(double lookahead) {
    lookahead_dist_ = lookahead; 
}