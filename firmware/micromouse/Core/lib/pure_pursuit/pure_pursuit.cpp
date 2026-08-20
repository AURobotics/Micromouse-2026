#include "pure_pursuit.h"
#include <algorithm>

PurePursuitPD::PurePursuitPD(double lookahead, double wheel_base, double kp, double kd)
    : lookahead_dist_(lookahead), 
      wheel_base_(wheel_base), 
      kp_o_(kp), 
      kd_o_(kd), 
      prev_error_o_(0.0),
      last_target_idx_(0) ,
      is_first_run_(true) {}

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
        
    wheelVelocity wheels={target_v, target_v}; // yfdl mashy in linear velocity l7d m el algo y2ol el motion type elly 3ayzo

    if (path.empty()) {  // kda lw el path empty yfdl mashy in linear velocity l7d m el algo y2ol el motion type elly 3ayzo?? is that correct??
        return wheels; 
    }

    const Point& goal= path.back();
    double goal_dist = std::hypot(goal.x - current_pose.x, goal.y - current_pose.y);
    if (goal_dist < 0.01) {
       // ana 3ayza el 🛺 y3rf en kda el turn 5lst f yrg3 ymsh f line aw zy ma el algo hy2ol elmotion type
         return wheels; 
    }

    Point lookahead_point = findLookaheadPoint(current_pose, path);
// bzbt el lookahead point ll current pose
    double dx = lookahead_point.x - current_pose.x;
    double dy = lookahead_point.y - current_pose.y;
    double y_body = -dx * std::sin(current_pose.theta) + dy * std::cos(current_pose.theta);
    
    if (lookahead_dist_ <= 0.001) {   // guard mn el lookahead distance being too small
        return wheels;
    }
// generate curvature based on the lookahead point 
    double curvature = (2.0 * y_body) / (lookahead_dist_ * lookahead_dist_);

    // bzbt el omega ref based on generated curvature and target velocity
    double omega_ref = target_v * curvature;

// cntrl loop for omega using PD control
    double error_omega = omega_ref - omega_measured;
    double d_error_omega = 0.0;
   
   
    if (is_first_run_) {  //skip derivative fel first run to avoid large spikes
        is_first_run_ = false;
    } else if (dt > 0.0) {
        d_error_omega = (error_omega - prev_error_o_) / dt;
    }
    
    
    
    prev_error_o_ = error_omega;

    double omega_cmd = omega_ref + (kp_o_ * error_omega) + (kd_o_ * d_error_omega);

// limit el omega command to prevent excessive turning rates
    const double max_omega = 12.0;
    omega_cmd = std::clamp(omega_cmd, -max_omega, max_omega);

    // convert el omega command to wheel velocities
    // bzbt el wheel velocities based on the commanded omega and target velocity
    
    wheels.left  = target_v - (omega_cmd * wheel_base_ / 2.0);
    wheels.right = target_v + (omega_cmd * wheel_base_ / 2.0);

    return wheels;
}

void PurePursuitPD::reset() {
    prev_error_o_ = 0.0;
    last_target_idx_ = 0;
    is_first_run_ = true;
}

void PurePursuitPD::setGains(double kp, double kd) {
    kp_o_ = kp; 
    kd_o_ = kd;
}

void PurePursuitPD::setLookahead(double lookahead) {
    lookahead_dist_ = lookahead; 
}