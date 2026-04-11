// Artificial Potential Field with a gap detection
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>
#include <vector>

enum RobotState { PRE_SLOPE = 0, CLIMBING = 1, POST_SLOPE = 2 };

class SlopeClimber {
private:
    ros::NodeHandle nh_;
    ros::Publisher  vel_pub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber laser_sub_;

    double current_yaw_   = 0.0;
    double current_pitch_ = 0.0;
    double current_roll_  = 0.0;
    std::vector<float> latest_scan_;
    double angle_min_ = 0.0;
    double angle_inc_ = 0.0;
    double range_max_ = 10.0;

    RobotState state_            = PRE_SLOPE;
    bool imu_ready_              = false;
    bool laser_ready_            = false;
    double uphill_yaw_reference_ = 0.0;
    bool obstacle_mode_          = false;   // hysteresis latch for gap detector

    double prev_v_ = 0.0;
    double prev_w_ = 0.0;

    // =====================================================================
    //  TUNING PARAMETERS
    // =====================================================================
    const double SLOPE_ENTRY_TILT_DEG = 5.0;
    const double SLOPE_EXIT_TILT_DEG  = 1.5;

    const double V_PRE_SLOPE = 0.5;
    const double V_CLIMBING  = 0.8;

    const double TARGET_WALL_DIST = 0.51;
    const double WALL_DEADBAND    = 0.06;
    const double K_WALL           = 1.0;
    const double MAX_WALL_PULL    = 0.6;

    // Hysteresis band: enter gap mode at DETECT, exit only at CLEAR
    const double HALFWALL_DETECT_DIST = 0.80;
    const double HALFWALL_CLEAR_DIST  = 1.10;
    const double K_GAP_BIAS           = 0.9;

    const double BUMPER_DIST = 0.40;
    const double K_BUMPER    = 0.6;

    const double K_YAW        = 0.6;
    const double YAW_DEADBAND = 0.03;

    const double K_V   = 1.0;
    const double K_W   = 0.8;
    const double ALPHA = 0.18;
    // =====================================================================

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);
        imu_ready_ = true;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        latest_scan_ = msg->ranges;
        angle_min_   = msg->angle_min;
        angle_inc_   = msg->angle_increment;
        range_max_   = msg->range_max;
        laser_ready_ = true;
    }

    double normalizeAngle(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    double minRangeInSector(double a_min, double a_max) {
        double min_r = range_max_;
        for (size_t i = 0; i < latest_scan_.size(); ++i) {
            double angle = angle_min_ + i * angle_inc_;
            if (angle < a_min || angle > a_max) continue;
            float r = latest_scan_[i];
            if (r > 0.05f && std::isfinite(r))
                min_r = std::min(min_r, (double)r);
        }
        return min_r;
    }

    double leftWallForce() {
        double left_dist = minRangeInSector(1.05, 2.09);
        if (left_dist >= range_max_) left_dist = TARGET_WALL_DIST;

        double error = left_dist - TARGET_WALL_DIST;
        if (std::abs(error) < WALL_DEADBAND) return 0.0;

        double f = K_WALL * (error - std::copysign(WALL_DEADBAND, error));
        return std::max(-MAX_WALL_PULL, std::min(MAX_WALL_PULL, f));
    }

    // Hysteresis gap detector: latches ON at DETECT threshold,
    // only releases at CLEAR threshold (0.30 m buffer between them).
    double gapDetectorForce() {
        const double FRONT_ARC = 0.785; // ±45 deg
        double min_front = minRangeInSector(-FRONT_ARC, FRONT_ARC);

        // Latch ON: obstacle closer than DETECT distance
        if (!obstacle_mode_ && min_front < HALFWALL_DETECT_DIST) {
            obstacle_mode_ = true;
            ROS_INFO("[SlopeClimber] Half-wall detected (%.2f m). Entering gap mode.",
                     min_front);
        }
        // Latch OFF: obstacle must clear past CLEAR distance before returning to wall-follow
        else if (obstacle_mode_ && min_front > HALFWALL_CLEAR_DIST) {
            obstacle_mode_ = false;
            ROS_INFO("[SlopeClimber] Obstacle cleared (%.2f m). Returning to wall follow.",
                     min_front);
        }

        if (!obstacle_mode_) return 0.0;

        // Compare average range: left-front half vs right-front half
        double sum_left = 0.0, sum_right = 0.0;
        int cnt_left = 0, cnt_right = 0;
        for (size_t i = 0; i < latest_scan_.size(); ++i) {
            double angle = angle_min_ + i * angle_inc_;
            if (angle < -FRONT_ARC || angle > FRONT_ARC) continue;
            float r = latest_scan_[i];
            if (r <= 0.05f || !std::isfinite(r)) continue;
            if (angle >= 0.0) { sum_left  += r; ++cnt_left;  }
            else              { sum_right += r; ++cnt_right; }
        }
        double avg_left  = (cnt_left  > 0) ? sum_left  / cnt_left  : range_max_;
        double avg_right = (cnt_right > 0) ? sum_right / cnt_right : range_max_;

        double bias = K_GAP_BIAS * (avg_left - avg_right);
        return std::max(-MAX_WALL_PULL, std::min(MAX_WALL_PULL, bias));
    }

    void frontBumperForce(double& fx, double& fy) {
        fx = 0.0; fy = 0.0;
        for (size_t i = 0; i < latest_scan_.size(); i += 2) {
            double angle = angle_min_ + i * angle_inc_;
            if (angle < -0.524 || angle > 0.524) continue; // ±30 deg
            float r = latest_scan_[i];
            if (r <= 0.05f || !std::isfinite(r) || r >= BUMPER_DIST) continue;
            double mag = K_BUMPER * ((1.0/r) - (1.0/BUMPER_DIST)) / (r * r);
            mag = std::min(mag, 3.0);
            fx += -mag * std::cos(angle);
            fy += -mag * std::sin(angle);
        }
    }

public:
    SlopeClimber() {
        vel_pub_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        imu_sub_   = nh_.subscribe("/imu/data",   10, &SlopeClimber::imuCallback,   this);
        laser_sub_ = nh_.subscribe("/front/scan", 10, &SlopeClimber::laserCallback, this);

        ROS_INFO("[SlopeClimber] Waiting for sensors...");
        while (ros::ok() && (!imu_ready_ || !laser_ready_)) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("[SlopeClimber] Sensors ready. Starting PRE_SLOPE phase.");
    }

    void run() {
        ros::Rate rate(20);
        geometry_msgs::Twist cmd;

        while (ros::ok() && state_ != POST_SLOPE) {
            ros::spinOnce();

            double pitch_deg = current_pitch_ * 180.0 / M_PI;
            double roll_deg  = current_roll_  * 180.0 / M_PI;
            double tilt      = std::sqrt(pitch_deg*pitch_deg + roll_deg*roll_deg);

            double target_v = 0.0;
            double target_w = 0.0;

            if (state_ == PRE_SLOPE && tilt > SLOPE_ENTRY_TILT_DEG) {
                state_                = CLIMBING;
                uphill_yaw_reference_ = current_yaw_;
                ROS_INFO("[SlopeClimber] Slope detected (tilt=%.1f deg). "
                         "Locking yaw=%.3f rad. Switching to CLIMBING.", tilt, uphill_yaw_reference_);
                continue;
            }

            if (state_ == CLIMBING && tilt < SLOPE_EXIT_TILT_DEG) {
                state_ = POST_SLOPE;
                ROS_INFO("[SlopeClimber] Flat ground detected. Mission complete.");
                break;
            }

            // ── PRE_SLOPE ─────────────────────────────────────────────────
            if (state_ == PRE_SLOPE) {
                double fx = V_PRE_SLOPE;
                double fy = leftWallForce();

                // Front bumper now active on flat ground too
                double bx = 0.0, by = 0.0;
                frontBumperForce(bx, by);
                fx += bx;
                fy += by;

                double heading = std::atan2(fy, fx);
                target_w = K_W * heading;
                target_v = std::max(0.0, std::min(fx, V_PRE_SLOPE));
                target_w = std::max(-1.2, std::min(1.2, target_w));
            }

            // ── CLIMBING ──────────────────────────────────────────────────
            else if (state_ == CLIMBING) {
                // 1. Yaw compass with deadband
                double yaw_err = normalizeAngle(uphill_yaw_reference_ - current_yaw_);
                double fx = V_CLIMBING;
                double fy = (std::abs(yaw_err) > YAW_DEADBAND)
                            ? K_YAW * std::sin(yaw_err)
                            : 0.0;

                // 2. Gap detector (hysteresis latched) takes priority;
                //    wall spring is fallback when path is clear
                double gap_bias = gapDetectorForce(); // also updates obstacle_mode_
                if (obstacle_mode_) {
                    fy += gap_bias;        // gap steering takes lateral control
                } else {
                    fy += leftWallForce(); // wall spring resumes
                }

                // 3. Close-range front bumper
                double bx = 0.0, by = 0.0;
                frontBumperForce(bx, by);
                fx += bx;
                fy += by;

                double heading   = std::atan2(fy, fx);
                double force_mag = std::sqrt(fx*fx + fy*fy);

                target_w = K_W * heading;
                target_v = K_V * force_mag * std::cos(heading);
                target_v = std::max(0.0, std::min(target_v, V_CLIMBING));
                target_w = std::max(-1.2, std::min(1.2, target_w));
            }

            // ── Exponential smoothing ─────────────────────────────────────
            cmd.linear.x  = ALPHA * target_v + (1.0 - ALPHA) * prev_v_;
            cmd.angular.z = ALPHA * target_w + (1.0 - ALPHA) * prev_w_;
            prev_v_ = cmd.linear.x;
            prev_w_ = cmd.angular.z;

            vel_pub_.publish(cmd);
            rate.sleep();
        }

        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        vel_pub_.publish(cmd);
        ROS_INFO("[SlopeClimber] === NAVIGATION COMPLETE ===");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "slope_climber");
    SlopeClimber robot;
    robot.run();
    return 0;
}