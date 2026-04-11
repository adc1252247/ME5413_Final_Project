#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>

enum State { APPROACH, CLIMB, FINISHED };

class JackalPotentialField {
private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber imu_sub_, laser_sub_;

    State current_state_ = APPROACH;
    double cur_roll_ = 0.0, cur_pitch_ = 0.0, cur_yaw_ = 0.0, start_yaw_ = 0.0;
    double d_front_ = 99.0, d_left_ = 99.0, d_front_left_ = 99.0;
    bool imu_init_ = false, laser_init_ = false, baseline_set_ = false;
    bool mission_complete_ = false;

    // Temporal Accumulators
    double wind_force_built_up_ = 0.0;
    double last_angular_z_ = 0.0;

    // --- TUNED PARAMETERS ---
    const double D_SET = 0.51;          // Desired distance from left wall
    const double WIND_THRESH = 0.50;    // Heading tolerance (~28 deg)
    const double WIND_GROWTH = 0.015;   // Building up slowly (Adjusted for 20Hz)
    const double WIND_MAX = 0.7;        // Max heading correction push
    const double APPROACH_SPEED = 0.4;
    const double CLIMB_SPEED = 0.75;
    const double OBS_THRESHOLD = 0.85;  // Distance to start braking/dodging
    const double CRITICAL_DIST = 0.40;  // Distance for stationary pivot
    const double K_WALL = 1.5;          // Wall following gain
    const double SMOOTH_FACTOR = 0.3;   // Steering low-pass filter

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        cur_roll_ = r;
        cur_pitch_ = p; 
        cur_yaw_ = y;

        if (!baseline_set_) {
            start_yaw_ = cur_yaw_;
            baseline_set_ = true;
        }
        imu_init_ = true;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        int n = msg->ranges.size();
        // Narrower front window (center 7 beams)
        double min_f = 99.0;
        for (int i = n/2 - 3; i <= n/2 + 3; ++i) {
            if (msg->ranges[i] > 0.1) min_f = std::min(min_f, (double)msg->ranges[i]);
        }
        d_front_ = min_f;
        d_left_ = msg->ranges[n * 3/4];
        d_front_left_ = msg->ranges[n * 5/8];
        laser_init_ = true;
    }

public:
    JackalPotentialField() {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        imu_sub_ = nh_.subscribe("/imu/data", 10, &JackalPotentialField::imuCallback, this);
        laser_sub_ = nh_.subscribe("/front/scan", 10, &JackalPotentialField::laserCallback, this);
    }

    void run() {
        ros::Rate loop_rate(20);
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0; stop_cmd.angular.z = 0.0;

        while (ros::ok()) {
            ros::spinOnce();
            if (!imu_init_ || !laser_init_) continue;

            // 1. Calculate Multi-Axis Tilt
            double pitch_deg = cur_pitch_ * 180.0 / M_PI;
            double roll_deg  = cur_roll_ * 180.0 / M_PI;
            double total_tilt = std::sqrt(pitch_deg*pitch_deg + roll_deg*roll_deg);

            // Calculate Heading Error
            double yaw_err = start_yaw_ - cur_yaw_;
            while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

            // --- 2. STATE MACHINE MANAGEMENT ---
            if (current_state_ == APPROACH && total_tilt > 5.0) {
                current_state_ = CLIMB;
                ROS_INFO("=== PHASE 2: CLIMBING STARTED (Total Tilt: %.2f) ===", total_tilt);
            }
            
            // Exit logic: Both Pitch and Roll must be level
            if (current_state_ == CLIMB && total_tilt < 1.2) { 
                current_state_ = FINISHED;
                mission_complete_ = true;
            }

            if (mission_complete_) {
                vel_pub_.publish(stop_cmd);
                ROS_INFO("=== SLOPE COMPLETE: HARD STOP EXECUTED ===");
                ros::Duration(1.0).sleep(); 
                ros::shutdown();
                return;
            }

            // --- 3. VECTOR CALCULATION ---
            double F_res_x = (current_state_ == APPROACH) ? APPROACH_SPEED : CLIMB_SPEED;
            double F_res_y = 0.0;
            bool dodging = false;

            // Front Obstacle Avoidance
            if (d_front_ < OBS_THRESHOLD) {
                dodging = true;
                // Braking logic as we approach wall
                double range = OBS_THRESHOLD - CRITICAL_DIST;
                double dist_into_obs = OBS_THRESHOLD - d_front_;
                double speed_factor = std::max(0.05, 1.0 - (dist_into_obs / range));
                F_res_x *= speed_factor;

                if (d_front_left_ < (OBS_THRESHOLD + 0.1)) {
                    F_res_y -= 1.5; // Turn Right (away from wall)
                } else {
                    F_res_y += 1.3; // Turn Left (gap available)
                }
            } else {
                // Bi-directional Wall Follow
                F_res_y += (d_left_ - D_SET) * K_WALL;
            }

            // Accumulative Cross-wind
            if (!dodging && std::abs(yaw_err) > WIND_THRESH) {
                double direction = (yaw_err > 0) ? 1.0 : -1.0;
                wind_force_built_up_ += direction * WIND_GROWTH;
            } else {
                // Quick decay when on track or dodging to reset steering
                wind_force_built_up_ *= 0.80; 
            }
            wind_force_built_up_ = std::max(-WIND_MAX, std::min(wind_force_built_up_, WIND_MAX));
            F_res_y += wind_force_built_up_;

            // --- 4. OUTPUT MAPPING ---
            geometry_msgs::Twist cmd;
            
            // Force stationary pivot if dangerously close to front wall
            if (d_front_ < CRITICAL_DIST) {
                cmd.linear.x = 0.0;
            } else {
                cmd.linear.x = std::max(0.0, std::min(F_res_x, CLIMB_SPEED));
            }

            // Apply Low-Pass Smoothing to steering
            double target_steer = std::max(-1.6, std::min(F_res_y, 1.6));
            cmd.angular.z = (SMOOTH_FACTOR * target_steer) + ((1.0 - SMOOTH_FACTOR) * last_angular_z_);
            last_angular_z_ = cmd.angular.z;

            vel_pub_.publish(cmd);
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "jackal_smooth_climb_node");
    JackalPotentialField climber;
    climber.run();
    return 0;
}