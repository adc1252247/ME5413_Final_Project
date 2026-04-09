#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>

class SmoothedWallFollower {
private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber laser_sub_;
    
    double initial_slope_yaw_ = 0.0;
    double current_imu_yaw_ = 0.0;
    double current_imu_pitch_ = 0.0; 
    double current_imu_roll_  = 0.0; 
    
    double current_front_dist_ = 99.0; 
    double current_left_dist_  = 99.0; 

    bool imu_ready_ = false;
    bool laser_ready_ = false;
    bool baseline_set_ = false;
    bool is_on_slope_ = false; 

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        current_imu_yaw_ = tf2::getYaw(msg->orientation);
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_imu_pitch_ = pitch;
        current_imu_roll_  = roll; 
        imu_ready_ = true;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if (!baseline_set_) return;
        int num_beams = msg->ranges.size();
        
        int center_idx = num_beams / 2;
        double min_f = msg->range_max;
        for (int i = center_idx - 2; i <= center_idx + 2; ++i) {
            if (i >= 0 && i < num_beams && msg->ranges[i] > 0.1) min_f = std::min(min_f, (double)msg->ranges[i]);
        }
        current_front_dist_ = min_f;

        if (is_on_slope_) {
            double side_min_y = msg->range_max;
            double front_left_penalty = 0.0;

            for (int i = 0; i < num_beams; ++i) {
                float r = msg->ranges[i];
                if (r < 0.2 || r > msg->range_max) continue;

                double ray_angle = msg->angle_min + (i * msg->angle_increment);
                double corridor_angle = (current_imu_yaw_ + ray_angle) - initial_slope_yaw_;
                while (corridor_angle > M_PI) corridor_angle -= 2.0 * M_PI;
                while (corridor_angle < -M_PI) corridor_angle += 2.0 * M_PI;

                if (corridor_angle >= (45.0 * M_PI / 180.0) && corridor_angle <= (110.0 * M_PI / 180.0)) {
                    double lat_y = r * std::sin(corridor_angle);
                    side_min_y = std::min(side_min_y, lat_y);
                }

                if (corridor_angle > (5.0 * M_PI / 180.0) && corridor_angle < (45.0 * M_PI / 180.0)) {
                    double x_dist = r * std::cos(corridor_angle);
                    if (x_dist < 1.2) { 
                        front_left_penalty = std::max(front_left_penalty, (1.2 - x_dist) * 0.6);
                    }
                }
            }
            current_left_dist_ = side_min_y - front_left_penalty;
        } else {
            double pure_y = msg->range_max;
            for (int i = 0; i < num_beams; ++i) {
                float r = msg->ranges[i];
                if (r < 0.3) continue;
                double ray_ang = msg->angle_min + (i * msg->angle_increment);
                double corr_ang = (current_imu_yaw_ + ray_ang) - initial_slope_yaw_;
                if (std::abs(corr_ang - M_PI/2.0) < 0.2) pure_y = std::min(pure_y, (double)r);
            }
            current_left_dist_ = pure_y;
        }
        laser_ready_ = true;
    }

public:
    SmoothedWallFollower() {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        imu_sub_ = nh_.subscribe("/imu/data", 10, &SmoothedWallFollower::imuCallback, this);
        laser_sub_ = nh_.subscribe("/front/scan", 10, &SmoothedWallFollower::laserCallback, this);
        while (ros::ok() && !imu_ready_) { ros::spinOnce(); ros::Duration(0.1).sleep(); }
        initial_slope_yaw_ = current_imu_yaw_;
        baseline_set_ = true;
        while (ros::ok() && !laser_ready_) { ros::spinOnce(); ros::Duration(0.1).sleep(); }
    }

    void runClimb() {
        ros::Rate loop_rate(20);
        geometry_msgs::Twist cmd;

        // --- Velocity Parameters ---
        const double APPROACH_SPEED = 0.5; // Slow entry to prevent skidding
        const double CLIMB_SPEED = 0.9;    // Your preferred full speed

        while (ros::ok()) {
            ros::spinOnce();
            double pitch_deg = std::abs(current_imu_pitch_ * 180.0 / M_PI);
            double roll_deg  = std::abs(current_imu_roll_ * 180.0 / M_PI);
            double total_tilt = std::sqrt(pitch_deg*pitch_deg + roll_deg*roll_deg);

            if (!is_on_slope_ && total_tilt > 5.0) { 
                is_on_slope_ = true; 
                ROS_INFO("=== SLOPE ENTERED: Increasing to Full Speed ===");
            }
            if (is_on_slope_ && total_tilt < 1.5) break;

            double yaw_err = initial_slope_yaw_ - current_imu_yaw_;
            while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

            double wall_err = current_left_dist_ - 0.51;
            double k_yaw = (wall_err < -0.1) ? 0.12 : 1.2;

            double steer = (k_yaw * yaw_err) + (2.0 * wall_err);

            // --- Dynamic Velocity Logic ---
            double base_v = is_on_slope_ ? CLIMB_SPEED : APPROACH_SPEED;
            
            // Apply your preferred steering brake
            cmd.linear.x = (std::abs(steer) > 0.4) ? (base_v * 0.5) : base_v;
            
            // Front brake safety
            if (current_front_dist_ < 0.3) cmd.linear.x = 0.05;
            
            cmd.angular.z = std::max(-1.4, std::min(1.4, steer));

            vel_pub_.publish(cmd);
            loop_rate.sleep();
        }
        cmd.linear.x = 0; cmd.angular.z = 0;
        vel_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "slope_node");
    SmoothedWallFollower runner;
    runner.runClimb();
    return 0;
}