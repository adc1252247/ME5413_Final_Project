#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <cmath>
#include <algorithm>

class OpeningTracker {
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher cmd_pub;

    // State Logic
    bool in_corridor = false;
    int confirmation_counter = 0;

    // Tuning Parameters
    const int REQUIRED_CONSECUTIVE_SCANS = 30; 
    const float MIN_OPENING_DEPTH = 1.0;     
    const float SHOULDER_SKEW_TOLERANCE = 0.4; 
    
    // Side Confirmation Parameters
    const float MAX_SIDE_WALL_DIST = 2.0;    // Meters: Max distance to a wall on either side

    // Speed Control
    const float SEARCH_VEL = 0.3;            
    const float LINEAR_VEL = 0.8;            
    const float STOP_DISTANCE = 1.0;         

public:
    OpeningTracker() {
        scan_sub = nh.subscribe("front/scan", 10, &OpeningTracker::scanCallback, this);
        cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        ROS_INFO("System Initialized. Mode: SEARCHING.");
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        int n = msg->ranges.size();
        if (n < 2) return;

        // --- 1. Emergency Stop (ONLY after confirming corridor entry) ---
        if (in_corridor) {
            int mid_idx = n / 2;
            int check_range = n / 20; 
            bool obstacle_ahead = false;

            for (int i = mid_idx - check_range; i < mid_idx + check_range; ++i) {
                if (msg->ranges[i] < STOP_DISTANCE && msg->ranges[i] > 0.05) {
                    obstacle_ahead = true;
                    break;
                }
            }

            if (obstacle_ahead) {
                geometry_msgs::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.angular.z = 0.0;
                cmd_pub.publish(stop_msg);
                
                ROS_INFO("Front obstacle detected at target. Shutting down.");
                ros::shutdown(); 
                return;
            }
        }

        // --- 2. Pattern Detection ---
        int step = n / 18; 
        int best_idx = -1;
        float max_prominence = -1.0;
        bool pattern_seen_this_frame = false;

        for (int i = step; i < n - step; ++i) {
            float head = std::isfinite(msg->ranges[i]) ? msg->ranges[i] : msg->range_max;
            float left_s = std::isfinite(msg->ranges[i - step]) ? msg->ranges[i - step] : msg->range_max;
            float right_s = std::isfinite(msg->ranges[i + step]) ? msg->ranges[i + step] : msg->range_max;

            float diff_l = head - left_s;
            float diff_r = head - right_s;
            float symmetry = std::abs(left_s - right_s);

            if (diff_l > MIN_OPENING_DEPTH && diff_r > MIN_OPENING_DEPTH && symmetry < SHOULDER_SKEW_TOLERANCE) {
                pattern_seen_this_frame = true;
                float prominence = diff_l + diff_r;
                if (prominence > max_prominence) {
                    max_prominence = prominence;
                    best_idx = i;
                }
            }
        }

        // --- 3. Persistent Flag Logic (Updated with Side Lidar Info) ---
        if (!in_corridor) {
            // Check left side (index near 0) and right side (index near n-1) 
            // assuming a 180-degree scan where 0 is far right and n-1 is far left
            float right_side_dist = msg->ranges[0];
            float left_side_dist = msg->ranges[n - 1];

            // Validate side readings are finite
            bool right_wall = std::isfinite(right_side_dist) && right_side_dist < MAX_SIDE_WALL_DIST;
            bool left_wall = std::isfinite(left_side_dist) && left_side_dist < MAX_SIDE_WALL_DIST;

            // Pattern is seen AND we are flanked by walls
            if (pattern_seen_this_frame && left_wall && right_wall) {
                confirmation_counter++;
                if (confirmation_counter >= REQUIRED_CONSECUTIVE_SCANS) {
                    in_corridor = true;
                    ROS_INFO("--- CORRIDOR LATCHED (Sides Confirmed) ---");
                    ROS_INFO("Obstacle Detection: ARMED. Cruise Speed: %f", LINEAR_VEL);
                }
            } else {
                confirmation_counter = 0; 
            }
        }

        // --- 4. Motion Execution ---
        geometry_msgs::Twist drive_msg;
        if (best_idx != -1) {
            float target_angle = msg->angle_min + (best_idx * msg->angle_increment);
            drive_msg.linear.x = (in_corridor) ? LINEAR_VEL : SEARCH_VEL;
            drive_msg.angular.z = target_angle * 0.7; 
        } else {
            if (in_corridor) {
                drive_msg.linear.x = LINEAR_VEL * 0.9;
                drive_msg.angular.z = 0.0;
            } else {
                drive_msg.linear.x = 0.0; 
                drive_msg.angular.z = 0.3; 
            }
        }
        cmd_pub.publish(drive_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "corridor_navigator_node");
    OpeningTracker tracker;
    ros::spin();
    return 0;
}