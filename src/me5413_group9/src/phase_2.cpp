#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// --- UTILITY FUNCTIONS ---
double getDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// --- CORE NAVIGATION FUNCTION ---
// Modified to include 'enable_stall_check' flag
bool navigatePartitioned(MoveBaseClient& ac, tf2_ros::Buffer& tfBuffer, 
                         const geometry_msgs::Pose& start, const geometry_msgs::Pose& target, 
                         double step_size, bool is_unblock_approach, 
                         bool enable_stall_check) 
{
    double total_dist = getDistance(start.position.x, start.position.y, target.position.x, target.position.y);
    int num_waypoints = std::max(0, (int)std::floor(total_dist / step_size));
    
    for (int i = 1; i <= (num_waypoints + 1); i++) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        bool is_last_segment = (i > num_waypoints);

        if (!is_last_segment) {
            double t = (double)i / (num_waypoints + 1);
            goal.target_pose.pose.position.x = start.position.x + t * (target.position.x - start.position.x);
            goal.target_pose.pose.position.y = start.position.y + t * (target.position.y - start.position.y);
            goal.target_pose.pose.orientation = target.orientation;
        } else {
            goal.target_pose.pose = target;
        }

        ac.sendGoal(goal);
        ros::Rate loop_rate(20);

        // --- VELOCITY / STALL TRACKING VARIABLES ---
        ros::Time last_vel_check = ros::Time::now();
        double prev_x = 0.0;
        double prev_y = 0.0;
        bool first_loop = true;
        
        while (ros::ok()) {
            try {
                geometry_msgs::TransformStamped tf = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
                double curr_x = tf.transform.translation.x;
                double curr_y = tf.transform.translation.y;

                if (first_loop) {
                    prev_x = curr_x;
                    prev_y = curr_y;
                    first_loop = false;
                }

                double d = getDistance(curr_x, curr_y, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
                
                // --- 1. FLAG-BASED ZERO VELOCITY CHECK ---
                if (enable_stall_check && (ros::Time::now() - last_vel_check).toSec() > 3.0) {
                    double dist_moved = getDistance(prev_x, prev_y, curr_x, curr_y);
                    
                    if (dist_moved < 0.05) { 
                        ROS_WARN("Stall detected (Velocity ~0). Breaking current segment.");
                        ac.cancelAllGoals();
                        return true; 
                    }
                    
                    prev_x = curr_x;
                    prev_y = curr_y;
                    last_vel_check = ros::Time::now();
                }

                // --- 2. STANDARD ARRIVAL CHECKS ---
                if (!is_last_segment && d < 0.6) break; 

                if (is_last_segment) {
                    double robot_yaw = tf2::getYaw(tf.transform.rotation);
                    double target_yaw = tf2::getYaw(goal.target_pose.pose.orientation);
                    double angle_diff = std::abs(robot_yaw - target_yaw);
                    if (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;

                    if (is_unblock_approach) {
                        if (d < 0.25 && angle_diff < 0.15) {
                            ac.cancelAllGoals(); 
                            return true; 
                        }
                    } else {
                        if (d < 0.2) {
                            ac.cancelAllGoals(); 
                            return true; 
                        }
                    }
                }
            } catch (tf2::TransformException &ex) {}
            loop_rate.sleep();
        }
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "phase_2_navigator_oop");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Publisher unblock_pub = nh.advertise<std_msgs::Bool>("/cmd_unblock", 1, true);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    // --- 1. DEFINE POSES ---
    geometry_msgs::Pose goal_1, goal_3;
    goal_1.position.x = 7.46693; goal_1.position.y = -1.82690;
    goal_1.orientation.z = -0.75186; goal_1.orientation.w = 0.65932;

    goal_3.position.x = 25.238539; goal_3.position.y = -3.7360713;
    goal_3.orientation.z = -0.009966; goal_3.orientation.w = 0.999950;

    // --- 2. GET CURRENT POSE ---
    geometry_msgs::TransformStamped tf_start;
    try {
        tf_start = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
    } catch (...) { return 1; }
    
    geometry_msgs::Pose current_pose;
    current_pose.position.x = tf_start.transform.translation.x;
    current_pose.position.y = tf_start.transform.translation.y;

    // --- 3. MISSION START: TO GOAL 1 ---
    // Stall check is FALSE here
    ROS_INFO("Navigating to Goal 1...");
    navigatePartitioned(ac, tfBuffer, current_pose, goal_1, 4.0, true, false);

    // --- 4. UNBLOCK & BURST ---
    std_msgs::Bool unblock_msg; unblock_msg.data = true;
    unblock_pub.publish(unblock_msg);
    ac.cancelAllGoals(); 

    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 1.0; 
    double target_burst_distance = 2.0; 
    
    geometry_msgs::TransformStamped burst_start_tf;
    try {
        burst_start_tf = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
    } catch (...) { ROS_ERROR("TF Error"); }
    
    double start_x = burst_start_tf.transform.translation.x;
    double start_y = burst_start_tf.transform.translation.y;

    ros::Rate burst_loop_rate(20);
    while (ros::ok()) {
        try {
            geometry_msgs::TransformStamped current_tf = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            double dist = getDistance(start_x, start_y, current_tf.transform.translation.x, current_tf.transform.translation.y);
            if (dist >= target_burst_distance) break;
            vel_pub.publish(move_cmd);
        } catch (...) {}
        burst_loop_rate.sleep();
    }
    move_cmd.linear.x = 0.0; 
    vel_pub.publish(move_cmd);

    // --- 5. MISSION CONTINUE: TO GOAL 3 ---
    // Stall check is TRUE here
    ROS_INFO("Navigating to Goal 3 with Stall Detection enabled...");
    try {
        tf_start = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
        current_pose.position.x = tf_start.transform.translation.x;
        current_pose.position.y = tf_start.transform.translation.y;
    } catch (...) {}

    navigatePartitioned(ac, tfBuffer, current_pose, goal_3, 5.0, false, true);

    ROS_INFO("PHASE 2 COMPLETE.");
    return 0;
}