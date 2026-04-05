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
// Handles partitioning, waypoint fly-bys, and final approach alignment
bool navigatePartitioned(MoveBaseClient& ac, tf2_ros::Buffer& tfBuffer, 
                         const geometry_msgs::Pose& start, const geometry_msgs::Pose& target, 
                         double step_size, bool is_unblock_approach) 
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
            // Orient waypoints toward the final target early to save time turning later
            goal.target_pose.pose.orientation = target.orientation;
        } else {
            goal.target_pose.pose = target;
        }

        ac.sendGoal(goal);
        ros::Rate loop_rate(20);
        
        while (ros::ok()) {
            try {
                geometry_msgs::TransformStamped tf = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
                double d = getDistance(tf.transform.translation.x, tf.transform.translation.y,
                                       goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
                
                if (!is_last_segment && d < 0.6) break; // Waypoint fly-by

                if (is_last_segment) {
                    double robot_yaw = tf2::getYaw(tf.transform.rotation);
                    double target_yaw = tf2::getYaw(goal.target_pose.pose.orientation);
                    double angle_diff = std::abs(robot_yaw - target_yaw);
                    if (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;

                    // If approaching the cone, we need alignment. If just Goal 3, distance is enough.
                    if (is_unblock_approach) {
                        if (d < 0.25 && angle_diff < 0.15) return true; 
                    } else {
                        if (d < 0.2) return true; // Standard arrival
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
    geometry_msgs::Pose goal_1, goal_2, goal_3;
    goal_1.position.x = 7.46693; goal_1.position.y = -1.82690;
    goal_1.orientation.z = -0.75186; goal_1.orientation.w = 0.65932;

    goal_2.position.x = 7.17949; goal_2.position.y = -3.73237; // Burst target

    goal_3.position.x = 25.238539; goal_3.position.y = -4.2360713;
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
    ROS_INFO("Mission Start: Navigating to Goal 1 (Cone Area)...");
    navigatePartitioned(ac, tfBuffer, current_pose, goal_1, 4.0, true);

    // --- 4. UNBLOCK & BURST ---
    std_msgs::Bool unblock_msg; unblock_msg.data = true;
    unblock_pub.publish(unblock_msg);
    ac.cancelAllGoals(); 
    ROS_INFO("Unblocked! Initiating Manual Burst to Goal 2...");

    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 1.0;
    while (ros::ok()) {
        try {
            geometry_msgs::TransformStamped tf = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            if (getDistance(tf.transform.translation.x, tf.transform.translation.y, goal_2.position.x, goal_2.position.y) < 0.2) break;
            vel_pub.publish(move_cmd);
        } catch (...) {}
    }
    move_cmd.linear.x = 0.0; vel_pub.publish(move_cmd); // Stop burst

    // --- 5. MISSION CONTINUE: TO GOAL 3 ---
    ROS_INFO("Burst complete. Navigating to Final Destination (Goal 3)...");
    
    // Refresh current pose after burst
    try {
        tf_start = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
        current_pose.position.x = tf_start.transform.translation.x;
        current_pose.position.y = tf_start.transform.translation.y;
    } catch (...) {}

    navigatePartitioned(ac, tfBuffer, current_pose, goal_3, 5.0, false);

    ROS_INFO("PHASE 2 COMPLETE. Successfully reached Goal 3.");
    return 0;
}