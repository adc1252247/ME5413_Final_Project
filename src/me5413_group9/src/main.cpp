/**
 * Strategies:
 *   - Room 1: Go to each corner using move_base, stop and face box to detect when new one is found
 *   - Outdoors: Wall following
 *   - Room 2: Go to each partition to check for obstacle, have stop on either side of cylinder too
 * 
 * Intended for me5413_world navigation.launch w/ me5413_group9's lvl1_gmapping
 */

 #include <stdexcept>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/LoadMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/// @brief Move to a location, returning whether or not target was reached
bool move_to(double x, double y, double z, double w, double timeout = 30) {
    MoveBaseClient mbc("move_base", true); /// @todo - Make this global

    if ( !mbc.waitForServer(ros::Duration(5.0)) ) {
        ROS_ERROR("move_base action not available!");
        throw std::runtime_error("move_base action not available!");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = z;
    goal.target_pose.pose.orientation.w = w;

    ROS_INFO("move_base requesting move to (%.2f, %.2f)", x, y);
    mbc.sendGoal(goal);

    bool finished = mbc.waitForResult(ros::Duration(timeout));

    if ( finished ) {
        actionlib::SimpleClientGoalState state = mbc.getState();

        if ( state == actionlib::SimpleClientGoalState::SUCCEEDED ) {
            return true;
        }

        /// @todo - Handle timeout vs no path...
        // else {
            ROS_WARN("move_base failed to reach (%.2f, %.2f) in time or entirely...", x, y);
            return false;
        // }
    }
    // else
        ROS_WARN("move_base timed out (%.0f s)", timeout);
        mbc.cancelGoal();
        return false;

}

void amcl_estimate(double x, double y, double z, double w) {
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    ros::Duration(0.5).sleep();

    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = ros::Time::now();

    pose_msg.pose.pose.position.x = x;
    pose_msg.pose.pose.position.y = y;
    pose_msg.pose.pose.position.z = 0;

    pose_msg.pose.pose.orientation.x = 0;
    pose_msg.pose.pose.orientation.y = 0;
    pose_msg.pose.pose.orientation.z = z;
    pose_msg.pose.pose.orientation.w = w;

    /// @todo - Check that this is good for all, not just level 1 init.
    pose_msg.pose.covariance = {
        0.25, 0,    0, 0, 0, 0,
        0,    0.25, 0, 0, 0, 0,
        0,    0,    0, 0, 0, 0,
        0,    0,    0, 0, 0, 0,
        0,    0,    0, 0, 0, 0,
        0,    0,    0, 0, 0, 0.06853892326654787
    };

    pose_pub.publish(pose_msg);
    ROS_INFO("AMCL initialized to map position (%.2f. %.2f)", x, y);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "group9_main");
    ros::NodeHandle nh;
    
    amcl_estimate(2.386495590209961, 5.6931610107421875, -0.4612403984795658, 0.8872752080445003);
    ros::spinOnce();

    move_to(2.9305427074432373, -3.29251766204834, -0.49092752830359954, 0.871200414344322);
    ros::spinOnce();

    return 0;
}
