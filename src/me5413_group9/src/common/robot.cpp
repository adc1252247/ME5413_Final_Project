#include "robot.hpp"

#include <tf/tf.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/LoadMap.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

Pose::Pose(double x_, double y_, double yaw_): 
    x(x_), 
    y(y_), 
    yaw(yaw_) {}

Pose::Pose(double x_, double y_, double z_, double w_):
    x(x_),
    y(y_),
    yaw(tf::getYaw(tf::Quaternion(0, 0, z_, w_))) {}

Pose Pose::flip() const {
    return Pose(x, y, yaw + M_PI);
}

Pose Pose::add_yaw(double yaw_) const {
    return Pose(x, y, yaw + yaw_);
}

double Pose::get_z() const {
    return tf::createQuaternionFromYaw(yaw).getZ();
} 

double Pose::get_w() const {
    return tf::createQuaternionFromYaw(yaw).getW();
}

bool Pose::operator==(const Pose& rhs) const {
    return x == rhs.x && y == rhs.y && yaw == rhs.yaw;
}

bool Pose::operator!=(const Pose& rhs) const {
    return !(*this == rhs);
}

Covariance::Covariance(double x_, double y_, double yaw_):
    x(x_),
    y(y_),
    yaw(yaw_) {}

Covariance::Covariance(const boost::array<double, 36>& c):
    x(c[0]),
    y(c[7]),
    yaw(c[35]) {}

double Covariance::std_x() const {
    return sqrt(x);
}

double Covariance::std_y() const {
    return sqrt(y);
}

double Covariance::std_yaw() const {
    return sqrt(yaw);
}

bool Covariance::good() const {
    return x < good_threshold && y < good_threshold;
}

bool Covariance::bad() const {
    return x > bad_threshold && y > bad_threshold;
}

boost::array<double, 36> Covariance::array() const {
    return {
        x, 0, 0, 0, 0, 0,
        0, y, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, yaw
    };
}

Robot::Robot(): _move_base(std::string("move_base"), true) {
    if ( !_move_base.waitForServer(ros::Duration(1.0)) ) {
        ROS_ERROR("<Robot> move_base was not available in 1.0s!");
        throw std::runtime_error("<Robot> move_base was not available in 1.0s!");
    }

    _est = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    _pose = _nh.subscribe("/amcl_pose", 1, &Robot::pose_callback, this);
    // _pose = _nh.subscribe("/amcl_pose", 1, ..., this);
    // _scan = _nh.subscribe();
    // _img  = _nh.subscribe();
}

void Robot::clear_cone() {
    ros::Publisher cone_pub = _nh.advertise<std_msgs::Bool>("/cmd_unblock", 1);
    std_msgs::Bool msg;
    ros::Duration(1.0).sleep(); // Give it some time to register or whatever...
    msg.data = true;
    cone_pub.publish(msg);
    ros::spinOnce();
}

void Robot::move_to(Pose pose_, int retries) {
    using State = actionlib::SimpleClientGoalState;
    const Covariance re_est_cov(0.15, 0.15, 0.07);

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp    = ros::Time::now();

    goal.target_pose.pose.position.x = pose_.x;
    goal.target_pose.pose.position.y = pose_.y;
    goal.target_pose.pose.position.z = 0.0;
    
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = pose_.get_z();
    goal.target_pose.pose.orientation.w = pose_.get_w();

    _move_base.sendGoal(goal);

    ros::Rate loop_rate(10);

    while ( ros::ok() && _move_base.getState() == State::PENDING )
        loop_rate.sleep();

    while ( ros::ok() && _move_base.getState() == State::ACTIVE ) {
        ros::spinOnce();
        
        // If robot gets lost
        if ( _latest_covariance.bad() ) {
            const Covariance& c = _latest_covariance;
            ROS_ERROR("<Robot> move_base motion resulted in too much covariance!");
            ROS_ERROR("        x: %.2f | y: %.2f | yaw: %.2f", c.x, c.y, c.yaw);

            // Retry
            if ( retries > 0 ) {
                ROS_INFO("<Robot> move_to retrying - resetting estimate | remaining: %d", retries);
                // set_estimate(_latest_good_pose, re_est_cov);
                set_estimate(_latest_pose, re_est_cov);
                return move_to(pose_, retries - 1);
            }
            else {
                ROS_ERROR("<Robot> move_to no retries remaining.");
                ROS_ERROR(
                    "        Current: (x: %.2f | y: %.2f | yaw: %.2f)",
                    pose().x,
                    pose().y,
                    pose().yaw
                );
                stop();

                ROS_ERROR(
                    "        Target: (x: %.2f | y: %.2f | yaw: %.2f)",
                    pose_.x,
                    pose_.y,
                    pose_.yaw
                );
                throw std::runtime_error("<Robot> move_to retries went to 0!");
            }
        }

        loop_rate.sleep();
    }

    ros::spinOnce();

    State end_state = _move_base.getState();
    if ( end_state != State::SUCCEEDED ) {
        ROS_ERROR("<Robot> move_base did not succeed!");
        // if ( retries > 0 ) {
        //     ROS_INFO("        Retrying...");
        //     set_estimate(_latest_good_pose, re_est_cov);
        //     return move_to(pose_, retries - 1);
        // }

        ROS_ERROR("        State (%d): %s", end_state.state_, end_state.getText().c_str());
        throw std::runtime_error("<Robot> move_base did not succeed!");
    }
}

void Robot::change_map(const std::string& pkg, const std::string& map) {
    std::string path = ros::package::getPath(pkg);

    if ( path.empty() ) {
        ROS_ERROR("<Robot> package '%s' not found for map change!", pkg.c_str());
        throw std::runtime_error("<Robot> package not found for map change!");
    }

    if ( map.size() == 0 ) {
        ROS_ERROR("<Robot> map given with length 0!");
        throw std::runtime_error("<Robot> map given with length 0!");
    }

    if ( map.front() != '/' )
        path += "/";
    path += map;

    ros::ServiceClient client = _nh.serviceClient<nav_msgs::LoadMap>("change_map");
    client.waitForExistence();

    nav_msgs::LoadMap msg;
    msg.request.map_url = path;

    if ( !client.call(msg) ) {
        ROS_ERROR("<Robot> change_map service failed!");
        throw std::runtime_error("<Robot> change_map service failed!");
    }

    if ( msg.response.result != msg.response.RESULT_SUCCESS ) {
        ROS_ERROR("<Robot> change_map failed, code: %d!", msg.response.result);
        throw std::runtime_error("<Robot> change_map failed!");
    }

    ROS_INFO("Map changed to: %s", map.c_str());
    ros::Duration(0.5).sleep();
}

void Robot::set_estimate(Pose pose_) {
    set_estimate(pose_, Covariance());
}

void Robot::set_estimate(Pose pose_, Covariance cov_) {
    stop();

    geometry_msgs::PoseWithCovarianceStamped msg;
    
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();

    msg.pose.pose.position.x = pose_.x;
    msg.pose.pose.position.y = pose_.y;
    msg.pose.pose.position.z = 0;

    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = pose_.get_z();
    msg.pose.pose.orientation.w = pose_.get_w();

    msg.pose.covariance = cov_.array();

    // First clear the local costmap, then update
    clear_costmap();

    _est.publish(msg);
    ros::spinOnce();

    ros::Duration(3.0).sleep(); // Takes longer w/ TEB
    ros::spinOnce();

    ROS_INFO("Set map estimate to position (%.2f, %.2f)", pose_.x, pose_.y);
}

void Robot::clear_costmap() {
    ros::ServiceClient client = _nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    client.waitForExistence();

    std_srvs::Empty msg;
    
    if ( client.call(msg) )
        ROS_INFO("Cleared costmap!");
    else {
        ROS_ERROR("<Robot> clear_costmap failed!");
        throw std::runtime_error("<Robot> clear_costmap failed!");
    }
}

void Robot::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    Pose curr(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
        
    _latest_covariance = msg->pose.covariance;

    _latest_pose = curr;   

    if ( !_latest_covariance.bad() )
        _latest_good_pose = curr;
    else
        ROS_WARN(
            "<Robot> covariance of pose is very high (x: %.2f, y: %.2f, yaw: %.2f)",
            _latest_covariance.x,
            _latest_covariance.y,
            _latest_covariance.yaw
        );
}

const Pose& Robot::pose() const {
    return _latest_pose;
}

const Pose& Robot::last_good_pose() const {
    return _latest_good_pose;
}

const Covariance& Robot::covariance() const {
    return _latest_covariance;
}

void Robot::stop() {
    using State = actionlib::SimpleClientGoalState;
    ros::Rate loop_rate(10);

    // Unsure if I can cancel a PENDING state... 
    while ( ros::ok() && _move_base.getState() == State::PENDING )
        loop_rate.sleep();

    if ( _move_base.getState() == State::ACTIVE )
        _move_base.cancelGoal();

    ros::spinOnce();
}