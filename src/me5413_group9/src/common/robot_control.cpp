#include "robot_control.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <memory>

// =========================== :::::::::::::::::::::::::::::::::::::::::
// === Variables & Helpers === :::::::::::::::::::::::::::::::::::::::::
// =========================== :::::::::::::::::::::::::::::::::::::::::
namespace robot_control {
    std::unique_ptr<ros::NodeHandle> nh;
    
    ros::Subscriber            image_sub;
    cv_bridge::CvImageConstPtr latest_image;

    ros::Timer           twist_timer;
    ros::Publisher       twist_pub;
    bool                 stop_twist = true;
    geometry_msgs::Twist next_twist;

    void init();
    
    void set_twist(double lin, double ang);
}

// ================= :::::::::::::::::::::::::::::::::::::::::::::::::::
// === Callbacks === :::::::::::::::::::::::::::::::::::::::::::::::::::
// ================= :::::::::::::::::::::::::::::::::::::::::::::::::::
void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    robot_control::latest_image = cv_bridge::toCvShare(msg, "bgr8");
}

void send_twist_callback(const ros::TimerEvent& event) {
    if ( !robot_control::stop_twist )
        robot_control::twist_pub.publish(robot_control::next_twist);
}

// ======================= :::::::::::::::::::::::::::::::::::::::::::::
// === Implementations === :::::::::::::::::::::::::::::::::::::::::::::
// ======================= :::::::::::::::::::::::::::::::::::::::::::::
void robot_control::init() {
    nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

    image_sub = nh->subscribe("/front/image_raw", 1, image_callback);

    twist_pub   = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    twist_timer = nh->createTimer(ros::Duration(0.05), send_twist_callback);
}

void robot_control::set_twist(double lin, double ang) {
    geometry_msgs::Twist temp;

    temp.linear.x = lin;
    temp.linear.y = 0;
    temp.linear.z = 0;
    temp.angular.x = 0;
    temp.angular.y = 0;
    temp.angular.z = ang;
    
    next_twist = temp;

    stop_twist = (lin == 0.0) && (ang == 0.0);
}

void robot_control_init() {
    robot_control::init();
}

void manual::set_fwd(double vel) {
    robot_control::set_twist(vel, 0);
}

void manual::release() {
    robot_control::set_twist(0, 0);
}

void manual::set_ang(double vel) {
    robot_control::set_twist(0, vel);
}

const cv::Mat& sensors::get_image() {
    if ( !robot_control::latest_image ) {
        static cv::Mat empty;
        return empty;
    }

    return robot_control::latest_image->image;
}

