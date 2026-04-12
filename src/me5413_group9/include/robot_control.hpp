#ifndef ROBOT_CONTROL_H_
#define ROBOT_CONTROL_H_

#include <cmath>
#include <stdexcept>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/LoadMap.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MapChanger {
    private:
        ros::NodeHandle nh;
        ros::ServiceClient client;
    
    public:
        /// @brief Initialize it
        MapChanger(const std::string& name="change_map");

        /// @brief Set map using absolute system path
        /// @throws std::runtime_error if couldn't open
        void change_map(const std::string& path);

        /// @brief Set map using path from package
        /// @throws std::runtime_error if couldn't open
        /// @note path MUST START WITH A '/'
        void change_map(const std::string& pkg, const std::string& path);
};

/// @brief 
class RobotPose {
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        geometry_msgs::PoseWithCovarianceStamped pose;

    public:
        /// @brief Set up subsription to get pose of robot
        RobotPose(const std::string& topic="/amcl_pose");

        /// @brief Populate the pose
        void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

        double get_x() const;

        double get_y() const;

        double get_z() const;

        double get_w() const;

        double get_yaw() const;
};

class RobotMover {
    public:
        RobotPose                    pose;

    private:
        MoveBaseClient               mbc;
        move_base_msgs::MoveBaseGoal goal;
        bool                         unused;

    public:
        /// @brief GoalState enum
        ///   Notable Fields
        /// SUCCEEDED
        /// ACTIVE
        /// PENDING || PREEMPTED
        /// RECALLED || REJECTED || ABORTED || LOST
        using GoalState = actionlib::SimpleClientGoalState;

    public:
        /// @brief Initialize the Mover action client
        /// @param client  Name of client
        /// @param frame   Name of frame
        /// @param timeout Initial connection timeout
        RobotMover(const std::string& client="move_base", const std::string& frame="map", float timeout=0.5);

    public:
        /// @brief Move to a new target pose
        /// @param x X-position in map
        /// @param y Y-position in map
        /// @param z Z-component of quaterion
        /// @param w W-component of quaternion
        void new_target(double x, double y, double z, double w);

        /// @brief Move to a new target pose
        /// @param x X-position in map
        /// @param y Y-position in map
        /// @param a Yaw-angle in map
        void new_target(double x, double y, double a);

        /// @brief Cancel the last move
        void cancel();

        /// @brief Wait for completion, or until timeout
        bool wait(ros::Duration timeout);

        /// @brief Wait for completion
        bool long_wait();

    public:
        /// @brief Getter X-position of target
        double target_x() const;

        /// @brief Getter Y-position of target
        double target_y() const;

        /// @brief Getter Z-component of target quaternion
        double target_z() const;

        /// @brief Getter W-component of target quaterinion
        double target_w() const;

        /// @brief Getter yaw of target
        double target_yaw() const;

        /// @brief Check if move currently in progress
        bool is_active() const;

        /// @brief Getter time of request
        ros::Time requested_time() const;

        /// @brief Distance to target 
        double dist_to_target() const;

        double angle_to_target() const;

        void move_to(double x, double y, double z, double w);
};

class RobotPlan {
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        nav_msgs::Path  path;

    public:
        /// @brief Set up subscription to get path of robot
        RobotPlan(const std::string& topic);

        /// @brief Populate the path
        void callback(const nav_msgs::Path::ConstPtr& msg);

        /// @brief Compute the minimum distance for traveling this path
        /// @param x Current x position
        /// @param y Current y position
        double distance_to_target(double x, double y);

        /// @brief Raw access to the path
        const nav_msgs::Path& get_path() const;
};

class RobotOdometry {
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        nav_msgs::Odometry odom;

    public:
        RobotOdometry(const std::string& topic="/odom");

        void callback(const nav_msgs::Odometry::ConstPtr& msg);

        double get_speed() const;

        double get_vx() const;

        double get_vy() const;

        double get_w() const;
};

/// @brief Replace the map used by AMCL
void replace_amcl_map(const std::string& map_name);

/// @brief Set an initial estimate for the AMCL map localization
void set_amcl_estimate(double x, double y, double z, double w, const std::string& frame_id="map", const std::string& pose_topic="/initialpose");

void clear_cone();


///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
/// IMPLEMENTATION ////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
/// ====================
/// === Map Changing ===
MapChanger::MapChanger(const std::string& name) {
    client = nh.serviceClient<nav_msgs::LoadMap>(name);
    client.waitForExistence();
}

void MapChanger::change_map(const std::string& path) {
    nav_msgs::LoadMap srv;
    srv.request.map_url = path;

    if ( client.call(srv) ) {
        if ( srv.response.result == srv.response.RESULT_SUCCESS ) {
            ROS_INFO("Map changed to: %s", path.c_str());
            return;
        }
        else {
            ROS_ERROR("Failed to call change map. Result code: %d", srv.response.result);
        }
    }
    else {
        ROS_ERROR("Failed to call change_map service");
    }

    throw std::runtime_error("<MapChanger> couldn't change map!");
}

void MapChanger::change_map(const std::string& pkg, const std::string& path) {
    std::string pkg_path = ros::package::getPath(pkg);

    if ( pkg_path.empty() ) {
        ROS_ERROR("Could not find package '%s'", pkg.c_str());
        throw std::runtime_error("<MapChanger> unfindable package!");
    }

    return change_map(pkg_path + path);
}

/// ===========================
/// === Path Planning STUFF ===
RobotMover::RobotMover(const std::string& client, const std::string& frame, float timeout): mbc(client, true), unused(true) {
    if ( !mbc.waitForServer(ros::Duration(timeout)) ) {
        ROS_ERROR("<Mover> move_base action not available on '%s' [%s] (%.2f second timeout)", client.c_str(), frame.c_str(), timeout);
        throw std::runtime_error("<Mover> move_base action server not available!");
    }
    
    goal.target_pose.header.frame_id = frame;
    // goal.target_pose.header.stamp    = ros::Time::now();

    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
}

void RobotMover::new_target(double x, double y, double z, double w){
    if ( mbc.getState() == GoalState::ACTIVE ) {
        ROS_WARN("Overriding move to (%.2f, %.2f)", target_x(), target_y());
        mbc.cancelGoal();
    }

    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    goal.target_pose.pose.orientation.z = z;
    goal.target_pose.pose.orientation.w = w;

    mbc.sendGoal(goal);
}

void RobotMover::new_target(double x, double y, double a){
    tf::Quaternion q = tf::createQuaternionFromYaw(a);
    new_target(x, y, q.getZ(), q.getW());
}

void RobotMover::cancel(){
    if ( mbc.getState() == GoalState::ACTIVE ) {
        mbc.cancelGoal();
    }
}

bool RobotMover::wait(ros::Duration timeout) {
    if ( mbc.getState() == GoalState::ACTIVE
      || mbc.getState() == GoalState::PENDING )
        return mbc.waitForResult(timeout);
    else
        return true;
}

bool RobotMover::long_wait() {
    if ( mbc.getState() == GoalState::ACTIVE 
      || mbc.getState() == GoalState::PENDING )
        return mbc.waitForResult();
    else
        return true;
}


double RobotMover::target_x() const {
    return goal.target_pose.pose.position.x;

}

double RobotMover::target_y() const {
    return goal.target_pose.pose.position.y;

}

double RobotMover::target_z() const {
    return goal.target_pose.pose.orientation.z;
}

double RobotMover::target_w() const {
    return goal.target_pose.pose.orientation.w;

}

double RobotMover::target_yaw() const {
    tf::Quaternion q(0, 0, target_z(), target_w());
    return tf::getYaw(q);
}

bool RobotMover::is_active() const {
    return mbc.getState() == GoalState::ACTIVE;
}

ros::Time RobotMover::requested_time() const {
    return goal.target_pose.header.stamp;
}

double RobotMover::dist_to_target() const {
    double x = target_x() - pose.get_x();
    double y = target_y() - pose.get_y();

    return sqrt(x*x + y*y);
}

double RobotMover::angle_to_target() const {
    return target_yaw() - pose.get_yaw();
}

void RobotMover::move_to(double x, double y, double z, double w) {
    new_target(x, y, z, w);
    ros::Rate loop_rate(10);

    while ( mbc.getState() == GoalState::PENDING ) {
        ROS_INFO("Waiting for move_base to start...");
        loop_rate.sleep();
    }

    while ( ros::ok() ) {
        ros::spinOnce();

        if ( dist_to_target() < 0.03 || abs(angle_to_target()) < 0.01 ) {
            ROS_INFO("Stopped moving by proximity!");
            cancel();
            break;
        }

        else if ( !is_active() ) {
            ROS_INFO("Stopped moving by controller!");
            break;
        }

        loop_rate.sleep();
    }
}


/// ==================
/// === Pose STUFF ===
RobotPose::RobotPose(const std::string& topic) {
    sub = nh.subscribe(topic, 10, &RobotPose::callback, this);
}

void RobotPose::callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    pose = *msg;
}

double RobotPose::get_x() const {
    return pose.pose.pose.position.x;
}

double RobotPose::get_y() const {
    return pose.pose.pose.position.y;
}

double RobotPose::get_z() const {
    return pose.pose.pose.orientation.z;
}

double RobotPose::get_w() const {
    return pose.pose.pose.orientation.w;
}

double RobotPose::get_yaw() const {
    tf::Quaternion q(0, 0, get_z(), get_w());
    return tf::getYaw(q);
}




/// ==================
/// === AMCL STUFF ===
void set_amcl_estimate(double x, double y, double z, double w, const std::string& frame_id, const std::string& pose_topic) {
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1);

    ros::Duration(0.5).sleep();

    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.frame_id = frame_id;
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

// ============
// === Cone ===
void clear_cone() {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/cmd_unblock", 1);
    std_msgs::Bool msg;
    msg.data = true;
    pub.publish(msg);
}
#endif // ROBOT_CONTROL_H_