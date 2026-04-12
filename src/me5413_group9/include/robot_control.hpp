#ifndef ROBOT_CONTROL_H_
#define ROBOT_CONTROL_H_

#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RobotMover {
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

/// @brief 
class RobotPose {
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        geometry_msgs::PoseStamped pose;

    public:
        /// @brief Set up subsription to get pose of robot
        RobotPose(const std::string& topic);

        /// @brief Populate the pose
        void callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        double get_x() const;

        double get_y() const;

        double get_z() const;

        double get_w() const;

        double get_yaw() const;

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

#endif // ROBOT_CONTROL_H_