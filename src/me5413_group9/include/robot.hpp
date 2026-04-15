/**
 * @brief Provides the Robot class for controlling the robot
 * 
 *   Important Methods:
 * set_estimate(pose_)             - Change the initial map pose estimate.
 * move_to(pose_)                  - Go to desired pose in map.
 * clear_cone()                    - Remove level 1 cone.
 * change_map(pkg, path)           - Replace the map used.
 * facing_cone()                   - Check if a level 2 cone is within 
 *                                   front +/- 45 degrees.
 * safe_cylinder_timing()          - What time it is safe to drive 
 *                                   forward given cylinder position.
 * next_cylinder_timing(last_time) - What time it is safe to return 
 *                                   given cylinder position.
 * read_box()                      - What class of box is detected 
 *                                   (best if only 1 is visible).
 * 
 *   Requires:
 * PCL for circle detection
 * OpenCV for feature matching
 * ROS for topic subscription
 * TF for quaternion handling
 */
#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include "template_matching.hpp"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

/// @brief A pose in the map frame
struct Pose {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;

    /// @brief Pose with all values set to 0
    Pose() = default;

    /// @brief Constructor using yaw for angle
    /// @param x_   X coordinate
    /// @param y_   Y coordinate
    /// @param yaw_ Yaw - 0 is aligned with x-positive
    Pose(double x_, double y_, double yaw_);

    /// @brief Constructor using quaternion values for angle
    /// @param x_ X coordinate
    /// @param y_ Y coordinate
    /// @param z_ Z-component of quaternion
    /// @param w_ W-component of quaternion
    Pose(double x_, double y_, double z_, double w_);

    /// @brief Get a pose at the same position with opposite orientation
    Pose flip() const;

    /// @brief Get a new pose, rotated by a certain yaw in radians
    Pose add_yaw(double yaw_) const;

    double get_z() const;

    double get_w() const;

    bool operator==(const Pose& rhs) const;

    bool operator!=(const Pose& rhs) const;

    /// Use translation of poses (in this frame)
    Pose operator+(const Pose& rhs) const;
};

struct Covariance {
    double x   = 0.25;
    double y   = 0.25;
    double yaw = 0.07;

    static constexpr double good_threshold = 0.5; // Good linear cov.
    static constexpr double bad_threshold = 1.5;  // Bad linear cov.

    /// @brief Set with all to 0
    Covariance() = default;

    /// @brief Construct with covariance values
    Covariance(double x_, double y_, double yaw_);

    /// @brief Construct from ROS covariance array
    Covariance(const boost::array<double, 36>& c);

    /// @brief Standard deviation in x
    double std_x() const;

    /// @brief Standard deviation in y
    double std_y() const;

    /// @brief Standard deviation in yaw 
    double std_yaw() const;

    /// @brief Check if this is a good linear covariance
    bool good() const;

    /// @brief Check if this is a bad linear covariance
    bool bad() const;

    /// @brief Get as an ROS-appropriate array
    boost::array<double, 36> array() const;
};

/// @brief For use in detecting the cylinder or cone from level 2
/// @note pose Is the pose RELATIVE TO THE ROBOT
///       - yaw is in range [-M_PI, M_PI]
///       - yaw of 0 means right in front
struct Level2Obstacle {
    bool is_cone;
    Pose pose;
};

/// @brief Main class for controlling robot
class Robot {
    public: /* === CONTROL === */
        /// @brief Set the estimate pose for AMCL
        /// @param pose_ Pose estimate
        void set_estimate(Pose pose_);

        /// @brief Set the estimate pose for AMCL with desired covariance
        /// @param pose_ Pose estimate
        /// @param cov_  Covariance
        void set_estimate(Pose pose_, Covariance cov_);

        /// @brief Clear the costmap
        void clear_costmap();

        /// @brief Move to a desired location in the map frame
        /// @note Runs spinOnce before returning, to ensure up-to-date getters
        /// @param pose_ Pose to move to
        /// @param retries How many retriese to move with acceptable covariance
        void move_to(Pose pose_, int retries = 3);

        /// @brief Publishes the clear cone topic for level 1
        void clear_cone();

        /// @brief Change the map for AMCL-based navigation
        /// @param pkg Catkin package the map is stored within
        /// @param map Path to map .yaml from package root
        void change_map(const std::string& pkg, const std::string& map);

        /// @brief Stop the robot
        void stop();

    public: /* === GETTERS === */
        /// @brief Get the most recent pose of the robot
        const Pose& pose() const;

        /// @brief Get the most recent good pose of the robot
        /// @note This means covariance is 
        const Pose& last_good_pose() const;

        /// @brief Get the most recent covariance of the robot pose
        const Covariance& covariance() const;

    private:
        Pose _latest_pose;
        Pose _latest_good_pose;
        Covariance _latest_covariance;

        void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    public: /* === CONST METHODS === */
        // /// @brief Detect an obstacle from the scan topic
        // /// Done using circle fitting on the laser scan. 
        // /// Any radius under 0.2m is a cone
        // /// Any radius close to 0.5m is a cylinder
        // std::vector<Level2Obstacle> detect_obstacles() const;

        /// @brief Checks if a cone is visible AND in +/- 45 degrees from front of robot
        bool detect_cone() const;

        /// @brief Gets the pose of any cylinder of radius ~0.5m detected
        Pose detect_cylinder() const;

        /// @brief Wait for cylinder to be at a safe distance
        /// @note Assumes perpendicular to motion of cylinder
        ros::Time wait_cylinder();

        /// @brief Wait for cylinder to cycle to same point
        void wait_next_cylinder(ros::Time time);

    private:
        std::vector<float> _ranges;
        float _angle_min;
        float _angle_max;
        float _angle_increment;
        float _range_min;
        float _range_max;

        std::vector<std::pair<float, float>> pointify() const;

        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

        // /// @brief Get time of safe passage based on cylinder position
        // ros::Time safe_cylinder_timing() const;

        // /// @brief Get next viable safe tyming based on last detection
        // ros::Time next_cylinder_timing(ros::Time last_time) const;

        // /// @brief Read the box's class - 0 if failed to read
        // int read_box();

        // /// @brief Shows the best match - same image shown if 
        // cv::Mat draw_read_box();

        // /// @brief Shows features for a given box class
        // cv::Mat draw_template(int box);

        // /// @brief Shows the matched features for a given box class
        // cv::Mat draw_matched_features(int box);

    private:
        void init_detection();

        int read_target();

    public:
        int _target_box = 0;
        cv::Mat _curr_img;

        /// @brief Check if target box is found
        bool detect_target_box();

        /// @brief Get the target box's value
        int target_box();

        /// @brief Get the current box viewed
        int box_found();

    public:
        /// 

        // /// @brief Get the template for a box's class
        // const cv::Mat& get_template(int box);

        // /// @brief Draw the features matched between input image and desired box class
        // cv::Mat draw_matches(const cv::Mat&, int box);

    private: /* === PRIVATE ROS MEMBERS === */
        ros::NodeHandle    _nh;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _move_base;
        ros::Subscriber    _pose;
        ros::Subscriber    _scan;
        ros::Subscriber    _img;
        ros::Publisher     _est;

    public: /* === CONSTRUCTORS/SUB HANDLERS === */
        Robot();

        // /// @brief Add subscriptions needed for level 2 - scan & image
        // void start_level2();
};

#endif // ROBOT_HPP_