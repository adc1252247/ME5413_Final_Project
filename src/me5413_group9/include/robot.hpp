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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


#include <array>
#include <chrono>
#include <string>
#include <vector>

/// @brief A pose in the map frame
struct Pose {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;

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

        /// @brief Clear the costmap
        void clear_costmap();

        /// @brief Move to a desired location in the map frame
        /// @note Runs spinOnce before returning, to ensure up-to-date getters
        void move_to(Pose pose_);

        /// @brief Publishes the clear cone topic for level 1
        void clear_cone();

        /// @brief Change the map for AMCL-based navigation
        /// @param pkg Catkin package the map is stored within
        /// @param map Path to map .yaml from package root
        void change_map(const std::string& pkg, const std::string& map);

    public: /* === GETTERS === */
        // /// @brief Get the most recent pose of the robot
        // const Pose& pose() const;

    public: /* === CONST METHODS === */
        // /// @brief Detect an obstacle from the scan topic
        // /// Done using circle fitting on the laser scan. 
        // /// Any radius under 0.2m is a cone
        // /// Any radius close to 0.5m is a cylinder
        // std::vector<Level2Obstacle> detect_obstacles() const;

        // /// @brief Checks if a cone is visible AND in +/- 45 degrees from front of robot
        // bool facing_cone() const;

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
        // /// @brief Initialize the robot feature matching
        // void init_matching();

        // /// @brief Get the best matched "template"
        // int best_match(const cv::Mat& image);

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