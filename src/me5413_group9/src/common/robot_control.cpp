#include "robot_control.hpp"

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