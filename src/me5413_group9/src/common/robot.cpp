// #include <iostream>

#include "robot.hpp"
#include "template_matching.hpp"

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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>

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

Pose Pose::operator+(const Pose& rhs) const {
    double dx = rhs.x * cos(yaw) - rhs.y * sin(yaw);
    double dy = rhs.x * sin(yaw) + rhs.y * cos(yaw);
    return Pose(x + dx, y + dy, yaw + rhs.yaw);
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
    _scan = _nh.subscribe("/front/scan", 1, &Robot::scan_callback, this);
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
        
        // Insufficient to recover - just manually override
        // // If robot gets lost
        // if ( _latest_covariance.bad() ) {
        //     const Covariance& c = _latest_covariance;
        //     ROS_ERROR("<Robot> move_base motion resulted in too much covariance!");
        //     ROS_ERROR("        x: %.2f | y: %.2f | yaw: %.2f", c.x, c.y, c.yaw);

        //     // Retry
        //     if ( retries > 0 ) {
        //         ROS_INFO("<Robot> move_to retrying - resetting estimate | remaining: %d", retries);
        //         // set_estimate(_latest_good_pose, re_est_cov);
        //         set_estimate(_latest_pose, re_est_cov);
        //         return move_to(pose_, retries - 1);
        //     }
        //     else {
        //         ROS_ERROR("<Robot> move_to no retries remaining.");
        //         ROS_ERROR(
        //             "        Current: (x: %.2f | y: %.2f | yaw: %.2f)",
        //             pose().x,
        //             pose().y,
        //             pose().yaw
        //         );
        //         stop();

        //         ROS_ERROR(
        //             "        Target: (x: %.2f | y: %.2f | yaw: %.2f)",
        //             pose_.x,
        //             pose_.y,
        //             pose_.yaw
        //         );
        //         throw std::runtime_error("<Robot> move_to retries went to 0!");
        //     }
        // }

        loop_rate.sleep();
    }

    ros::spinOnce();

    State end_state = _move_base.getState();
    if ( end_state != State::SUCCEEDED ) {
        ROS_ERROR("<Robot> move_base did not succeed!");
        ROS_ERROR("        State (%d): %s", end_state.state_, end_state.getText().c_str());
        
        // if ( retries > 0 ) {
        //     ROS_INFO("Retry available with manual input to terminal...");
        //     std::cout << "\n\nRe-localize, then press 'ENTER'..." << std::endl;
        //     char c;
        //     std::cin >> c;
        //     move_to(pose_, retries - 1);
        // }
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


/* === Level 2 obstacle detection === */
void Robot::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    _ranges = msg->ranges;
    _angle_min = msg->angle_min;
    _angle_max = msg->angle_max;
    _angle_increment = msg->angle_increment;
    _range_min = msg->range_min;
    _range_max = msg->range_max;
}


std::vector<std::pair<float, float>> Robot::pointify() const {
    std::vector<std::pair<float, float>> points;

    for ( size_t i = 0; i < _ranges.size(); i++ ) {
        float dist = _ranges[i];

        if ( std::isnan(dist) || std::isinf(dist) || dist > _range_max || dist < _range_min )
            continue;

        float angle = _angle_min + _angle_increment * static_cast<float>(i);

        std::pair<float, float> point(cos(angle) * dist, sin(angle) * dist);
        points.push_back(point);
    }

    return points;
}

struct Arc {
    float center_x;
    float center_y;
    float radius;
    std::vector<int> indices;
};

std::vector<Arc> segment_arcs(const std::vector<std::pair<float,float>>& points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& p : points) {
        pcl::PointXYZ pt;
        pt.x = p.first;
        pt.y = p.second;
        pt.z = 0.0f;
        cloud->push_back(pt);
    }

    std::vector<Arc> arcs;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // https://github.com/PointCloudLibrary/pcl/blob/master/sample_consensus/include/pcl/sample_consensus/sac_model_circle3d.h
    // Coefficients:
    // 0: x
    // 1: y
    // 2: z
    // 3: r
    // 4: normal x
    // 5: normal y
    // 6: normal z
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setDistanceThreshold(0.005);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
    
    while (remaining->size() > 5) {
        seg.setInputCloud(remaining);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.empty()) break;
        
        // If too large, ignore
        if ( coefficients->values[3] < 3.0 ) {
            Arc arc;
            arc.center_x = coefficients->values[0];
            arc.center_y = coefficients->values[1];
            arc.radius = coefficients->values[3]; 

            arc.indices = inliers->indices;
            arcs.push_back(arc);
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < remaining->size(); ++i) {
            if (std::find(inliers->indices.begin(), inliers->indices.end(), i) == inliers->indices.end()) {
                temp->push_back(remaining->at(i));
            }
        }
        remaining = temp;
    }
    
    return arcs;
}

bool Robot::detect_cone() const {
    for ( auto a: segment_arcs(pointify()) )
        if ( a.radius < 0.2 ) { // Expected size of cone is 0.148
            float alpha = atan2(a.center_y, a.center_x);
            if ( abs(alpha) < (M_PI / 4) )
                return true;
        }
    return false;
}

Pose Robot::detect_cylinder() const {
    for ( auto a: segment_arcs(pointify()) )
        if ( a.radius > 0.45 && a.radius < 0.55 ) {
            float alpha = atan2(a.center_y, a.center_x);
            return Pose(a.center_x, a.center_y, alpha);
        }
    return Pose(0, 0, 0);
}

ros::Time Robot::wait_cylinder(bool keep_left) {
    ros::Time started = ros::Time::now();

    ros::spinOnce();
    Pose last = detect_cylinder();

    ros::Rate loop_rate(3);
    
    // bool ready = abs(first.yaw) > (M_PI / 4);

    while ( ros::ok() ) {
        loop_rate.sleep();
        ros::spinOnce();
        
        Pose cylinder = detect_cylinder();

        double diff = cylinder.yaw - last.yaw;

        bool appropriate_angle = abs(cylinder.yaw) > 0 && abs(cylinder.yaw) < M_PI / 4;

        // Move when in front, moving leftwards
        if ( appropriate_angle ) {
            if ( keep_left && cylinder.yaw > last.yaw )
                break;

            if ( (!keep_left) && cylinder.yaw < last.yaw )
                break;
        }
        
        // Wait for 45 degree angle && cylinder moving away
        if ( diff > 0.0 && cylinder.yaw > M_PI / 4 )
            break;

        if ( ros::Time::now() > started + ros::Duration(68) ) // Already cycled
            throw std::runtime_error("<Robot> wait_cylinder waited too long!");
    }

    ROS_INFO("Cylinder at: %.2f yaw!", detect_cylinder().yaw);

    return ros::Time::now();
}

void Robot::wait_next_cylinder(ros::Time time) {
    while ( ros::Time::now() > time )
        time += ros::Duration(68);
    
    ros::Time::sleepUntil(time);
}


bool Robot::detect_target_box() {
    return target_box() == box_found();
}

int Robot::target_box() {
    init_detection();

    return _target_box;
}

int Robot::box_found() {
    init_detection();

    return matching::detect(_curr_img).box_id;
}

void Robot::init_detection() {
    if ( _target_box == 0 ) {
        matching::init_matching();
        _target_box = read_target();
    }
}

int Robot::read_target() {
    std::ifstream file("/tmp/rarest_box.txt");

    if ( !file.is_open() ) {
        ROS_ERROR("<Robot> Ensure that the rarest box is stored at '/tmp/rarest_box.txt'");
        throw std::runtime_error("<Robot> Rarest box was not appropriately stored!");
    }

    char digit;
    if ( !file.get(digit) || digit < '1' || digit > '9' ) {
        ROS_ERROR("<Robot> Malformed '/tmp/rarest_box.txt'");
        throw std::runtime_error("<Robot> Malformed rarest box file!");
    }

    return digit - '0';
}