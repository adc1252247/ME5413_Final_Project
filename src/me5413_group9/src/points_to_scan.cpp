/**
 * Call with `roslaunch me5413_world world.launch JACKAL_LASER_TOPIC:=front/og_scan`
 * 
 * @brief Node for transforming the Velodyne points to a LaserScan usable by gmapping and similar.
 * 
 * @note The intensities of the LaserScan represent the height of the nearest obstacle.
 * 
 * @internal Fields
 *   x: offset 0
 *   y: offset 4
 *   z: offset 8
 *   intensity: offset 12
 *   ring:      offset 16
 *   time:      offset 18
 * 
 * 0.385 +/- 0.005 seems reasonable for Velodyne 16
 * 0.42 +/- 0.01 seems reasonable for Velodyne 32
 * 
 * Change to: Max height difference of +/-0.005cm from 0.385, max linear distance of 7.2
 */
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "ros/ros.h"
#include <ros/master.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


/// ====================== :::::::::::::::::::::::::::::::::::::::::::::
/// === Points To Scan === :::::::::::::::::::::::::::::::::::::::::::::
ros::Publisher pub;

inline bool value_between(float v, float lower, float upper) {
    return lower <= v && upper >= v;
}

void publish_scan(const sensor_msgs::PointCloud2ConstPtr& msg) {
    /// ::: "CONFIG"
    const float MAX_RANGE    = 9.0;
    const float MAX_DRIVABLE = 0.02;
    const float ERR_DISTANCE = 0.005;
    const float FLOOR        = -0.42;

    /// ::: 0.i) Format and publish the LaserScan message
    sensor_msgs::LaserScan scan_msg;
    
    scan_msg.header.stamp    = msg->header.stamp;
    scan_msg.header.frame_id = msg->header.frame_id;
    
    scan_msg.angle_min       = 0;
    scan_msg.angle_increment = 2 * M_PI / 360;
    scan_msg.angle_max       = 2 * M_PI - scan_msg.angle_increment;

    scan_msg.time_increment = 0;
    scan_msg.scan_time = 0;

    scan_msg.range_min = 0.3;
    scan_msg.range_max = MAX_RANGE;   // Max range at which it can resolve the small "door steps"

    // scan_msg.ranges      = ranges;
    // scan_msg.intensities = intensities;

    // L-references to simplify handling
    std::vector<float>& ranges = scan_msg.ranges;
    std::vector<float>& intensities = scan_msg.intensities;

    /// ::: 0.ii) Initialize ranges and intensities to be empty
    ranges.assign(360, std::numeric_limits<float>::infinity());
    intensities.assign(360, std::numeric_limits<float>::infinity());

    /// ::: 0.iii) Initialize the points in case a point not published
    std::array<std::array<std::pair<float, float>, 32>, 360> scan_layers;
    for ( size_t a = 0; a < 360; a++ ) {
        for ( size_t i = 0; i < 32; i++ ) {
            scan_layers[a][i].first  = std::numeric_limits<float>::quiet_NaN();
            scan_layers[a][i].second = std::numeric_limits<float>::quiet_NaN();
        }
    }

    /// ::: 0.iv) Assert correct dimensions
    if ( msg->point_step != 22 ) {
        ROS_ERROR("PointCloud2 message 'point_step' expected 22, received %d!", msg->point_step);
        throw std::runtime_error("<PointCloud2>(msg)->point_step != 22");
    }
    if ( 22 * msg->width * msg->height < msg->data.size() ) {
        ROS_ERROR("PointCloud2 malformed message 'size' expected %d, received %d!", 22 * msg->width * msg->height, (int)msg->data.size());
        throw std::runtime_error("<PointCloud2>(msg)->data.size() too low!");
    }

    /// ::: 1) Compute x-/y- plane distance to point, and keep z- height    
    size_t max_bin = 0, min_bin = 360;
    for ( size_t i = 0; i < msg->width * msg->height; i++ ) {
        uint16_t ring = *(uint16_t*)(&msg->data[i * msg->point_step + 16]);

        if ( ring > 32 ) {
            ROS_ERROR("PointCloud2 too many rings, expected 32, received %d or more!", ring);
            continue; // Skip this ring...
        }

        float x = *(float*)(&msg->data[i * msg->point_step + 0]);
        float y = *(float*)(&msg->data[i * msg->point_step + 4]);
        float z = *(float*)(&msg->data[i * msg->point_step + 8]);

        // ROS_INFO("Got some! %d / %d", i, msg->width * msg->height);

        std::pair<float, float> pt;
        pt.first  = sqrt(x*x + y*y);
        pt.second = z; 

        float  angle_rad = atan2(y, x);
        size_t angle_bin = ((int)(angle_rad * 360 / (2 * M_PI)) + 360) % 360;

        // ROS_INFO("Survived the step!, %zu, %zu", angle_bin, ring);

        if ( angle_bin > max_bin ) max_bin = angle_bin;
        if ( angle_bin < min_bin ) min_bin = angle_bin;

        scan_layers[angle_bin][ring] = pt;
    }

    /// ::: 2) Compute the furthest drivable distance for each 360 angle
    ///        Assumption: All obstacles only made of vertical walls
    for ( size_t a = 0; a < 360; a++ ) {
        float last_d   = 0;
        bool unblocked = true;

        for ( size_t r = 0; r < 32; r++ ) {
            float d = scan_layers[a][r].first;
            float z = scan_layers[a][r].second - FLOOR;


            if ( std::isnan(d) || std::isnan(z) )
                continue;

            if ( d > MAX_RANGE )
                break;

            // Find first obstacle
            if ( unblocked ) {
                if ( fabs(z) > MAX_DRIVABLE ) {
                    // ROS_INFO("(r: %d, a: %d) Blocked! | Z: %.3f | D: %.2f", r, a, z, d);
                    
                    ranges[a]      = d;
                    intensities[a] = z;
                    last_d    = d;
                    unblocked = false;

                    if ( z < 0 )
                        break;
                }
            }

            // Find top of wall
            else {
                if ( fabs(last_d - d) <= ERR_DISTANCE )
                    intensities[a] = z;
                
                else
                    break;
            }
        }

    }

    /// ::: 3) Publish LaserScan message
    pub.publish(scan_msg);
}



/// ============= ::::::::::::::::::::::::::::::::::::::::::::::::::::::
/// === Utils === ::::::::::::::::::::::::::::::::::::::::::::::::::::::
bool topic_in_use(const std::string& t) {
    ros::NodeHandle nh;

    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for ( const auto& topic: topics ) {
        if ( topic.name == t )
            return true;
    }

    return false;
}



/// ============ :::::::::::::::::::::::::::::::::::::::::::::::::::::::
/// === Main === :::::::::::::::::::::::::::::::::::::::::::::::::::::::
int main(int argc, char** argv) {
    ros::init(argc, argv, "points_to_scan");
    ros::NodeHandle nh;

    std::string points_topic, scan_topic;
    nh.param<std::string>("points_topic", points_topic, "/mid/points");
    nh.param<std::string>("scan_topic", scan_topic, "/front/scan");

    if ( topic_in_use(scan_topic) ) {
        ROS_ERROR("Topic '%s' is already used!", scan_topic.c_str());
        throw std::runtime_error("Topic for points_to_scan already in use!");
    }

    ros::Subscriber sub = nh.subscribe(points_topic, 1, publish_scan);
    pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic, 1);

    ros::spin();
}
