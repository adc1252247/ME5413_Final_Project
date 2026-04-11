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
 * Change to: Max height difference of +/-0.05cm from 0.385, max linear distance of 7.2
 */
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "ros/ros.h"
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
    float MAX_RANGE = 5.0;

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
    std::array<std::array<std::pair<float, float>, 16>, 360> scan_layers;
    for ( size_t a = 0; a < 360; a++ ) {
        for ( size_t i = 0; i < 16; i++ ) {
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
        ROS_ERROR("PointCloud2 message 'size' expected %d, received %d!", 22 * msg->width * msg->height, (int)msg->data.size());
        throw std::runtime_error("<PointCloud2>(msg)->data.size() too low!");
    }


    /// ::: 1) Compute x-/y- plane distance to point, and keep z- height    
    size_t max_bin = 0, min_bin = 360;
    for ( size_t i = 0; i < msg->width * msg->height; i++ ) {
        uint16_t ring = *(uint16_t*)(&msg->data[i * msg->point_step + 16]);
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
    const float MAX_DRIVABLE = 10 * 2 * M_PI / 360; // Anything below 10 degrees is drivable
    const float VERTICAL     = 85 * M_PI / 360; // Anything above 85 degrees is vertical

    for ( size_t a = 0; a < 360; a++ ) {
        bool  unblocked = true;

        float last_d    = 0.0;
        float last_z    = -0.2617; // Height of velodyne from ground...
                                   // Short baricades are at heigh 0.19 above the ground

        for ( size_t i = 0; i < 16; i++ ) {
            float curr_d = scan_layers[a][i].first;
            float curr_z = scan_layers[a][i].second;

            if ( std::isnan(curr_d) || std::isnan(curr_z) ) {
                continue;
            }
            // if ( curr_d > MAX_RANGE )
            //     break;

            float delta_d = curr_d - last_d;
            float delta_z = curr_z - last_z;

            float angle = atan2(delta_z, delta_d);

            if ( /* i == 6 && */ a == 0 ) // DEBUG
                ROS_INFO("(i: %d, a: %d) | H: %f | D: %f | A: %f", i, a, curr_z, curr_d, angle); 

            // if ( a != 0 )
            //     continue;
            // ROS_INFO("Step (%d:%d) angle %f x %f -> %f / %f", a, i, curr_d, curr_z, angle, MAX_DRIVABLE);
                        
            // No wall/drop found yet
            if ( unblocked ) {
                // If drop too large, quit iterating
                // Skip first iteration
                if ( !value_between(angle, -MAX_DRIVABLE, MAX_DRIVABLE) ) {
                    ranges[a]      = curr_d;
                    intensities[a] = curr_z;

                    if ( angle < 0 )
                        break;

                    unblocked = false;
                }

                last_d = curr_d;
                last_z = curr_z;
            }
            // Wall/drop already found
            else {
                // Top of wall found
                if ( angle < VERTICAL ) {
                    if ( a != 0 ) // DEBUG
                    break;
                }

                else
                    intensities[a] = curr_z;
            }
        }

    }

    /// ::: 3) Publish LaserScan message
    pub.publish(scan_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "points_to_scan");
    ros::NodeHandle nh;

    std::string points_topic, scan_topic;
    nh.param<std::string>("points_topic", points_topic, "/mid/points");
    nh.param<std::string>("scan_topic", scan_topic, "/front/scan");

    ros::Subscriber sub = nh.subscribe(points_topic, 1, publish_scan);
    pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic, 1);

    ros::spin();
}
