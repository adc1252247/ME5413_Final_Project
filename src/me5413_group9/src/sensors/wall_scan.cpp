/**
 * @brief Filter the 3D LIDAR to provide LaserScan topic filtering only walls.
 */
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/master.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

/// @brief Handles subscription and publishing logic
class WallFilterer {
    public:
        /// @brief The important settings for filtering the points
        /// @note Avoid multiples of (2 * M_PI) for angles - the logic
        ///       used will interpret for example [0, 360] degrees
        ///       which is [0, 2 * M_PI] radians as having 0 angle width
        struct Settings {
            float angle_min         = -M_PI;
            float angle_max         = M_PI;
            unsigned int increments = 360;

            float range_min = 0.05;
            float range_max = 30.0;

            float min_height = 0.8; // Height of one of the cubes
            float max_height = 1.8; // Height of door frames
        };

    private:
        Settings settings;

        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher  pub;

    public:
        /// @brief Angle from angle_min to desired angle
        /// @param angle Angle to compute offset for
        float local_angle(float angle) const {
            angle = fmod(angle - settings.angle_min, 2 * M_PI);
            if ( angle < 0 )    
                angle += 2 * M_PI;
            return angle;
        }

        /// @brief Compute the increments desired between angle_min and angle_max in settings.
        float get_angle_increments() const {
            return (settings.angle_max - settings.angle_min) / static_cast<float>(settings.increments);
        }

        /// @brief Filter for greatest distance above min_height
        /// @param (out) ranges Where to put the filtered ranges
        /// @param data Data field from message, formatted as:
        ///   0-3:   x <float>
        ///   4-7:   y <float>
        ///   8-11:  z <float>
        ///   12-15: (ignored)
        ///   16-19: ring <float>
        ///   19-21: (ignored)
        /// @warning Does not implement safety checks, and assumes vector is contiguous
        void filter_ranges(std::vector<float>& ranges, const std::vector<uint8_t>& data) const {
            ranges = std::vector<float>(settings.increments, std::numeric_limits<float>::infinity());

            float angle_increment = get_angle_increments();

            for ( size_t i = 0; i < data.size() / 22; i++ ) {
                float z = *(float*)(&data[i * 22 + 8]);

                // Too high to matter
                if ( z > settings.max_height )
                    continue;

                // Don't want points below or at 0
                if ( z <= 0.0 )
                    continue;

                float x = *(float*)(&data[i * 22]);
                float y = *(float*)(&data[i * 22 + 4]);

                float angle      = local_angle(atan2(y, x));
                size_t angle_bin = static_cast<size_t>(angle / angle_increment);

                if ( angle_bin >= settings.increments )
                    continue;

                // Can't tell due to obstruction
                if ( z < settings.min_height && std::isinf(ranges[angle_bin]) ) {
                    ranges[angle_bin] = std::numeric_limits<float>::quiet_NaN();
                    continue;
                }
                
                float distance   = sqrt(x*x + y*y);
                float registered = ranges[angle_bin];

                if ( std::isinf(registered) || std::isnan(registered) )
                    ranges[angle_bin] = distance;
                else if ( registered > distance )
                    ranges[angle_bin] = distance;
                // else ;
            }
        }

    public:
        /// @brief Callback per LIDAR scan message received
        /// @param msg LIDAR scan, with point_step 22
        void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
            // Validate msg format is as expected
            if ( msg->point_step != 22 ) {
                ROS_ERROR(
                    "wall_scan on topic '%s' got point_step of %d", 
                    sub.getTopic().c_str(),
                    msg->point_step
                );
                return;
            }

            if ( 22 * msg->width * msg->height < msg->data.size() ) {
                ROS_ERROR(
                    "wall_scan on topic '%s' got malformed PointCloud2 data!",
                    sub.getTopic().c_str()
                );
                return;
            }
            
            // Format message to publish
            sensor_msgs::LaserScan scan;

            scan.header.stamp = msg->header.stamp;
            scan.header.frame_id = msg->header.frame_id;

            scan.angle_min       = settings.angle_min;
            scan.angle_max       = settings.angle_max;
            scan.angle_increment = get_angle_increments();
        
            scan.range_min = settings.range_min;
            scan.range_max = settings.range_max;

            scan.scan_time      = 0.0;
            scan.time_increment = 0.0;

            scan.intensities = std::vector<float>(settings.increments, 0.0);
    
            // Publish filtered scan result
            filter_ranges(scan.ranges, msg->data);

            pub.publish(scan);
        }

    public:
        /// @brief Constructor
        /// @param st Subscribe topic
        /// @param pt Publish topic
        /// @param s  Settings for scan output
        WallFilterer(const std::string& st, const std::string& pt, Settings s) {
            settings = s;
            pub      = nh.advertise<sensor_msgs::LaserScan>(pt, 1);
            sub      = nh.subscribe(st, 1, &WallFilterer::callback, this);
        }

        WallFilterer(const std::string& st, const std::string& pt): WallFilterer(st, pt, Settings()) {}
};

/// @brief Check if a topic is currently registered with roscore
/// @param t Topic to search for
bool topic_occupied(const std::string& t) {
    ros::NodeHandle nh;

    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for ( const auto& topic: topics )
        if ( topic.name == t )
            return true; 
    
    return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_scan");
    ros::NodeHandle nh;

    ROS_INFO("wall_scan running...");

    float loop_rate;

    nh.param<float>("loop_rate", loop_rate, 6);

    std::string lidar_topic;
    std::string output_topic;

    nh.param<std::string>("lidar_topic", lidar_topic, "/mid/points");
    nh.param<std::string>("output_topic", output_topic, "/front/filtered_scan");

    WallFilterer::Settings settings;
    nh.param<float>("min_height", settings.range_max, 30);
    nh.param<float>("min_height", settings.min_height, 0.8);
    nh.param<float>("max_height", settings.max_height, 1.6);

    if ( topic_occupied(output_topic) ) {
        ROS_ERROR(
            "wall_scan can't use topic '%s' - already in use",
            output_topic.c_str()
        );
        return -1;
    }

    WallFilterer filterer(lidar_topic, output_topic, settings);

    /// @todo 
    // ros::spin();
    ros::Rate looper(loop_rate);
    while ( ros::ok() ) {
        ros::spinOnce();
        looper.sleep();
    }
}