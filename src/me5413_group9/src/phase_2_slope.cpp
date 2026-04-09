#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

enum class StopTrigger {
    LIDAR,      // Stop when front obstacle <= value
    CALCULATED, // Stop when integrated distance >= value
    SLOPE_END   // Stop when pitch angle returns to flat (NEW)
};

struct MoveInstruction {
    std::string type;
    StopTrigger trigger;
    double value;       // Distance, Degrees, or Pitch Threshold
    double speed;
};

class SlopeRunner {
private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber laser_sub_;
    
    double target_yaw_;
    double current_imu_yaw_;
    double current_imu_pitch_; // Track pitch for slope detection
    double current_front_dist_; 
    bool imu_ready_ = false;
    bool laser_ready_ = false;
    
    double Kp = 1.8; 
    double Kd = 0.2;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // Extract Yaw for heading
        current_imu_yaw_ = tf2::getYaw(msg->orientation);
        
        // Extract Pitch for slope detection
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_imu_pitch_ = pitch;

        imu_ready_ = true;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        int num_beams = msg->ranges.size();
        int center_idx = num_beams / 2;
        double min_f = msg->range_max;
        for (int i = center_idx - 2; i <= center_idx + 2; ++i) {
            if (i >= 0 && i < num_beams) {
                float r = msg->ranges[i];
                if (r > 0.3 && r < msg->range_max) min_f = std::min(min_f, (double)r);
            }
        }
        current_front_dist_ = min_f;
        laser_ready_ = true;
    }

public:
    SlopeRunner() {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        imu_sub_ = nh_.subscribe("/imu/data", 10, &SlopeRunner::imuCallback, this);
        laser_sub_ = nh_.subscribe("/front/scan", 10, &SlopeRunner::laserCallback, this);
        
        ROS_INFO("Waiting for Sensors...");
        while (ros::ok() && (!imu_ready_ || !laser_ready_)) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        target_yaw_ = current_imu_yaw_;
    }

    void executeMove(const MoveInstruction& inst) {
        ros::Rate loop_rate(20);
        geometry_msgs::Twist cmd;
        double dt = 0.05;
        double previous_error = 0.0;
        double distance_integrated = 0.0;
        ros::Time start_time = ros::Time::now();

        // Trapezoidal Params
        const double accel_dist = 0.5;
        const double decel_dist = 0.8;
        const double min_speed = 0.2;

        if (inst.type == "ROTATE") {
            target_yaw_ += (inst.value * M_PI / 180.0);
            while (target_yaw_ > M_PI) target_yaw_ -= 2.0 * M_PI;
            while (target_yaw_ < -M_PI) target_yaw_ += 2.0 * M_PI;
        }

        while (ros::ok()) {
            ros::spinOnce(); 

            // --- Heading PID ---
            double error = target_yaw_ - current_imu_yaw_;
            while (error > M_PI) error -= 2.0 * M_PI;
            while (error < -M_PI) error += 2.0 * M_PI;
            double derivative = (error - previous_error) / dt;
            double turn_correction = (Kp * error) + (Kd * derivative);
            previous_error = error;
            turn_correction = std::max(-1.0, std::min(1.0, turn_correction));

            if (inst.type == "FORWARD") {
                distance_integrated += std::abs(inst.speed) * dt;
                double dist_left = 99.9; // Default large
                bool should_break = false;

                // Stop Triggers
                if (inst.trigger == StopTrigger::LIDAR) {
                    dist_left = current_front_dist_ - inst.value;
                    if (current_front_dist_ <= inst.value) should_break = true;
                } 
                else if (inst.trigger == StopTrigger::CALCULATED) {
                    dist_left = inst.value - distance_integrated;
                    if (distance_integrated >= inst.value) should_break = true;
                }
                else if (inst.trigger == StopTrigger::SLOPE_END) {
                    // Logic: Stop if Pitch is near zero (less than ~1 degree)
                    // We also ensure we've moved at least 1m to avoid stopping at the start
                    double pitch_deg = std::abs(current_imu_pitch_ * 180.0 / M_PI);
                    if (distance_integrated > 1.0 && pitch_deg < inst.value) should_break = true;
                }

                if (should_break) break;

                // Velocity Profile
                double current_v = inst.speed;
                if (distance_integrated < accel_dist) {
                    current_v = min_speed + (inst.speed - min_speed) * (distance_integrated / accel_dist);
                }
                if (dist_left < decel_dist && inst.trigger != StopTrigger::SLOPE_END) {
                    double ramp_down_v = min_speed + (inst.speed - min_speed) * (dist_left / decel_dist);
                    current_v = std::min(current_v, ramp_down_v);
                }
                current_v = std::max(min_speed, current_v);

                cmd.linear.x = current_v;
                cmd.angular.z = turn_correction;

                if ((ros::Time::now() - start_time).toSec() > 40.0) break;
            } 
            else { // ROTATE
                cmd.linear.x = 0.0;
                cmd.angular.z = turn_correction; 
                if (std::abs(error) < 0.02) break; 
            }

            vel_pub_.publish(cmd);
            loop_rate.sleep();
        }
        
        cmd.linear.x = 0; cmd.angular.z = 0;
        vel_pub_.publish(cmd);
        ros::Duration(0.8).sleep(); 
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "phase_2_slope_final");
    SlopeRunner runner;

    std::vector<MoveInstruction> mission = {
        {"FORWARD", StopTrigger::CALCULATED, 5.0, 0.8},   
        {"FORWARD", StopTrigger::LIDAR,      1.8, 0.8},  
        {"ROTATE",  StopTrigger::CALCULATED, -45.0, 0.5},
        {"FORWARD", StopTrigger::CALCULATED, 2.65, 0.8},
        {"ROTATE",  StopTrigger::CALCULATED, 45.0, 0.5},
        {"FORWARD", StopTrigger::LIDAR,      1.5, 0.8},
        {"ROTATE",  StopTrigger::CALCULATED, 45.0, 0.3},
        {"FORWARD", StopTrigger::CALCULATED, 2.65, 0.8},
        {"ROTATE",  StopTrigger::CALCULATED, -45.0, 0.5},
        
        // --- UPDATED FINAL STEP ---
        // trigger: SLOPE_END
        // value: 1.5 (Stop when pitch is less than 1.5 degrees)
        {"FORWARD", StopTrigger::SLOPE_END, 1.5, 0.7}
    };

    for (const auto& step : mission) { runner.executeMove(step); }
    return 0;
}