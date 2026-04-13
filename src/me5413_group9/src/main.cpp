/**
 *   Requires:
 * sudo apt install ros-noetic-pcl-ros libpcl-dev
 * 
 *   Recommendation:
 * catkin_make > /dev/null && rosrun me5413_group9 main
 * rostopic echo /initialpose
 * rostopic echo /move_base_simple/goal
 * 
 * Intended for me5413_world navigation.launch w/ me5413_group9's lvl1_gmapping
 */
#include <cstdlib> // To run the python file
#include <chrono>  // To take the time
#include <fstream>
#include <stdexcept>

#include "robot_control.hpp" // include/robot_control.hpp
#include "phase_2_slope.cpp"
#include "corridor_navigator.cpp"
#include "circle_fitting.cpp"

///////////// //////////////////////////////////////////////////////////
/// POSES /// //////////////////////////////////////////////////////////
///////////// //////////////////////////////////////////////////////////
using Pose = RobotMover::Pose;

Pose on_spawn(0, 0, 0);
// Pose first_move(5.72, 0.45, 1.57);
Pose pose_before_cone(6.12, -1.13, -0.33, 0.94);
// Pose pose_after_cone(8.17, -3.38, 0.00, 0.99);
Pose pose_after_cone(8.93, -2.98, 0.00, 0.99);


Pose pose_after_4i(47.90, -20.16, 0.21, 0.98);
Pose facing_first_corner(47.30, -17.07, -0.99, 0.17);
Pose facing_top_cone(42.22, -15.85, 0.77, 0.63);
Pose facing_bottom_cone(32.92, -18.88, 0.78, 0.63);
// Pose after_bottom_cone();

// Pose before_room_3();
// Pose in_room_3();
// Pose facing_room_4();
// Pose facing_room_2();
// Pose before_room_4();
// Pose in_room_4();
// Pose before_room_3();
// Pose in_room_3();
// Pose before_room_2();
// Pose in_room_2();
// Pose before_room_1();
// Pose in_room_1();

///////////// //////////////////////////////////////////////////////////
/// UTILS /// //////////////////////////////////////////////////////////
///////////// //////////////////////////////////////////////////////////
/// @brief Timer to keep track of elapsed time
class Timer {
    private:
        using clock = std::chrono::high_resolution_clock;

        clock::time_point started;
        clock::time_point section;

    public:
        /// @brief Start the timer
        Timer();

        /// @brief Reset the section timer
        void reset();

        /// @brief Seconds since last reset
        float seconds() const;

        /// @brief Minutes since last reset
        float minutes() const;

        /// @brief Seconds since start (of main)
        float seconds_main() const;

        /// @brief Minutes since start (of main)
        float minutes_main() const;
};

/// @brief Print a message showing clearly what section is running
void section(const std::string& s, Timer& t);

/// @brief Print a message showing time elapsed in a section
void section_end(const std::string& s, const Timer& t);

/// @brief Utility for circle fitting
std::vector<Arc> detect_circles(const sensor_msgs::LaserScan& scan);

/// @brief Get the target box
int read_target_box();

//////////// ///////////////////////////////////////////////////////////
/// MAIN /// ///////////////////////////////////////////////////////////
//////////// ///////////////////////////////////////////////////////////
int main(int argc, char** argv) {
    // Keep track of time elapsed
    Timer timer;

    // Init ROS
    ros::init(argc, argv, "group9_main");
    ros::NodeHandle nh;

    // Move using the move_base
    RobotMover mover("move_base", "map", 0.5);
    ConeLvl1 lvl1_cone;

    /// ::::::::::::::::::::::::::::::::
    /// ::: SECTION 1 - Box Counting :::
    section("Step 1) Box Counter", timer);

    bool skip_1, from_spawn;
    nh.param("skip_1", skip_1, false);
    nh.param("from_spawn", from_spawn, true);

    if ( from_spawn ) 
        set_amcl_estimate(on_spawn);

    // Because it was implemented in Python, we start it
    // as a separate node, and wait for it to end
    if ( skip_1 ) {
        ROS_WARN("Skipping code for step 1, moving directly to exit!");
        // mover.move_to(pose_before_cone);
    }
    else if ( system("rosrun me5413_group9 box_counter.py") != 0 ) {
        ROS_ERROR("Section 1 script failed!");
        exit(-1);
    }

    int box_target = read_target_box();

    section_end("1) Box Counter", timer);
    ROS_INFO("Target box was found to be '%d'!", box_target);

    /// ::::::::::::::::::::::::::::
    /// ::: SECTION 2 - The cone :::
    section("Step 2) De-Coner", timer);

    mover.move_to(pose_before_cone);
    lvl1_cone.unblock();

    mover.move_to(pose_after_cone);
    
    section_end("2) De-Coner", timer);

    /// ::::::::::::::::::::::::::::
    /// ::: SECTION 3 - The ramp :::
    section("Step 3) The Ramp", timer);

    JackalPotentialField().run();

    section_end("3) The Ramp", timer);

    /// ::::::::::::::::::::::::::::::::::::::
    /// ::: SECTION 4 pt. i - The corridor :::
    section("Step 4 pt. i) The Corridor", timer);

    OpeningTracker().run();

    section_end("4 pt. i) The Corridor", timer);

    /// :::::::::::::::::::::::::::::::::::
    /// ::: SECTION 4 pt. ii - The cone :::
    section("Step 4 pt. ii) Another Cone", timer);

    FrontScan front_scan;

    MapChanger mc;
    mc.change_map("me5413_world", "/maps/lvl2_improved.yaml");

    ROS_INFO("Zeroing in on level 2 map!");
    clear_costmap();
    set_amcl_estimate(pose_after_4i);
    ros::Duration(2.0).sleep(); // Give some time to think :)

    // Do a quarter turn
    ros::spinOnce();
    mover.move_to(RobotMover::Pose(mover.pose.get_x(), mover.pose.get_y(), mover.pose.get_yaw() + M_PI / 2));

    ros::Duration(2.0).sleep();
    mover.move_to(facing_first_corner);

    // Now continue
    ROS_INFO("If it ain't zeroed in now, you've failed :(");
    mover.move_to(facing_top_cone);

    std::vector<Arc> arcs = detect_circles(front_scan.get());
    bool blocked = false;
    for ( auto arc: arcs )
        if ( arc.radius < 0.2 ) // Expected size of cone is 0.148
            blocked = true;

    if ( blocked ) {
        ROS_INFO("The cone was detected!");
        mover.move_to(facing_bottom_cone);
        // mover.move_to(after_bottom_cone);
    }
    else {
        ROS_INFO("Seems empty!");
    }

    section_end("4 pt. ii) Another Cone", timer);

    /// ::::::::::::::::::::::::::::::::::
    /// ::: SECTION 5 - The final room :::
    section("5) The final room", timer);

    // mover.move_to(before_room_3);
    // wait_for_opening();
    // mover.move_to(in_room_3);
    // if ( ... detect ... stay ... )

    // mover.move_to(facing_room_4);
    // if ( ... detect ... go to 4 ... )

    // mover.move_to(facing_room_2);
    // if ( ... detect ... go to 2 ... )

    // ... go to 1 ...
    section_end("5) The final room", timer);
}




Timer::Timer(): started(clock::now()), section(clock::now()) {}

void Timer::reset() {
    section = clock::now();
}

float Timer::seconds() const {
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - section).count();
    return static_cast<float>(ms) / 1000.0;
}

float Timer::minutes() const {
    return seconds() / 60.0;
}

float Timer::seconds_main() const {
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - started).count();
    return static_cast<float>(ms) / 1000.0; 
}

float Timer::minutes_main() const {
    return minutes() / 60;
}

void section(const std::string& s, Timer& t) {
    // 2m (dim) 0m (reset) 1m (bold) 34m (blue)
    const char* pre  = "\033[2m========= \033[0m\033[1m\033[34m";
    // 0m (reset) 2m (dim) 0m (reset)
    const char* post = "\033[0m\033[2m =========\033[0m";

    std::string buns(s.size(), '=');
    // 0m (reset) 2m (dim)
    buns = "\033[0m\033[2m" + buns;
    buns = pre + buns + post;
    ROS_INFO("New section (at %d:%d)\n\n%s\n%s%s%s\n%s\n", 
        static_cast<int>(t.minutes_main()),
        static_cast<int>(t.seconds_main()),
        buns.c_str(), 
        pre, 
        s.c_str(), 
        post, 
        buns.c_str()
    );

    t.reset();
}

void section_end(const std::string& s, const Timer& t) {
    // 1m (bold) 34m (blue)
    const char* pre = "\033[1m\033[34m";
    // 0m (reset)
    const char* post = "\033[0m";

    ROS_INFO("Section %s%s%s took %d:%d minutes to complete",
        pre,
        s.c_str(),
        post,
        static_cast<int>(t.minutes()),
        static_cast<int>(t.seconds())
    );
}

std::vector<Arc> detect_circles(const sensor_msgs::LaserScan& scan) {
    ScanConfig cfg;
    cfg.angle_min = scan.angle_min;
    cfg.angle_max = scan.angle_max;
    cfg.angle_increment = scan.angle_increment;
    cfg.range_min = scan.range_min;
    cfg.range_max = scan.range_max;

    auto points = pointify(scan.ranges, cfg);

    return segment_arcs(points);
}

int read_target_box() {
    std::ifstream file("/tmp/rarest_box.txt");

    if ( !file.is_open() ) {
        ROS_ERROR("Ensure that the rarest box is stored at '/tmp/rarest_box.txt'");
        throw std::runtime_error("Rarest box was not appropriately stored!");
    }

    char digit;
    if ( !file.get(digit) || digit < '1' || digit > '9' ) {
        ROS_ERROR("Malformed '/tmp/rarest_box.txt'");
        throw std::runtime_error("Malformed rarest box file!");
    }

    return digit - '0';
}