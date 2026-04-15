#include "robot.hpp"

#include <cstdlib> // To run the python file
#include <chrono>  // To take the time

#include "phase_2_slope.cpp"
#include "corridor_navigator.cpp"

///////////// //////////////////////////////////////////////////////////
/// POSES /// //////////////////////////////////////////////////////////
///////////// //////////////////////////////////////////////////////////

/* Level 1 coordinates */
Pose spawn(0, 0, 0);
Pose before_cone(6.12, -1.13, -0.67);
Pose after_cone(8.93, -2.98, 0);

/* Level 2 coordinates - Section 4.5 */
Pose map_changed(47.81, -20.36, 0.17, 0.99);
// Pose level2_first(47.30, -17.07, -0.99, 0.17);
Pose top_cone(42.22, -15.85, 0.77, 0.63);
Pose bottom_cone(32.92, -18.88, 0.78, 0.63);
// Pose after_bottom_cone();

///////////// //////////////////////////////////////////////////////////
/// UTILS /// //////////////////////////////////////////////////////////
///////////// //////////////////////////////////////////////////////////
/* Monitor duration of each section */
class Section {
    using clock = std::chrono::high_resolution_clock;

    std::string header;
    clock::time_point started;

    public:
        Section(const std::string& h);
        ~Section();
};

//////////// ///////////////////////////////////////////////////////////
/// MAIN /// ///////////////////////////////////////////////////////////
//////////// ///////////////////////////////////////////////////////////
int main(int argc, char** argv) {
    ros::init(argc, argv, "testing");

    ros::NodeHandle nh;

    float skip_to, end_at;
    nh.param("skip_to", skip_to, 2.0f);
    nh.param("end_at", end_at, 6.0f);

    Robot robot;

    /// :::::::::::::::::::::::::::::::::::::::::
    /// ::: SECTION 0 - Initial Pose Estimate :::
    if ( skip_to <= 0 ) {
        robot.set_estimate(spawn);
    }

    /// ::::::::::::::::::::::::::::::::
    /// ::: SECTION 1 - Box Counting :::
    if ( skip_to <= 1 ) {
        Section section("1) Box counting");
        
        // Implemented in "scripts/box_counter.py"
        if ( int err = system("rosrun me5413_group9 box_counter.py") ) {
            ROS_ERROR("Section 1 script failed w/ error code %d!", err);
        }
    }

    /// @todo - Read box target
    if ( end_at <= 1 )
        return 0;

    /// ::::::::::::::::::::::::::::::::
    /// ::: SECTION 2 - Level 1 cone :::

    if ( skip_to <= 2 ) {
        Section section("2) Passing the cone");
        
        robot.move_to(before_cone);
        robot.clear_cone();
        robot.move_to(after_cone);
    }

    if ( end_at <= 2 )
        return 0;

    /// ::::::::::::::::::::::::::::
    /// ::: SECTION 3 - The ramp :::
    if ( skip_to <= 3 ) {
        Section section("3) The ramp");

        // Implemented in "src/phase_2_slope.cpp"
        JackalPotentialField().run();
    }

    if ( end_at <= 3 )
        return 0;

    /// ::::::::::::::::::::::::::::::::::::::
    /// ::: SECTION 4 pt. i - The corridor :::
    if ( skip_to <= 4 ) {
        Section section("4-i) The corridor");

        // Implemented in "src/corridor_navigator.cpp"
        OpeningTracker().run();
    }

    if ( end_at <= 4 )
        return 0;

    /// :::::::::::::::::::::::::::::::::::::
    /// ::: SECTION 4 pt. ii - Change map :::
    if ( skip_to <= 4.2 ) {
        Section section("4-ii) The possible cone");

        robot.change_map("me5413_world", "/maps/lvl2_improved.yaml");
    }

    if ( end_at <= 4.2 )
        return 0;

    /// ::::::::::::::::::::::::::::::::::::::::
    /// ::: SECTION 4 pt. iii - The 2nd Cone :::
    if ( skip_to <= 4.3 ) {
        robot.set_estimate(map_changed);
        robot.move_to(map_changed.flip());
        
        // Intermediary to help localize
        // robot.move_to(level2_first);

        // Check top position - nearest room 3
        robot.move_to(top_cone);
        if ( 1 /* robot.facing_cone() */ ) {
            robot.move_to(bottom_cone);
            // robot.move_to(after_bottom_cone);
        }
    }
}






///////////// //////////////////////////////////////////////////////////
/// UTILS /// //////////////////////////////////////////////////////////
///////////// //////////////////////////////////////////////////////////
/* Some nice formatting */
const char* bold = "\033[1m\033[34m";
const char* reset = "\033[0m";

// Start a timer, and print the section header
Section::Section(const std::string& h): started(clock::now()), header(h) {
    const std::string wing(12, '~');

    ROS_INFO(
        "\n %s %s%s%s %s",
        wing.c_str(),
        bold,
        header.c_str(),
        reset,
        wing.c_str()
    );
}

// End the timer, and print the duration
Section::~Section() {
    auto ended = clock::now();
    int seconds = std::chrono::duration_cast<std::chrono::seconds>(ended - started).count(); 
    ROS_INFO(
        "Section '%s' duration: %s%d:%d%s",
        header.c_str(),
        bold,
        seconds / 60,
        seconds % 60,
        reset
    );
}

// /* Final Room Movement Handling */
// // Go to the 2 poses in the path for a given room
// ros::Time go_to_room(Robot& robot, int room) {
//     const std::pair<Pose, Pose>& path = room_paths[room];
//     robot.move_to(path.first);
//     ros::Time cylinder_time = robot.safe_cylinder_timing();
//     ros::Time::sleepUntil(cylinder_time);
//     robot.move_to(path.second);
//     return cylinder_time;
// }

// // Go in reverse to the 2 poses in the path for a given room
// void exit_room(Robot& robot, int room, ros::Time t) {
//     const std::pair<Pose, Pose>& path = room_paths[room];
//     robot.move_to(path.second.flip());
//     ros::Time::sleepUntil(robot.next_cylinder_timing(t));
//     robot.move_to(path.first.flip());
// }