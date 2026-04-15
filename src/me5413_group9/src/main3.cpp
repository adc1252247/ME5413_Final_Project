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
Pose level2_first(46.8, -17.9, -0.99, 0.14);
Pose top_cone(42.22, -15.85, 0.77, 0.63);
Pose bottom_cone(32.92, -18.88, 0.78, 0.63);
Pose after_bottom_cone(31.7, -16.5, 0.82, 0.57);

/* Level 2 final rooms */
bool go_to_room(Robot& robot, int room);

bool exit_room(Robot& robot);

Pose peek_room4(38, -6, 0.13, 0.99);
Pose peek_room2(35, -7, -0.97, 0.22);

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

    double skip_to, end_at, only = -1.0;
    nh.param("only", only, -1.0);
    nh.param("skip_to", skip_to, 2.0);
    nh.param("end_at", end_at, 6.0);

    if ( only > 0 ) { 
        skip_to = only; 
        end_at = only; 
        ROS_WARN("Only running stage %.1f", only);
    }

    bool visit_both_cones;
    nh.param("visit_both_cones", visit_both_cones, false);

    Robot robot;

    /// :::::::::::::::::::::::::::::::::::::::::
    /// ::: SECTION 0 - Initial Pose Estimate :::
    if ( skip_to <= 0.0 ) {
        robot.set_estimate(spawn);
        ros::Duration(2.0).sleep(); // Just in case...
    }

    /// ::::::::::::::::::::::::::::::::
    /// ::: SECTION 1 - Box Counting :::
    if ( skip_to <= 1.0 ) {
        Section section("1) Box counting");
        
        // Implemented in "scripts/box_counter.py"
        if ( int err = system("rosrun me5413_group9 box_counter.py") ) {
            ROS_ERROR("Section 1 script failed w/ error code %d!", err);
            return err; // Propagate error
        }
    }

    /// @todo - Read box target
    if ( end_at <= 1 )
        return 0;

    /// ::::::::::::::::::::::::::::::::
    /// ::: SECTION 2 - Level 1 cone :::

    if ( skip_to <= 2.0 ) {
        Section section("2) Passing the cone");
        
        robot.move_to(before_cone);
        robot.clear_cone();
        robot.move_to(after_cone);
    }

    if ( end_at <= 2.0 )
        return 0;

    /// ::::::::::::::::::::::::::::
    /// ::: SECTION 3 - The ramp :::
    if ( skip_to <= 3 ) {
        Section section("3) The ramp");

        // Implemented in "src/phase_2_slope.cpp"
        JackalPotentialField().run();
    }

    if ( end_at <= 3.0 )
        return 0;

    /// ::::::::::::::::::::::::::::::::::::::
    /// ::: SECTION 4 pt. i - The corridor :::
    if ( skip_to <= 4.1 ) {
        Section section("4-i) The corridor");

        // Implemented in "src/corridor_navigator.cpp"
        OpeningTracker().run();
    }

    if ( end_at <= 4.1 )
        return 0;

    /// :::::::::::::::::::::::::::::::::::::
    /// ::: SECTION 4 pt. ii - Change map :::
    if ( skip_to <= 4.2 ) {
        Section section("4-ii) Map change");

        robot.change_map("me5413_world", "/maps/lvl2_improved.yaml");
    }

    if ( end_at <= 4.2 )
        return 0;

    /// :::::::::::::::::::::::::::::::::::::::::
    /// ::: SECTION 4 pt. iii - Pose estimate :::
    if ( skip_to <= 4.3 ) {
        Section section("4-iii) Pose estimate");

        robot.set_estimate(map_changed, Covariance(0.01, 0.01, 0.02));
    }

    if ( end_at <= 4.3 )
        return 0;

    /// :::::::::::::::::::::::::::::::::::::::
    /// ::: SECTION 4 pt. iv - The 2nd Cone :::
    if ( skip_to <= 4.4 ) {
        Section section("4-iv) Getting past the cones");
        // robot.move_to(map_changed.flip());
        
        // Intermediary to help localize
        // robot.move_to(level2_first);

        // Check top position - nearest room 3
        robot.move_to(top_cone);

        if ( robot.detect_cone() ) {
            ROS_INFO("Cone detected at 'TOP' position");

            robot.move_to(bottom_cone);
            // robot.move_to(after_bottom_cone);
            ROS_ERROR("Not implemented yet - after_bottom_cone!");
        }

        // For testing
        else if ( visit_both_cones ) {
            robot.move_to(bottom_cone);

            if ( robot.detect_cone() )
                ROS_INFO("Cone detected at 'BOTTOM' position");
            else
                ROS_ERROR("Cone detection failed!");

            robot.move_to(top_cone);
        }
    }

    if ( end_at <= 4.4 )
        return 0;

    /// :::::::::::::::::::::::::::::
    /// ::: SECTION 5 - The Rooms :::
    // robot.init_matching();
    int target_room = 3;

    go_to_room(robot, 3);
    if ( robot.detect_target_box() ) {
        ROS_INFO("Success!");
        return 0;
    }

    robot.move_to(peek_room4);
    if ( robot.detect_target_box() )
        target_room = 4;
    
    if ( target_room == 3 ) {
        robot.move_to(peek_room2);
        if ( robot.detect_target_box() )
            target_room = 2;
        else 
            target_room = 1;
    }

    exit_room(robot);
    go_to_room(robot, target_room);
    
    if ( robot.detect_target_box() ) {
        ROS_INFO("Success!");
        return 0;
    }

    else {
        ROS_ERROR(
            "Failed - wanted box %d, found box %d, in room %d",
            robot.target_box(),
            robot.box_found(),
            target_room
        );
        return -1;
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

/* Utilities ... */
int current_room = 0;
ros::Time cylinder_time;

using Path = std::pair<Pose, Pose>;
std::vector<Path> paths = {
    Path(
        Pose(28.9, -14, 0.8, 0.6), // Facing 1
        Pose(29.3, -15.7, 0.75, 0.65) // In 1
    ),
    Path(
        Pose(33.5, -13.6, 0.8, 0.6), // Facing 2
        Pose(32.5, -9.8, 0.77, 0.64) // In 2
    ),
    Path(
        Pose(38.2, -11.8, 0.8, 0.6), // Facing 3
        Pose(37.3, -8.5, 0.83, 0.55) // In 3
    ),
    Path(
        Pose(43, -9, 0.75, 0.65), // Facing 4
        Pose(42, -6, 0.8, 0.6) // In 4
    )
};

bool go_to_room(Robot& robot, int room) {
    const Path& path = paths[room - 1];

    robot.move_to(path.first);
    cylinder_time = robot.wait_cylinder();
    robot.move_to(path.second);
    current_room = room;

    return true;
}

bool exit_room(Robot& robot) {
    const Path& path = paths[current_room - 1];

    robot.move_to(path.second.flip());
    robot.wait_next_cylinder(cylinder_time);
    robot.move_to(path.first.flip());
    current_room = 0;

    return true;
}