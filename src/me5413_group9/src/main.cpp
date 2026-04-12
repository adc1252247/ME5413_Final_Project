/**
 * Strategies:
 *   - Room 1: Go to each corner using move_base, stop and face box to detect when new one is found
 *   - Outdoors: Wall following
 *   - Room 2: Go to each partition to check for obstacle, have stop on either side of cylinder too
 * 
 * Intended for me5413_world navigation.launch w/ me5413_group9's lvl1_gmapping
 */

#include "robot_control.hpp" // include/robot_control.hpp
#include "phase_2_slope.cpp"
#include "corridor_navigator.cpp"

const bool running_lvl2 = false;

int main(int argc, char** argv) {
    ros::init(argc, argv, "group9_main");
    ros::NodeHandle nh;

    RobotMover mover("move_base", "map", 0.5);
    
    /// ::: Placeholder for Level 1 - just exit to outside :::
    // Initial pose for level 1
    set_amcl_estimate(2.386495590209961, 5.6931610107421875, -0.4612403984795658, 0.8872752080445003);
    ros::spinOnce();

    /// Go to reliable start-position for Brian's portion
    ROS_INFO("Exiting Level 1...");
    mover.new_target(3.5218276977539062, -4.5353779792785645, -0.47405048895524854, 0.8804976626438538);
    if ( !mover.wait(ros::Duration(20.0)) ) {
        ROS_INFO("Did not complete!");
        mover.cancel();
    }

    /// ::: Start Brian's Code :::
    {
        ROS_INFO("Field Climber...");
        JackalPotentialField climber;
        climber.run();
    }

    /// ::: Start Cheng En's Code :::
    {
        ROS_INFO("Corridor Navigator");
        OpeningTracker tracker;
        tracker.run();
    }
}
