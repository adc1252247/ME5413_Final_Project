/**
 * Strategies:
 *   - Room 1: Go to each corner using move_base, stop and face box to detect when new one is found
 *   - Outdoors: Wall following
 *   - Room 2: Go to each partition to check for obstacle, have stop on either side of cylinder too
 * 
 * Intended for me5413_world navigation.launch w/ me5413_group9's lvl1_gmapping
 */
#include <cstdlib>

#include "robot_control.hpp" // include/robot_control.hpp
#include "phase_2_slope.cpp"
#include "corridor_navigator.cpp"

const bool running_lvl2 = false;

int main(int argc, char** argv) {
    ros::init(argc, argv, "group9_main");
    ros::NodeHandle nh;

    ROS_INFO("Starting first script");
    {
        int ret = system("rosrun me5413_group9 box_counter.py");

        if ( ret == 0 ) {
            ROS_INFO("Python script succeeded!");
        }
        else {
            ROS_ERROR("Python script failed!");
            exit(-1);
        }

        exit(0);
    }

    RobotMover mover("move_base", "map", 0.5);
    
    /// ::: Placeholder for Level 1 - just exit to outside :::
    // Initial pose for level 1
    set_amcl_estimate(2.386495590209961, 5.6931610107421875, -0.4612403984795658, 0.8872752080445003);
    ros::spinOnce();

    /// Go to reliable start-position for Brian's portion
    ROS_INFO("Exiting Level 1...");
    mover.move_to(4.537387847900391, 2.528341770172119, -0.4747749438723549, 0.8801072393015537);
    // mover.move_to(3.5218276977539062, -4.5353779792785645, -0.47405048895524854, 0.8804976626438538);
    mover.move_to(4.307516098022461, -5.279576301574707, -0.4948095134644987, 0.8690014645471124);
    // if ( !mover.wait(ros::Duration(25.0)) ) {
    //     ROS_INFO("Did not complete!");
    //     mover.cancel();
    // }

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

    /// ::: Reset map to level 2
    {
        MapChanger mc;
        mc.change_map("me5413_world", "/maps/lvl2_gmapping.yaml");

        ROS_INFO("Zeroing in on level 2 map!");

        set_amcl_estimate(38.801937103271484, 25.503082275390625, 0.8795368586003314, 0.4758307623130945);
        ros::spinOnce();

        mover.move_to(36.76942825317383, 23.973785400390625, -0.4538629410501477, 0.891071507086446);
        // mover.wait(ros::Duration(20.0));
    }
}
