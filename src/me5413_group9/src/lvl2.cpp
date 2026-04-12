/**
 * 
 * IMPORTANT: Run these commands BEFORE using RVIZ - otherwise you miss them
 * To get initial pose estimate:
 *   rostopic echo /initialpose
 * 
 * To get map travel positions:
 *   rostopic echo /move_base_simple/goal
 * 
 * Order:
 * 1) world.launch
 * 2) navigation.launch w/ lvl1_gmapping
 * 3) rosrun me5413_group9 main
 * 4) close navigation.launch
 * 5) reopen navigation.launch w/ lvl2_gmapping (now starting at level 2)
 * 6) rostopic echo /initialpose
 * 7) Set initial pose estimate
 * 8) rostopic echo /move_base_simple/goal
 * 9) Set each target
 */


#include "robot_control.hpp" // include/robot_control.hpp


int main(int argc, char** argv) {
    ros::init(argc, argv, "group9_lvl2_dev");
    ros::NodeHandle nh;

    RobotMover mover("move_base", "map", 0.5);

    /// ::: Start Level 2 Code... :::
    set_amcl_estimate(38.801937103271484, 25.503082275390625, 0.8795368586003314, 0.4758307623130945);
    ros::spinOnce();

    // Turned around after first position
    mover.new_target(36.76942825317383, 23.973785400390625, -0.4538629410501477, 0.891071507086446);
}