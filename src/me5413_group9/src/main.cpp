/**
 * Strategies:
 *   - Room 1: Go to each corner using move_base, stop and face box to detect when new one is found
 *   - Outdoors: Wall following
 *   - Room 2: Go to each partition to check for obstacle, have stop on either side of cylinder too
 * 
 * Intended for me5413_world navigation.launch w/ me5413_group9's lvl1_gmapping
 */

#include "robot_control.hpp"

#include <std_msgs/Bool.h>

// #include <stdexcept>

// #include <tf/tf.h>

// #include <ros/ros.h>
// #include <actionlib/client/simple_action_client.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <std_srvs/Empty.h>
// #include <nav_msgs/LoadMap.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "group9_main");
    ros::NodeHandle nh;

    RobotMover mover("move_base", "map", 0.5);
    
    set_amcl_estimate(2.386495590209961, 5.6931610107421875, -0.4612403984795658, 0.8872752080445003);
    ros::spinOnce();

    // 1) Go out from start room
    ROS_INFO("Move 1");
    mover.new_target(4.537387847900391, 2.528341770172119, -0.4747749438723549, 0.8801072393015537);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 1");

    // 2) Go to upper left corner
    ROS_INFO("Move 2");
    mover.new_target(18.58062744140625, 10.089950561523438, -0.48640184136444226, 0.8737352280395245);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 2");

    // 3) Go to next corner
    ROS_INFO("Move 3");
    mover.new_target(22.172653198242188, 3.834049701690674, -0.9686993720668705, 0.24823683561722024);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 3");
    
    // 4) Go to first doorway
    ROS_INFO("Move 4");
    mover.new_target(19.34845542907715, 2.258225440979004, -0.49846252459185286, 0.866911247808861);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 4");

    // 5) Pass through first doorway
    ROS_INFO("Move 5");
    mover.new_target(20.73386573791504, 0.28475475311279297, 0.2595057253528059, 0.9657415692146186);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 5");

    // 6) Go to next corner
    ROS_INFO("Move 6");
    mover.new_target(23.432252883911133, 1.7840884923934937, -0.5127406840896237, 0.8585435288203532);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 6");
    
    // 7) Go to upper right corner
    ROS_INFO("Move 7");
    mover.new_target(27.112064361572266, -4.988246440887451, -0.9732384270266234, 0.2297976591672415);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 7");

    // 8) Go to bottom right corner
    ROS_INFO("Move 8");
    mover.new_target(11.74793815612793, -13.960494995117188, 0.8518338779344058, 0.5238120315563894);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 8");

    // 9) Go to next corner
    ROS_INFO("Move 9");
    mover.new_target(8.013261795043945, -6.77471923828125, 0.2607084974466407, 0.9654175673557608);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 9");

    // 10) Go to next doorway
    ROS_INFO("Move 10");
    mover.new_target(11.042487144470215, -5.249324798583984, 0.8783494077275181, 0.47801916064601224);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 10");

    // 11) Gone through next doorway
    ROS_INFO("Move 11");
    mover.new_target(9.44726276397705, -3.10849666595459, -0.9533787390461144, 0.301776374050125);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 11");

    // 12) Final corner
    ROS_INFO("Move 12");
    mover.new_target(6.983809947967529, -4.928114891052246, 0.8487638035378238, 0.5287721681442835);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 12");

    // 13) Before cone
    ROS_INFO("Move 13");
    mover.new_target(5.257331371307373, -1.346045970916748, -0.9674026579037677, 0.25324315880340337);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 13");

    /// @todo - This does not work
    // 14) Trigger cone
    ROS_INFO("Clearing Cone (14)");
    clear_cone();

    /// @todo - This is the wrong position
    // 15) After cone
    ROS_INFO("Move 15");
    mover.new_target(2.9305427074432373, -3.29251766204834, -0.49092752830359954, 0.871200414344322);
    if ( !mover.long_wait() )
        throw std::runtime_error("Failed to make it to 15");

    return 0;
}
