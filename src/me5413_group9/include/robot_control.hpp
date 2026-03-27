/**
 * A set of functions intended to control the velocity of the robot.
 * 
 * Units are metric:
 * - Distances in [meters]
 * - Angles in [radians]
 */
#ifndef GROUP_9_ROBOT_CONTROL_H_
#define GROUP_9_ROBOT_CONTROL_H_

#include "opencv2/core/mat.hpp"

void robot_control_init();

namespace manual {
    bool is_manual();

    void release();

    void set_fwd(double vel);

    void set_ang(double vel);

    void set_arc(double vel, double r);

    double get_fwd();

    double get_ang();

    double get_arc();    
}

namespace sensors {
    const cv::Mat& get_image();

    double get_imu_fwd();
}

#endif // ...