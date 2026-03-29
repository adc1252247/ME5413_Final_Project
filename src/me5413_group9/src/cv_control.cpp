/**
 * @brief Program to manually control robot, viewing the camera in an OpenCV window.
 */
#include "ros/ros.h"

#include "opencv2/highgui/highgui.hpp"

#include "robot_control.hpp"
#include "template_matching.hpp"

const float LIN_SPEED = 1.0;
const float ANG_SPEED = 0.5;

int main(int argc, char** argv) {
    ros::init(argc, argv, "cv_control");
    matching::init_matching();
    robot_control_init();

    cv::namedWindow("cv_control", cv::WINDOW_AUTOSIZE);

    ros::Rate loop_rate(100);

    while ( ros::ok() ) {
        ros::spinOnce();

        cv::Mat img = sensors::get_image();

        if ( !img.empty() )
            cv::imshow("cv_control", sensors::get_image());

        int key = cv::waitKey(10);

        switch ( key ) {
            case 'w':
                manual::set_fwd(LIN_SPEED);
                break;

            case 's':
                manual::set_fwd(-LIN_SPEED);
                break;

            case 'q':
                manual::set_ang(ANG_SPEED);
                break;

            case 'e':
                manual::set_ang(-ANG_SPEED);
                break;

            case 'x':
                cv::imwrite("cv_control_capture.png", sensors::get_image());

            default:
                manual::release();
        }

        loop_rate.sleep();
    }

    cv::destroyAllWindows();
}