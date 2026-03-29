/**
 * @brief A selection of functions in order to use OpenCV matching.
 * 
 * @code
 * // Assume good view of a box - box fairly centered
 * using namespace matching;
 * 
 * cv::Mat image = robot::get_image();
 * 
 * MatchedBox box = detect(image);
 * 
 * if ( box.is_match() ) {
 *   cv::Mat annotated = image;
 *   draw(annotated, box);
 *   cv::imshow("matched_box", annotated);
 * }
 * @endcode
 */
#ifndef TEMPLATE_MATCHING_H_
#define TEMPLATE_MATCHING_H_

#include <string>
#include <vector>

#include "opencv2/core.hpp"

namespace matching {
    /// @brief Results of the box matching algorithm
    struct MatchedBox {
        /// @brief Number on side of box in range [1, 9], 0 if no match
        int box_id;

        /// @brief Homography, defining the scale and orientation of match
        cv::Mat homography;

        /// @brief Format homography in string to be able to print
        std::string get_homography_string() const;  

        /// @brief Checks that box_id and homography are appropriate
        bool is_match() const;

        /// @brief Get rotation from normal - positive is counter-clockwise
        /// If the detected surface is head-on, this value is 0
        double get_angle() const;

        /// @brief Angle in degrees
        double get_angle_deg() const;

        /// @brief Get the pixel height at the middle of the matched box surface
        /// @param image The image of the scene
        double get_height(const cv::Mat& image) const;
    };

    // ================= :::::::::::::::::::::::::::::::::::::::::::::::
    // === Functions === :::::::::::::::::::::::::::::::::::::::::::::::
    // ================= :::::::::::::::::::::::::::::::::::::::::::::::
    /// @brief Initialize the feature matching with template images
    /// @throws std::runtime_error if templates cannot be loaded
    void init_matching();

    /// @brief Detect boxes in image
    /// @param image Image to detect boxes in
    /// @returns The box that matched - check using returned .is_match
    /// @throws std::runtime_error if init_matching() not already called
    /// @throws std::invalid_argument if image is empty
    MatchedBox detect(const cv::Mat& image);

    /// @brief Draws outline of detected box face
    /// @param image (in/out) The image to draw onto
    /// @param match The matched box to draw/label onto image
    /// @returns Whether match is_valid, also whether anything was drawn
    /// @throws std::invalid_argument if image is empty
    /// @note Does not draw anything if match.is_match() == false
    bool draw(cv::Mat& image, const MatchedBox& match);

    // /// @brief Returns a simulation of the rotation of image using homography transform
    // /// @param image Image to rotate
    // /// @param angle_deg Angle [degrees] to rotate counter-clockwise
    // /// @param focal_length Focal length for distortion of image
    // cv::Mat simulate_rotation(const cv::Mat& image, double angle_deg, double focal_length);

    // ============== ::::::::::::::::::::::::::::::::::::::::::::::::::
    // === Extras === ::::::::::::::::::::::::::::::::::::::::::::::::::
    // ============== ::::::::::::::::::::::::::::::::::::::::::::::::::
    /// @brief Returns the template image for a box
    const cv::Mat& get_box_template(int box_id);

    /// @note draw_features Intended for testing purposes only
    /// @brief Draws matched features in both images
    /// @param image The image to detect within
    /// @param box_id The box id to show the comparison for
    /// @returns Side-by-side box template and input images w/ features matched
    /// @throws std::invalid_argument if image is empty or box_id not in [1, 9]
    cv::Mat draw_features(const cv::Mat& image, int box_id);

    /// @note draw_feature_template Intended for testing purposes only
    /// @brief Draws an image of all features in the box template
    /// @param box_id The box id to show the features within
    /// @throws std::invalid_argument if box_id not in [1. 9]
    cv::Mat draw_feature_template(int box_id);
}

#endif // TEMPLATE_MATCHING_H_