/**
 * @brief A selection of functions in order to use OpenCV matching.
 * 
 * @code
 * // Find closest match
 * cv::Mat image = sensors::get_image();
 * auto found = matching::detect(image);
 * 
 * if ( found.size() > 0 ) {
 *   int box_id = 0;
 *   auto height = 0;
 *   
 *   // Find biggest bounding box
 *   for ( auto box: found ) {
 *     if ( box.bounding_box.height > height ) {
 *       box_id = box.box_id;
 *       height = box.bounding_box.height;
 *     }
 *   }
 *   ROS_INFO("Box found: %d", box_id);
 * }
 * @endcode
 */
#include <vector>

#include "opencv2/core.hpp"

namespace matching {
    /// @brief Defines a matched box
    struct Matched {
        int      box_id;       ///< Box ID 1-9
        // cv::Rect bounding_box; ///< Position and size (x, y, width, height)
        cv::Mat  homography;   ///< Homography of template in image
    };

    /// @brief Initialize the feature matching with template images
    /// @throws std::runtime_error if templates cannot be loaded
    void init_matching();

    /// @brief Detect boxes in image
    /// @param image Image to detect boxes in
    /// @returns Vector of detected boxes
    /// @throws std::runtime_error if init_matching() not already called
    std::vector<Matched> detect(const cv::Mat& image);

    /// @brief Draws outline of detected box face
    /// @param image (in/out) The image to draw onto
    /// @param matches The matched boxes to draw/label
    void draw(cv::Mat& image, const std::vector<Matched>& matches);
}