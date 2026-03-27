/**
 * @brief Implements functions for OpenCV matching.
 * 
 * See: https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
 */
#include "template_matching.hpp"

#include <iostream> // For debugging

#include <stdexcept>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"

namespace matching {
    // Template contains all the computed info about the image
    struct Template {
        int                       box_id;
        cv::Mat                   image;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat                   descriptors;
    };

    // ================= :::::::::::::::::::::::::::::::::::::::::::::::
    // === Variables === :::::::::::::::::::::::::::::::::::::::::::::::
    // ================= :::::::::::::::::::::::::::::::::::::::::::::::
    const float RATIO_THRESH = 0.7f;
    
    std::vector<Template> box_templates;

    cv::Ptr<cv::Feature2D> detector;

    cv::Ptr<cv::DescriptorMatcher> matcher;

    // =============== :::::::::::::::::::::::::::::::::::::::::::::::::
    // === Helpers === :::::::::::::::::::::::::::::::::::::::::::::::::
    // =============== :::::::::::::::::::::::::::::::::::::::::::::::::
    std::string get_parent_dir(const std::string& path) {
        size_t last_sep = path.rfind('/');

        if ( last_sep == std::string::npos ) 
            return ".";
        
        if ( last_sep == 0 )
            return path;

        if ( last_sep == path.size() - 1 )
            return get_parent_dir(path.substr(0, last_sep)); 
    
        return path.substr(0, last_sep);
    }

    cv::Mat load_box(int i) {
        std::string path = get_parent_dir(get_parent_dir(get_parent_dir(__FILE__)));
        path += "/boxes/img" + std::to_string(i) + ".png";
        
        cv::Mat img = cv::imread(path);
        if ( img.empty() )
            throw std::runtime_error("Failed to load image: " + path);

        return img;
    }

    cv::Mat crop_box_texture(const cv::Mat& full_texture) {
        cv::Rect roi(688, 348, 330, 330);
        return full_texture(roi);
    }

    // ====================== ::::::::::::::::::::::::::::::::::::::::::
    // === Implementation === ::::::::::::::::::::::::::::::::::::::::::
    // ====================== ::::::::::::::::::::::::::::::::::::::::::
    /// In order to initialize matching:
    /// 1) Load feature detector and matcher
    /// 2) Load template images
    /// 3) Compute features for template images
    void init_matching() {
        // 1)
        detector = cv::ORB::create();
        matcher = cv::DescriptorMatcher::create("BruteForce");

        // 2) & 3)
        for ( int i = 1; i < 10; i++ ) {
            Template box;

            box.box_id = i;
            box.image = load_box(i);
            detector->detectAndCompute(box.image, cv::noArray(), box.keypoints, box.descriptors);
            
            box_templates.push_back(box);
        }
    }

    /// In order to detect in image:
    /// 1) Compute features
    /// 2) For each box, match features
    std::vector<Matched> detect(const cv::Mat& image) {
        for ( int i = 1; i < 10; i++ ) {
            Template box = box_templates[i-1];
            
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat                   descriptors;
            detector->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
            
            // Lowe's ratio test
            std::vector<std::vector<cv::DMatch>> all_matches;
            matcher->knnMatch(box.descriptors, descriptors, all_matches, 2);

            std::vector<cv::DMatch> good_matches;
            for ( size_t j = 0; j < all_matches.size(); j++ )
                if ( all_matches[j].size() >=2 && (all_matches[j][0].distance < RATIO_THRESH * all_matches[j][1].distance) )
                    good_matches.push_back(all_matches[j][0]);
        }

        return {};
    }
}

// // =============== :::::::::::::::::::::::::::::::::::::::::::::::::::::
// // === Testing === :::::::::::::::::::::::::::::::::::::::::::::::::::::
// // =============== :::::::::::::::::::::::::::::::::::::::::::::::::::::
// int main() {
//     matching::init_matching();

//     for ( auto t: matching::box_templates ) {
//         std::cout << t.box_id << ": " << t.keypoints.size() << std::endl;
//     }
// }