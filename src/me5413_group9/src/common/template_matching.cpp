#ifndef TEMPLATE_MATCHING_C_
#define TEMPLATE_MATCHING_C_

#include "template_matching.hpp"

// #include <iostream> // << Debugging

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#ifdef HAVE_OPENCV_XFEATURES2D
    #include "opencv2/xfeatures2d"
#else
    // #warning "No XFeatures2d!"
#endif

namespace matching {
    using KeyPoints = std::vector<cv::KeyPoint>;
    using Matches = std::vector<cv::DMatch>;
    using Points = std::vector<cv::Point2f>;

    struct BoxTemplate {
        int       box_id;
        KeyPoints keypoints;
        cv::Mat   descriptors;
        cv::Mat   image;
    };

    // ================= :::::::::::::::::::::::::::::::::::::::::::::::
    // === Variables === :::::::::::::::::::::::::::::::::::::::::::::::
    // ================= :::::::::::::::::::::::::::::::::::::::::::::::
    const float RATIO_THRESH = 0.7f;
    const float MIN_GOOD_MATCHES = 0.15f;

    std::vector<BoxTemplate> box_templates;

    cv::Ptr<cv::Feature2D> detector;

    cv::Ptr<cv::DescriptorMatcher> matcher;

    // =============== :::::::::::::::::::::::::::::::::::::::::::::::::
    // === Helpers === :::::::::::::::::::::::::::::::::::::::::::::::::
    // =============== :::::::::::::::::::::::::::::::::::::::::::::::::
    // ::::::::::::::::::::::::::::::::
    // ::: Generating box templates :::
    // ::::::::::::::::::::::::::::::::
    /// @brief Get path to box texture within me5413_world/models/... 
    std::string box_texture_path(int box_id);

    /// @brief Crop a full single face from box texture
    cv::Mat crop_full_from_texture(const cv::Mat& texture);

    /// @brief Get path to matching template within me5413_group9/boxes/... 
    std::string box_image_path(int box_id, bool full = false);

    /// @brief Crop number more closely
    /// @note 0.7 seems to be the limit for "reasonable" matches
    cv::Mat crop_box_from_full(const cv::Mat& full_box, double scale_factor = 0.7);

    /// @brief Generate all box templates
    void generate_box_templates();

    // :::::::::::::::::::::::::::::::::::
    // ::: Initialization & Assertions :::
    // :::::::::::::::::::::::::::::::::::
    /// @brief Ensure matching::init_matching has been run
    void assert_init_matching();

    /// @brief Ensure image is non-empty
    void assert_non_empty(const cv::Mat& image, const std::string& operation);

    /// @brief Ensure box_id is appropriate (in range 1-9)
    void assert_box_id(int box_id);

    /// @brief Read matching template for a box
    cv::Mat load_box(int box_id);

    // :::::::::::::::::::
    // ::: Computation ::: 
    // :::::::::::::::::::
    /// @brief Compute good matches for a box
    Matches match_features(const KeyPoints& kpts, const cv::Mat& descr, int box_id);

    /// @brief Returns box_id of best match, if any are "good enough"
    /// @throws std::invalid_argument if good_matches.size() != 9
    int best_match_box(const std::vector<Matches>& good_matches);

    /// @brief Compute homography, or returns an empty cv::Mat if fewer than 4 good_matches
    cv::Mat get_homography(const Matches& good_matches, const KeyPoints& kpts, int box_id);

    /// @brief Change datatype of keypoint coordinates
    /// @param is_query - Running on template (first input to matcher)
    Points get_points_2f(const Matches& good_matches, const KeyPoints& kpts, bool is_query);

    /// @brief Get corner coordinates of image - ()
    Points get_corners(const cv::Mat& image);

    /// @brief Get center coordinates of image
    cv::Point2f get_center(const cv::Mat& image);

    /// @brief Clip points to within image borders
    void clip_points_2f(Points& points, const cv::Mat& image);

    // ::::::::::::::::::
    // ::: Homography :::
    // ::::::::::::::::::
    std::string homography_string(const cv::Mat& H) {
        if ( H.empty() || H.rows != 3 || H.cols != 3 )
            return "[INVALID HOMOGRAPHY]";

        std::stringstream ss;
        ss << std::fixed << std::setprecision(6);

        ss << "[";
        for ( int i = 0; i < 3; i++ ) {
            ss << "[ ";
            for ( int j = 0; j < 3; j++ ) {
                double val = H.at<double>(i, j);
                ss << std::setw(12) << val;
                if ( j < 2 ) 
                    ss << ", ";
            }
            ss << " ]";
            if ( i < 2 )
                ss << ",\n ";
            // ss << "\n";
        }
        ss << "]\n";
        return ss.str();
    }


    // ====================== ::::::::::::::::::::::::::::::::::::::::::
    // === Implementation === ::::::::::::::::::::::::::::::::::::::::::
    // ====================== ::::::::::::::::::::::::::::::::::::::::::
    // ::::::::::::::::::::::::::
    // ::: MatchedBox Members :::
    // ::::::::::::::::::::::::::
    std::string MatchedBox::get_homography_string() const {
        return homography_string(homography);
    }

    bool MatchedBox::is_match() const {
        return box_id && (homography.rows == 3) && (homography.cols == 3);
    }

    double MatchedBox::get_angle() const {
        assert_init_matching();
        
        if ( !is_match() )
            return 0.0;

        double h00 = homography.at<double>(0, 0);
        double h20 = homography.at<double>(2, 0);

        double mag = std::sqrt(h00*h00 + h20*h20);
        if ( mag < 1e-6 ) return 0.0;

        double c_theta = h00 / mag;
        double s_theta = h20 / mag;

        return std::atan2(s_theta, c_theta);
    }

    double MatchedBox::get_angle_deg() const {
        return get_angle() * 180.0 / M_PI;
    }

    double MatchedBox::get_height(const cv::Mat& image) const {
        assert_init_matching();
        assert_non_empty(image, "MatchedBox::get_height");

        if ( !is_match() )
            return 0.0;
        
            Points template_corners = get_corners(box_templates[box_id - 1].image);

            Points corners;
            cv::perspectiveTransform(template_corners, corners, homography);

            cv::Point2f top_center = (corners[0] + corners[1]) / 2.0;
            cv::Point2f bottom_center = (corners[3] + corners[2]) / 2.0;
            
            return bottom_center.y - top_center.y;
    }

    // ::::::::::::::::::
    // ::: Functions  :::
    // ::::::::::::::::::
    void init_matching() {
        #ifndef HAVE_OPENCV_XFEATURES2D
            // detector = cv::ORB::create();
            detector = cv::AKAZE::create();
        #else
            detector = cv::SIFT::create();
        #endif
        matcher = cv::DescriptorMatcher::create("BruteForce");

        for ( int box_id = 1; box_id < 10; box_id ++ ) {
            BoxTemplate box;

            box.box_id = box_id;
            box.image = load_box(box_id);
            
            detector->detectAndCompute(box.image, cv::noArray(), box.keypoints, box.descriptors);
            box_templates.push_back(box);
        }
    }



    MatchedBox detect(const cv::Mat& image) {
        assert_init_matching();
        assert_non_empty(image, "detect");

        KeyPoints kpts;
        cv::Mat descr;
        detector->detectAndCompute(image, cv::noArray(), kpts, descr);
    
        std::vector<Matches> all_good_matches;
        for ( int box_id = 1; box_id < 10; box_id ++ )
            all_good_matches.push_back(match_features(kpts, descr, box_id));
        
        MatchedBox match;
        match.box_id = best_match_box(all_good_matches);

        if ( match.box_id == 0 )
            return match;

        match.homography = get_homography(all_good_matches[match.box_id - 1], kpts, match.box_id);
        return match;
    }



    bool draw(cv::Mat& image, const MatchedBox& match) {
        assert_init_matching();
        assert_non_empty(image, "draw");
        
        if ( !match.is_match() )
            return false;
        
        Points template_corners = get_corners(box_templates[match.box_id - 1].image);

        Points corners;
        cv::perspectiveTransform(template_corners, corners, match.homography);
        clip_points_2f(corners, image);

        cv::line(image, corners[0], corners[1], cv::Scalar(0, 255, 0), 4);
        cv::line(image, corners[1], corners[2], cv::Scalar(0, 255, 0), 4);
        cv::line(image, corners[2], corners[3], cv::Scalar(0, 255, 0), 4);
        cv::line(image, corners[3], corners[0], cv::Scalar(0, 255, 0), 4);

        cv::Point center = (corners[0] + corners[2]) /  2.0;
        cv::putText(image, std::to_string(match.box_id), center, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        return true;
    }







    // ::::::::::::::
    // ::: Extras :::
    // ::::::::::::::
    const cv::Mat& get_box_template(int box_id) {
        assert_init_matching();
        assert_box_id(box_id);

        return box_templates[box_id - 1].image;
    }



    cv::Mat draw_features(const cv::Mat& image, int box_id) {
        assert_init_matching();
        assert_non_empty(image, "draw_features");
        assert_box_id(box_id);

        const BoxTemplate& box = box_templates[box_id - 1];

        KeyPoints kpts;
        cv::Mat descr;
        detector->detectAndCompute(image, cv::noArray(), kpts, descr);

        // std::cout << "Detected - also " << std::to_string(image.channels()) << std::endl;
        // std::cout << "           plus " << std::to_string(box.image.channels()) << std::endl;

        Matches good_matches = match_features(kpts, descr, box_id);

        cv::Mat result;
        cv::drawMatches(box.image, box.keypoints, image, kpts, good_matches, result, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // int height = std::max(box.image.rows, image.rows);
        // int width = box.image.cols + image.cols;
        // cv::Mat result(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

        // // Show template to the left
        // cv::Mat left_roi = result(cv::Rect(0, 0, box.image.cols, box.image.rows));
        // box.image.copyTo(left_roi);
        
        // // Show image to the right
        // cv::Mat right_roi = result(cv::Rect(box.image.cols, 0, image.cols, image.rows));
        // image.copyTo(right_roi);

        // // Draw each corresponding matched feature
        // for ( const auto& match: good_matches ) {
        //     cv::Point2f box_pt = box.keypoints[match.queryIdx].pt;
        //     cv::Point2f image_pt = kpts[match.trainIdx].pt;

        //     image_pt.x += box.image.cols;

        //     cv::line(result, box_pt, image_pt, cv::Scalar(0, 255, 0), 1);
        //     cv::circle(result, box_pt, 3, cv::Scalar(255, 0, 0), -1);
        //     cv::circle(result, box_pt, 3, cv::Scalar(0, 0, 255), -1);
        // }

        std::string match_text = std::to_string(good_matches.size()) + " / " + std::to_string(box.keypoints.size());
        cv::putText(result, match_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 50, 50));

        return result;
    }



    cv::Mat draw_feature_template(int box_id) {
        assert_init_matching();

        const BoxTemplate& box = box_templates[box_id];

        cv::Mat result = box.image.clone();

        for ( const auto& kpt: box.keypoints )
            cv::circle(result, kpt.pt, 5, cv::Scalar(0, 255, 0), 2);

        cv::putText(result, std::to_string(box.keypoints.size()), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(50, 50, 50));

        return result;
    }



    // ============================== ::::::::::::::::::::::::::::::::::
    // === Helpers Implementation === ::::::::::::::::::::::::::::::::::
    // ============================== ::::::::::::::::::::::::::::::::::
    std::string box_texture_path(int box_id) {
        auto parent_dir = [](const std::string& path) {
            size_t last_sep = path.rfind('/');
            return path.substr(0, last_sep);
        };

        std::string path = parent_dir(parent_dir(parent_dir(__FILE__)));
        path = parent_dir(path) + "/me5413_world/models";
        path += "/number" + std::to_string(box_id) + "/materials/textures";
        path += "/number" + std::to_string(box_id) + ".png";

        return path;
    }

    std::string box_image_path(int box_id, bool full) {
        auto parent_dir = [](const std::string& path) {
            size_t last_sep = path.rfind('/');
            return path.substr(0, last_sep);
        };

        std::string path = parent_dir(parent_dir(parent_dir(__FILE__)));
        if ( full )
            path += "/boxes/img";
        else
            path += "/boxes/cropped/img";

        return path + std::to_string(box_id) + ".png";
    }

    cv::Mat crop_full_from_texture(const cv::Mat& texture) {
        cv::Rect roi(688, 348, 330, 330); // Yielded good results
        return texture(roi);
    }

    cv::Mat crop_box_from_full(const cv::Mat& full_box, double scale_factor) {
        // NOTE: Using scale_factor less than 0.7 results in less than 10 features for box_id '7'
        double height = full_box.rows * scale_factor;
        double width = full_box.cols * scale_factor;
        double y_off = full_box.rows * (1 - scale_factor) / 2;
        double x_off = full_box.cols * (1 - scale_factor) / 2;
    
        cv::Rect roi(x_off, y_off, width, height);
        return full_box(roi);
    }

    void generate_box_templates() {
        for ( int i = 1; i < 10; i++ ) {
            cv::Mat texture = cv::imread(box_texture_path(i));
            // texture.convertTo(texture, CV_8U, 0.6, 0);
            
            if ( texture.empty() )
                throw std::runtime_error("Could not open image " + box_texture_path(i));

            cv::Mat full_box = crop_full_from_texture(texture);

            cv::imwrite(box_image_path(i, true), full_box);
            cv::imwrite(box_image_path(i, false), crop_box_from_full(full_box));
        }
    }

    void assert_init_matching() {
        if ( box_templates.size() != 9 || !(detector) || !(matcher) )
            throw std::runtime_error("Never run matching::init_matching!");
    }

    void assert_non_empty(const cv::Mat& image, const std::string& operation) {
        if ( image.empty() )
            throw std::invalid_argument("Can't run " + operation + " on an empty image!");
    }

    void assert_box_id(int box_id) {
        if ( box_id < 1 || box_id > 9 )
            throw std::invalid_argument("Box ID must be in [1, 9], not " + std::to_string(box_id));
    }

    cv::Mat load_box(int box_id) {
        std::string path = box_image_path(box_id);
        
        cv::Mat img = cv::imread(path);

        if ( img.empty() )
            throw std::runtime_error("Failed to load box image at: " + path);
        
        return img;
    }

    Matches match_features(const KeyPoints& kpts, const cv::Mat& descr, int box_id) {
        const BoxTemplate& box = box_templates[box_id - 1];

        // Lowe's ratio test
        std::vector<Matches> matches;
        matcher->knnMatch(box.descriptors, descr, matches, 2);

        Matches good_matches;
        for ( size_t i = 0; i < matches.size(); i++ )
            if ( matches[i].size() >= 2 )
                if ( matches[i][0].distance < RATIO_THRESH * matches[i][1].distance )
                    good_matches.push_back(matches[i][0]);

        return good_matches;
    }

    int best_match_box(const std::vector<Matches>& good_matches) {
        if ( good_matches.size() != 9 )
            throw std::runtime_error("best_match_box - needs 9 good_match sets!");

        int box_id  = 0;
        float most = MIN_GOOD_MATCHES;

        for ( size_t i = 0; i < 9; i++ ) {
            float count = (float) good_matches[i].size() / (float) box_templates[i].keypoints.size();
            
            // std::cout << std::to_string(i) << ": " << count << " ..." << std::endl;
            if ( count > most ) {
                box_id = i + 1;
                most = count;
            }
        }

        return box_id;
    }

    cv::Mat get_homography(const Matches& good_matches, const KeyPoints& kpts, int box_id) {
        if ( good_matches.size() < 4 ) return cv::Mat();
        
        Points box_pts, scene_pts;

        const BoxTemplate& box = box_templates[box_id - 1];

        for ( const cv::DMatch& match: good_matches ) {
            box_pts.push_back( box.keypoints[ match.queryIdx ].pt );
            scene_pts.push_back( kpts[ match.trainIdx ].pt );
        }

        return cv::findHomography(box_pts, scene_pts, cv::RANSAC );
    }

    Points get_points_2f(const Matches& good_matches, const KeyPoints& kpts, bool is_query) {
        Points points;
        if ( is_query )
            for ( size_t i = 0; i < good_matches.size(); i++ )
                points.push_back( kpts[good_matches[i].queryIdx].pt );
        else
            for ( size_t i = 0; i < good_matches.size(); i++ )
                points.push_back( kpts[good_matches[i].trainIdx].pt );
        return points;
    }

    Points get_corners(const cv::Mat& image) {
        Points corners(4);
        corners[0] = cv::Point2f(0, 0);
        corners[1] = cv::Point2f((float)image.cols, 0);
        corners[2] = cv::Point2f((float)image.cols, (float)image.rows);
        corners[3] = cv::Point2f(0, (float)image.rows);
        return corners;
    }

    cv::Point2f get_center(const cv::Mat& image) {
        return cv::Point2f((float)image.cols, (float)image.rows) / 2.0;
    }

    void clip_points_2f(Points& points, const cv::Mat& image) {
        for ( auto& point: points ) {
            point.x = std::max(0.0f, std::min((float)image.cols - 1.0f, point.x));
            point.y = std::max(0.0f, std::min((float)image.rows - 1.0f, point.y));
        }
    }

    
}

#endif // TEMPLATE_MATCHING_C_
