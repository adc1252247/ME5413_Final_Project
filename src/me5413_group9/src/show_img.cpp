#include <iostream>

#include "template_matching.hpp"

#include "common/template_matching.cpp"

#include "opencv2/highgui/highgui.hpp"

using namespace matching;

void display(const cv::Mat& image) {
    while ( true ) { 
        cv::imshow("show_img", image);

        if ( cv::waitKey(10) == 'q' )
            break;
    }
}

int main() {
    generate_box_templates();

    // init_matching();
    // std::cout << "Init...ed..." << std::endl;

    // for ( int i = 1; i < 10; i++ )
    //     display(draw_feature_template(i));

    // std::cout << "Done" << std::endl;

    // for ( int i = 1; i < 10; i++ ) {
    //     cv::Mat full_img = cv::imread(box_image_path(i, true));

    //     cv::Mat cropped = crop_box_from_full(full_img);

    //     cv::imwrite(box_image_path(i, false), cropped);
    // }

    init_matching();

    // Must be run from root of project
    cv::Mat image = cv::imread("./misc/distance_1.png");

    std::cout << "1] Image open..." << std::endl;

    auto match = detect(image);

    std::cout << "2] Detection complete..." << std::endl;
    std::cout << "   " << match.get_height(image) << std::endl;
    std::cout << "   " << match.get_angle_deg() << std::endl;
    std::cout << match.get_homography_string() << std::endl;

    std::cout << (match.is_match() ? "   (x) Found" : "   ( ) None found") << std::endl;

    cv::Mat drawing = image.clone();
    draw(drawing, match);

    std::cout << "3] Drawn..." << std::endl;

    display(drawing);


    for ( int i = 1; i < 10; i++ ) {
        cv::Mat drawing = draw_features(image, i);
        display(drawing);
    }
    
    cv::destroyAllWindows();

   
    
}