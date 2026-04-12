#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tesseract/baseapi.h>
#include <tesseract/resultiterator.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    
    tesseract::TessBaseAPI tess_;
    tess_.Init(NULL, "eng");
    tess_.SetImage((uchar*)gray.data, gray.cols, gray.rows, 1, gray.step);
    tess_.Recognize(0);
    
    // Iterate through results with positions
    tesseract::ResultIterator* rit = tess_.GetIterator();
    tesseract::PageIteratorLevel level = tesseract::RIL_WORD;
    
    do {
        int x1, y1, x2, y2;
        rit->BoundingBox(level, &x1, &y1, &x2, &y2);
        
        const char* word = rit->GetUTF8Text(level);
        float conf = rit->Confidence(level);
        
        ROS_INFO("Word: '%s' | Pos: (%d,%d)-(%d,%d) | Confidence: %.2f%%", 
                 word, x1, y1, x2, y2, conf);
        
        // Draw bounding box on image
        cv::rectangle(cv_ptr->image, cv::Point(x1, y1), cv::Point(x2, y2), 
                      cv::Scalar(0, 255, 0), 2);
        
        delete[] word;
    } while (rit->Next(level));
    
    delete rit;
    cv::imshow("OCR Results", cv_ptr->image);
    cv::waitKey(0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ocr_reader_node");
    ros::NodeHandle nh;
    
    // Subscribe to image topic
    ros::Subscriber image_sub = nh.subscribe("front/image_raw", 1, imageCallback);
    
    ROS_INFO("OCR Reader Node started");
    ROS_INFO("Listening to topic: camera/image_raw");
    
    ros::spin();
    
    return 0;
}