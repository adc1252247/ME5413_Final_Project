#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>

// #include <sensor_msgs/LaserScan.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <utility>
#include <cmath>
#include <limits>

struct Arc {
    float center_x;
    float center_y;
    float radius;
    std::vector<int> indices;
};

struct ScanConfig {
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
};

// Not using laser scan from ROS, such that I can test by myself
std::vector<std::pair<float, float>> pointify(const std::vector<float>& ranges, ScanConfig cfg) {
    std::vector<std::pair<float, float>> points;

    for ( size_t i = 0; i < ranges.size(); i++ ) {
        float dist  = ranges[i];

        if ( std::isnan(dist) || std::isinf(dist) || dist > cfg.range_max || dist < cfg.range_min )
            continue;

        float angle = cfg.angle_min + cfg.angle_increment * static_cast<float>(i);

        std::pair<float, float> point;
        point.first  = cos(angle) * dist;
        point.second = sin(angle) * dist;

        points.push_back(point);
    }

    return points;
}

std::vector<Arc> segment_arcs(const std::vector<std::pair<float, float>>& points){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& p : points) {
        pcl::PointXYZ pt;
        pt.x = p.first;
        pt.y = p.second;
        pt.z = 0.0f;
        cloud->push_back(pt);
    }
    
    std::vector<Arc> arcs;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // https://github.com/PointCloudLibrary/pcl/blob/master/sample_consensus/include/pcl/sample_consensus/sac_model_circle3d.h
    // Coefficients:
    // 0: x
    // 1: y
    // 2: z
    // 3: r
    // 4: normal x
    // 5: normal y
    // 6: normal z
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setDistanceThreshold(0.05);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
    
    while (remaining->size() > 3) {
        seg.setInputCloud(remaining);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.empty()) break;
        
        // If too large, ignore
        if ( coefficients->values[3] < 2.0 ) {
            Arc arc;
            arc.center_x = coefficients->values[0];
            arc.center_y = coefficients->values[1];
            arc.radius = coefficients->values[3];  // radius is at index 3 for CIRCLE3D

            // std::cout << "Found arc with: " << inliers->indices.size()
            //         << "points. | " << arc.radius << "m radius. | At: "
            //         << arc.center_x << " by " << arc.center_y << std::endl;

            arc.indices = inliers->indices;
            arcs.push_back(arc);
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < remaining->size(); ++i) {
            if (std::find(inliers->indices.begin(), inliers->indices.end(), i) == inliers->indices.end()) {
                temp->push_back(remaining->at(i));
            }
        }
        remaining = temp;
    }
    
    return arcs;
}