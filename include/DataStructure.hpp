#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
// #include "plane_slam/PlanesMsg.h"
// #include "sophus/se3.hpp"

#ifndef DATASTRUCT_HPP
#define DATASTRUCT_HPP

struct KeyFrame{
    int frame_id = 0;
    double timestamp;
    Eigen::Matrix4d Transform;
    int hasground;
    int plane_num;
    std::vector<std::vector<double>> planes;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_hulls;
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud;
};

struct PlaneInfo{

    int isGround = 0;
    int observedTime = 0;
    int keyframeNum = 0;
    std::vector<double> coe;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points;
};

#endif //  DATASTRUCT_HPP