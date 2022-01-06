#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

#pragma once

class PCL_common
{
public:

    static void sensor_to_PointCloud
    (
        const sensor_msgs::PointCloud2 & msg, 
        pcl::PointCloud<PointT>::Ptr & cloud_ptr 
    );



    static void PointCloud_to_sensor
    (
        sensor_msgs::PointCloud2 & msg, 
        const PointCloud::Ptr & cloud_ptr 
    );

    static void normal_PointCloud_to_sensor
    (
        sensor_msgs::PointCloud2 & msg, 
        const pcl::PointCloud<pcl::Normal>::Ptr & cloud_ptr 
    );


    static void extract
    (
        pcl::PointIndices::Ptr &cluster_index_ptr,
        PointCloud::Ptr& src_ptr,
        PointCloud &out
    );

    static double check_size
    (
        PointCloud::Ptr& scr_ptr
    );

    static visualization_msgs::Marker make_Marker
    (
        const std::string &frame_id,
        const std::string &marker_ns, 
        int marker_id, 
        const Eigen::Vector4f &max_pt, 
        const Eigen::Vector4f &min_pt, 
        float r, 
        float g, 
        float b, 
        float a 
    );


};

