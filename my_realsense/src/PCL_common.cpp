#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <../include/PCL_common.hpp>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

void PCL_common::sensor_to_PointCloud
(
    const sensor_msgs::PointCloud2 & msg, 
    pcl::PointCloud<PointT>::Ptr & cloud_ptr
)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(msg,pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud_ptr); 

};





void PCL_common::PointCloud_to_sensor
(
    sensor_msgs::PointCloud2 & msg, 
    const PointCloud::Ptr & cloud_ptr
)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2( *cloud_ptr, pcl_pc2 );
  pcl_conversions::fromPCL( pcl_pc2, msg );
  

};

void PCL_common::normal_PointCloud_to_sensor
(
    sensor_msgs::PointCloud2 & msg, 
    const pcl::PointCloud<pcl::Normal>::Ptr & cloud_ptr
)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2( *cloud_ptr, pcl_pc2 );
  pcl_conversions::fromPCL( pcl_pc2, msg );
  

};




void PCL_common::extract
(
  pcl::PointIndices::Ptr &cluster_index_ptr,
  PointCloud::Ptr& src_ptr,
  PointCloud &out
)
{
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud( src_ptr );
  extract.setIndices( cluster_index_ptr );
  extract.filter( out );
}



visualization_msgs::Marker PCL_common::make_Marker
(
  const std::string &frame_id,
  const std::string &marker_ns,
  int marker_id,
  const Eigen::Vector4f &max_pt,
  const Eigen::Vector4f &min_pt, 
  float r, float g, float b, float a) 
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = marker_ns;
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
  marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
  marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = max_pt.x() - min_pt.x();
  marker.scale.y = max_pt.y() - min_pt.y();
  marker.scale.z = max_pt.z() - min_pt.z();

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  marker.lifetime = ros::Duration(0.3);
  return marker;
}
