#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <../include/Segmentation.hpp>
#include <../include/PCL_common.hpp>


void Segmentation::Downsampled_callback(const sensor_msgs::PointCloud2& msg)
{

  try
  {
    PCL_common::sensor_to_PointCloud( msg, cloud_ptr_ );

    PointCloud cloud_output;
    // cloud_output.reset();
    sensor_msgs::PointCloud2 output_msg;
    // cloud_output = cloud_ptr_;
    pcl::copyPointCloud( *cloud_ptr_, cloud_output );
    PointCloud::Ptr cloud_output_ptr( new PointCloud( cloud_output ) );

    for (size_t i = 0; i < loop_; i++) 
    {

      segmentation_config
      (
        dist_th_,
        cloud_output_ptr,
        true,
        pcl::SACMODEL_PLANE,
        pcl::SAC_RANSAC
      );
    
      seg_.segment (inliers_, coefficients_);

      pcl::PointIndices::Ptr inliers_ptr( new pcl::PointIndices( inliers_ ) );

      pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

      extract.setInputCloud( cloud_output_ptr );
      extract.setIndices( inliers_ptr );
      extract.setNegative( false );
      extract.filter( *cloud_output_ptr );
    }

    PCL_common::PointCloud_to_sensor( output_msg, cloud_output_ptr );
    pub_segmented_.publish( output_msg );

    reset();

  }
  catch( std::exception &e )
  {
    ROS_ERROR("%s", e.what());
  }

}


int main( int argc, char** argv )
{
    ros::init( argc, argv, "Segmentation" );
    Segmentation segmentation;
    ros::spin();
}


void Segmentation::segmentation_config
(   
  double dist_th,
  PointCloud::Ptr & cloud_ptr,      
  bool Optimize,
  int model,
  int method
)
{

  seg_.setOptimizeCoefficients (Optimize);

  seg_.setModelType (pcl::SACMODEL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);

  seg_.setDistanceThreshold (dist_th );
  seg_.setInputCloud ( cloud_ptr );

}


void Segmentation::reset( void )
{
  // coefficients_.reset();
  // inliers_.reset();
}