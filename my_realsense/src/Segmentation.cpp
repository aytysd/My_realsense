#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <../include/Segmentation.hpp>
#include <../include/PCL_common.hpp>


void Segmentation::Downsampled_callback(const sensor_msgs::PointCloud2& msg)
{

  try
  {
    PCL_common::sensor_to_PointCloud( msg, cloud_ptr_ );
    PointCloud::Ptr out_ptr( new PointCloud );
    sensor_msgs::PointCloud2 output_msg;

    pcl::ModelCoefficients::Ptr coefficients_ptr( new pcl::ModelCoefficients );
    pcl::PointIndicesPtr inliers_ptr( new pcl::PointIndices );

    for (size_t i = 0; i < loop_; i++) 
    {

      segmentation_config
      (
        dist_th_,
        cloud_ptr_,
        true,
        pcl::SACMODEL_NORMAL_PARALLEL_PLANE,
        pcl::SAC_RANSAC
      );
    
      seg_.segment( *inliers_ptr, *coefficients_ptr );
      PCL_common::extract( inliers_ptr, cloud_ptr_, *cloud_ptr_ );

    }

    PCL_common::extract( inliers_ptr, cloud_ptr_, *out_ptr );
    PCL_common::PointCloud_to_sensor( output_msg, out_ptr );

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
  // coefficients_ptr_.reset();
  // inliers_ptr_.reset();
}