#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <../include/RegionGrowing.hpp>
#include <pcl/segmentation/region_growing.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/filter_indices.h>
#include <../include/PCL_common.hpp>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/features/integral_image_normal.h>



void RegionGrowing::Downsampled_callback(const sensor_msgs::PointCloud2 & msg)
{

  PCL_common::sensor_to_PointCloud( msg, cloud_ptr_ );

  try
  {
    clustering_config
    (
      cloud_ptr_,
      0.03,
      number_of_neighbours_,
      normals_,
      smoothness_threshold_,
      curvature_threshold_,
      0,
      cloud_ptr_ -> size()
    );
    
    clustering_.extract( cluster_indices_ );

    make_Marker_array
    (
      cloud_ptr_,
      cluster_indices_, 
      max_pt_, 
      min_pt_, 
      0.0, 
      1.5, 
      0.0, 
      0.5 
    );

    if( marker_array_.markers.empty() == false )
    {
      pub_clusters_.publish( marker_array_ );
    }
      

  }
  catch( std::exception &e )
  {
    ROS_ERROR("%s", e.what());
  }


  reset();



}


int main( int argc, char** argv )
{
  ros::init( argc, argv, "RegionGrowing" );
  RegionGrowing region_growing;
  ros::spin();
}





void RegionGrowing::clustering_config
(
  PointCloud::Ptr & cloud_ptr,
  double radius,
  int NumberOfNeighbours,
  pcl::PointCloud<pcl::Normal>::Ptr input_normals,
  double smoothness_Threshold,
  double CurvatureThreshold,
  double min_size,
  double max_size
)
{


  pcl::Indices NaN;
  pcl::removeNaNFromPointCloud( *cloud_ptr, *cloud_ptr, NaN );

	tree_ -> setInputCloud( cloud_ptr );

	// Estimate the normals.      法線を推定
  // normalEstimation_.set

  ROS_INFO( "%d", cloud_ptr -> isOrganized() );

	normalEstimation_.setInputCloud( cloud_ptr );
	normalEstimation_.setRadiusSearch( radius );
	normalEstimation_.setSearchMethod( tree_ );
	normalEstimation_.compute( *normals_ );

	// Region growing clustering object.　　　　領域成長クラスタリングのオブジェクト
	clustering_.setMinClusterSize( min_size );
	clustering_.setMaxClusterSize( max_size );
	clustering_.setSearchMethod( tree_ );
	clustering_.setNumberOfNeighbours( NumberOfNeighbours );
	clustering_.setInputCloud( cloud_ptr );
	clustering_.setInputNormals( normals_ );
	// Set the angle in radians that will be the smoothness threshold 滑らかさの閾値の角度をラディアンで設定
	// (the maximum allowable deviation of the normals).
	clustering_.setSmoothnessThreshold( smoothness_Threshold ); // 7 degrees. 7度をラディアンに
	// Set the curvature threshold. The disparity between curvatures will be
	// tested after the normal deviation check has passed.
    // 曲率の閾値の設定。標準偏差の検査が通ったあとで曲率差が調べられる
	clustering_.setCurvatureThreshold( CurvatureThreshold );


  viewer_ -> removePointCloud( "cloud", 0 );
  viewer_ -> removePointCloud( "normals", 0 );

  viewer_-> addPointCloud<PointT>( cloud_ptr_, "cloud" );
  viewer_ -> addPointCloudNormals<PointT, pcl::Normal>
  (
      cloud_ptr_, normals_, 
      1, 
      0.03, 
      "normals"
  );

	viewer_ -> spinOnce();



}


void RegionGrowing::make_Marker_array
(
  PointCloud::Ptr & cloud_ptr, 
  std::vector<pcl::PointIndices>& cluster_indices,
  Eigen::Vector4f &max_pt,
  Eigen::Vector4f &min_pt,
  double r,
  double g,
  double b,
  double a
)
{

  int marker_id = 0;
  std::vector< Eigen::Vector4f > centroid_point;
  std::vector< double > sizes;

  std::vector< pcl::PointIndices > clear_indices;

  for 
  (std::vector<pcl::PointIndices>::const_iterator 
  it = cluster_indices.begin(),
  it_end = cluster_indices.end();            
  it != it_end; ++it, ++marker_id
  )
  {
    pcl::getMinMax3D(*cloud_ptr, *it, min_pt_, max_pt_);

    marker_array_.markers.push_back
    (
        PCL_common::make_Marker
        (
            "realsense", 
            "cluster", 
            marker_id, 
            max_pt, 
            min_pt, 
            marker_id * 2, 
            marker_id * 2, 
            marker_id * 2, 
            a
        ) 
    );
  }



}

void RegionGrowing::reset( void )
{
  cluster_indices_.clear();
  marker_array_.markers.clear();
  

}


