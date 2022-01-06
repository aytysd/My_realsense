#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <../include/RegionGrowingRGB.hpp>
#include <pcl/segmentation/region_growing.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/filter_indices.h>
#include <../include/PCL_common.hpp>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/region_growing_rgb.h>


void RegionGrowing_RGB::Downsampled_callback(const sensor_msgs::PointCloud2 & msg)
{

  PCL_common::sensor_to_PointCloud( msg, cloud_ptr_ );

  try
  {
    clustering_config
    (
      cloud_ptr_,
      distance_threshold_,
      diff_threshold_,
      merge_threshold_,
      0
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
      pub_cluster_indices_.publish( cluster_indices_ );
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
  ros::init( argc, argv, "RegionGrowing_RGB" );
  RegionGrowing_RGB region_growing;
  ros::spin();
}





void RegionGrowing_RGB::clustering_config
(
  PointCloud::Ptr & cloud_ptr,
  double distance_threshold,
  double diff_threshold,
  double merge_threshold,
  double min_size
)
{


    pcl::Indices NaN;
    pcl::removeNaNFromPointCloud( *cloud_ptr, *cloud_ptr, NaN );


    clustering_.setInputCloud( cloud_ptr );
    clustering_.setSearchMethod( tree_ );
    // Here, the minimum cluster size affects also the postprocessing step:
    // clusters smaller than this will be merged with their neighbors.
    // ここで最小クラスタのサイズは後処理に影響する：これより小さなクラスタは近傍のクラスタにマージ
    clustering_.setMinClusterSize( min_size );
    // Set the distance threshold, to know which points will be considered neighbors.
    // 距離の閾値の設定。近傍としてみなせるポイントを決めるためのもの
    clustering_.setDistanceThreshold( 10 );
    // Color threshold for comparing the RGB color of two points.　2点のRGB色を比較するための色閾値
    clustering_.setPointColorThreshold( diff_threshold );
    // Region color threshold for the postprocessing step: clusters with colors
    // within the threshold will be merged in one.
    // 後処理のための領域色閾値: この閾値以下の色を持つクラスタは一つにマージ
    clustering_.setRegionColorThreshold( merge_threshold );

}


void RegionGrowing_RGB::make_Marker_array
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

void RegionGrowing_RGB::reset( void )
{
  cluster_indices_.clear();
  marker_array_.markers.clear();
  

}


