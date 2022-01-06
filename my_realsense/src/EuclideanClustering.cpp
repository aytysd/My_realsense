#include <ros/ros.h>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/Twist.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <visualization_msgs/Marker.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/centroid.h>
#include <../include/EuclideanClustering.hpp>
#include <../include/PCL_common.hpp>
#include <../include/Selection.hpp>


void EuclideanClustering::Downsampled_callback(const sensor_msgs::PointCloud2 & msg)
{

  PCL_common::sensor_to_PointCloud( msg, cloud_ptr_ );

  try
  {
    clustering_config
    (
      cloud_ptr_,
      0.06,
      0,
      cloud_ptr_ -> size()
    );
    
    ec_.extract( cluster_indices_ );

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
  ros::init( argc, argv, "EuclideanClustering" );
  EuclideanClustering clustering;
  ros::spin();
}




bool EuclideanClustering::check_size
(
  PointCloud::Ptr & cloud_ptr,
  pcl::PointIndices cluster_index,
  double max_size_x, 
  double max_size_y, 
  double max_size_z,
  double min_size_x, 
  double min_size_y, 
  double min_size_z
)
{
   
  pcl::getMinMax3D(*cloud_ptr, cluster_index, min_pt_, max_pt_);
  Eigen::Vector4f cluster_size = max_pt_ - min_pt_;

  bool is_ok = true;

  if (cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0)
  {

    if (cluster_size.x() < min_size_x || cluster_size.x() > max_size_x)
    {
      ROS_INFO( "sizex" );
      ROS_INFO( "%f", cluster_size.x() );
      is_ok = false;
    }
    if (cluster_size.y() < min_size_y || cluster_size.y() > max_size_y)
    {
      ROS_INFO( "sizey" );
      ROS_INFO( "%f", cluster_size.y() );
      is_ok = false;
    }
    if (cluster_size.z() < min_size_z || cluster_size.z() > max_size_z)
    {
      ROS_INFO( "sizez" );
      ROS_INFO( "%f", cluster_size.z() );
      is_ok = false;
    }

  }

  return is_ok;


}


bool EuclideanClustering::check_colour
(
  PointCloud::Ptr & cloud_ptr,
  pcl::PointIndices cluster_index, 
  double min_r, 
  double min_g,
  double min_b,
  double max_r,
  double max_g,
  double max_b
)
{
  pcl::PointIndices::Ptr cluster_index_ptr( new pcl::PointIndices( cluster_index ) );
  PointCloud out_points;

  PCL_common::extract
  (
    cluster_index_ptr,
    cloud_ptr,
    out_points
  );

  std::array< double, 3 > average;
  int sum = 0;
  bool is_ok = true;

  for( size_t i = 0; i < out_points.size(); i++ )
  {
    Eigen::Vector3i color = out_points.at( i ).getRGBVector3i();

    average[ 0 ] += color[ 0 ];
    average[ 1 ] += color[ 1 ];
    average[ 2 ] += color[ 2 ];        

  }

  average[ 0 ] /= ( double )out_points.size();
  average[ 1 ] /= ( double )out_points.size();
  average[ 2 ] /= ( double )out_points.size();

  if( !( average[ 0 ] < max_r_ && average[ 0 ] > min_r_ ) )
  {
    ROS_INFO( "color_r:%f", average[ 0 ] );
    is_ok = false;
  }
  else if( !( average[ 1 ] < max_g_ && average[ 1 ] > min_g_ ) )
  {
    is_ok = false;
    ROS_INFO( "color_g:%f", average[ 1 ] );
  }  
  else if( !( average[ 2 ] < max_b_ && average[ 2 ] > min_b_ ) )
  {
    is_ok = false;
    ROS_INFO( "color_b:%f", average[ 2 ] );
  }

  return is_ok;
          

}


void EuclideanClustering::clustering_config
(
    PointCloud::Ptr & cloud_ptr,
    double Tolerance,
    double min_size,
    double max_size
)
{

    pcl::Indices NaN;
    pcl::removeNaNFromPointCloud( *cloud_ptr, *cloud_ptr, NaN );

    tree_ -> setInputCloud( cloud_ptr );
    ec_.setClusterTolerance( Tolerance );
    ec_.setMinClusterSize( min_size );
    ec_.setMaxClusterSize( max_size );
    ec_.setSearchMethod( tree_ );


    ec_.setInputCloud( cloud_ptr );

}


void EuclideanClustering::make_Marker_array
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
    if( check_colour
        (
          cloud_ptr,
          *it, 
          min_r_,
          min_g_,
          min_b_,
          max_r_,
          max_g_,
          max_b_
        )
        &&
        check_size
        (
          cloud_ptr,
          *it,
          max_size_x_,
          max_size_y_,
          max_size_z_,
          min_size_x_,
          min_size_y_,
          min_size_z_
        )
      )
    {
      // sizes.push_back( it -> indices.size() );
      marker_array_.markers.push_back( PCL_common::make_Marker( "realsense", "cluster", marker_id, max_pt, min_pt, r, g, b, a) );
      PointCloud clustered;
      pcl::PointIndices::Ptr clustered_indices_ptr( new pcl::PointIndices );
      *clustered_indices_ptr = *it;
      PCL_common::extract( clustered_indices_ptr, cloud_ptr, clustered );
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid( clustered , centroid );

      ROS_INFO( "x:%f, y:%f, z:%f", centroid[ 0 ], centroid[ 1 ], centroid[ 2 ] );
      size_t size = clustered.points.size();
      ROS_INFO( "size%f", size );

      clear_indices.push_back( *clustered_indices_ptr );
      
    }


  }

  Selection select
  (
    cloud_ptr, 
    clear_indices, 
    ideal_r__, 
    ideal_g__, 
    ideal_b__,
    ideal_size__
  );
  
  pub_ideal_.publish( select.select() );
  pub_centroid_point_.publish( select.selected_centroid_ );



}

void EuclideanClustering::reset( void )
{
  cluster_indices_.clear();
  marker_array_.markers.clear();
  

}


