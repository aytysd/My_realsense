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
#include <../include/PCL_common.hpp>
#include <../include/Selection.hpp>

void Selection::extract_ideal_size( void )
{

  PointCloud extracted;
  std::vector< double > size_diff;

  for
  (
    std::vector<pcl::PointIndices>::iterator it = inliers_.begin(), it_end = inliers_.end();
    it != it_end;
    ++it 
  )
  {
    pcl::PointIndices::Ptr index_ptr( new pcl::PointIndices );
    *index_ptr = *it;
    PCL_common::extract( index_ptr, src_ptr_, extracted );

    Eigen::Vector4f max_pt, min_pt;
    pcl::getMinMax3D( extracted, min_pt, max_pt );


    Eigen::Vector4f diagnol_vector = max_pt - min_pt;

    double V = diagnol_vector[ 0 ] * diagnol_vector[ 1 ] * diagnol_vector[ 2 ];

    double diff = V - ideal_size_;

    size_diff.push_back( diff );
  }

  std::vector< double >::iterator it = std::min_element( size_diff.begin(), size_diff.end() );
  size_t index_num = size_index_num_ = std::distance( size_diff.begin(), it );

  size_diff_score_ = 1 - ( size_diff.at( index_num ) / ideal_size_ );
}

void Selection::extract_ideal_colour( void )
{
  PointCloud extracted;
  std::vector< std::array< double , 3 > > colour_diff;

  for
  (
    std::vector<pcl::PointIndices>::iterator it = inliers_.begin(), it_end = inliers_.end();
    it != it_end;
    ++it 
  )
  {
    pcl::PointIndices::Ptr index_ptr( new pcl::PointIndices );
    *index_ptr = *it;
    PCL_common::extract( index_ptr, src_ptr_, extracted );

    std::array< double, 3 > colour_average_diff;

    
    for( size_t i = 0; i < extracted.size(); ++i )
    {
      Eigen::Vector3i colour = extracted.at( i ).getRGBVector3i();

      colour_average_diff[ 0 ] += colour[ 0 ];
      colour_average_diff[ 1 ] += colour[ 1 ];
      colour_average_diff[ 2 ] += colour[ 2 ];

    }

    colour_average_diff[ 0 ] /= extracted.size();
    colour_average_diff[ 1 ] /= extracted.size();
    colour_average_diff[ 2 ] /= extracted.size();


    colour_average_diff[ 0 ] -= ideal_r_;
    colour_average_diff[ 1 ] -= ideal_g_;
    colour_average_diff[ 2 ] -= ideal_b_;



    colour_diff.push_back( colour_average_diff );


  };

  std::vector< std::array< double, 3 > >::iterator it = std::min_element( colour_diff.begin(), colour_diff.end() );
  size_t index_num = colour_index_num_ = std::distance( colour_diff.begin(), it );


  colour_diff_score_ = 
    (
      ( 1 - ( colour_diff.at( index_num )[ 0 ] / ideal_r_ ) ) +
      ( 1 - ( colour_diff.at( index_num )[ 1 ] / ideal_g_ ) ) + 
      ( 1 - ( colour_diff.at( index_num )[ 2 ] / ideal_b_ ) ) 
    ) / 3;

}


visualization_msgs::Marker Selection::select( void )
{

  size_t index_num;
  Eigen::Vector4f selected_centroid;

  if( colour_index_num_ == size_index_num_ )
    index_num = colour_index_num_;
  else
  {
    if( size_diff_score_ > colour_diff_score_ )
      index_num = colour_index_num_;
    else
      index_num = size_index_num_;
  }

    pcl::compute3DCentroid( *src_ptr_, inliers_.at( index_num ), selected_centroid );

    selected_centroid_.x = selected_centroid[ 0 ];
    selected_centroid_.y = selected_centroid[ 2 ];
    selected_centroid_.z = -selected_centroid[ 1 ];

    PointCloud out;
    Eigen::Vector4f max_pt;
    Eigen::Vector4f min_pt;

    pcl::getMinMax3D( *src_ptr_, inliers_.at( index_num ), min_pt, max_pt );

    return PCL_common::make_Marker
            (
              "realsense",
              "cluster",
              96, 
              max_pt, 
              min_pt, 
              255, 
              255, 
              0, 
              255
            );

}
