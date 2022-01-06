#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <../include/Downsampling.hpp>

void Downsampling::Passed_through_callback( const sensor_msgs::PointCloud2 & msg )
{
    downsampling( msg );
}


int main( int argc, char** argv )
{   
    ros::init( argc, argv, "Downsampling" );
    Downsampling downsampling;
    ros::spin();
}

void Downsampling::downsampling( const sensor_msgs::PointCloud2 & msg )
{

    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2();
    pcl::PCLPointCloud2ConstPtr cloud_ptr( cloud );
    pcl::PCLPointCloud2 cloud_downsampled;
    sensor_msgs::PointCloud2 cloud_output;


    if( downsampling_ == true )
    {
        pcl_conversions::toPCL( msg, *cloud );


        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
        voxel.setInputCloud( cloud_ptr );
        voxel.setLeafSize( voxel_size_, voxel_size_, voxel_size_ );
        voxel.filter( cloud_downsampled );

        pcl_conversions::fromPCL( cloud_downsampled, cloud_output );

        downsampled_pub_.publish( cloud_output );

        return;
    }
    else
    {
        downsampled_pub_.publish( msg );
        return;
    }

}

