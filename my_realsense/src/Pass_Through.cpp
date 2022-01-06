#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
// PCL specific includes

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>  
#include <tf2_ros/static_transform_broadcaster.h>
#include <../include/my_tf.hpp>
#include <../include/Pass_Through.hpp>

void Pass_Through::raw_callback( const sensor_msgs::PointCloud2 & msg )
{
    pass_through
    (
        msg,
        "realsense",
        width_max_x_,
        width_min_x_,
        width_max_y_,
        width_min_y_,
        width_max_z_,
        width_min_z_,
        pass_x_,
        pass_y_,
        pass_z_
    );
}


int main( int argc, char** argv )
{
    
    ros::init( argc, argv, "Passing_Through" );
    Pass_Through pass_through;
    my_tf realsense_tf( "robot", "realsense", pass_through.translation_, 0.0 );
    ros::spin();


}

void Pass_Through::pass_through
(
    const sensor_msgs::PointCloud2 & msg,
    const std::string &frame_id,
    double width_max_x,
    double width_min_x,
    double width_max_y,
    double width_min_y,
    double width_max_z,
    double width_min_z,
    bool pass_x,
    bool pass_y,
    bool pass_z
)
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2();
    pcl::PCLPointCloud2Ptr cloud_ptr( cloud );
    sensor_msgs::PointCloud2 output_msg;

  
    
    pcl_conversions::toPCL( msg, *cloud );
    cloud -> header.frame_id = frame_id;
 
    pcl::PassThrough<pcl::PCLPointCloud2> path_x;
    pcl::PassThrough<pcl::PCLPointCloud2> path_y;
    pcl::PassThrough<pcl::PCLPointCloud2> path_z;

    pcl::PCLPointCloud2 passed_through;


    if( pass_x == true )
    {
        path_x.setFilterFieldName( "x" );
        path_x.setFilterLimits( width_min_x, width_max_x );
        path_x.setInputCloud( cloud_ptr );
        path_x.filter( passed_through );
        *cloud = passed_through;
    }

    if( pass_y == true )
    {
        path_y.setFilterFieldName( "y" );
        path_y.setFilterLimits( width_min_y, width_max_y );
        path_y.setInputCloud( cloud_ptr );
        path_y.filter( passed_through );
        *cloud = passed_through;
    }

    if( pass_z == true )
    {
        path_z.setFilterFieldName( "z" );
        path_z.setFilterLimits( width_min_z, width_max_z );
        path_z.setInputCloud( cloud_ptr );
        path_z.filter( passed_through );
        *cloud = passed_through;
    }


    // pcl_conversions::fromPCL( passed_through, output_msg );
    pcl_conversions::fromPCL( *cloud, output_msg );
    pub_passed_through_.publish( output_msg );
}

