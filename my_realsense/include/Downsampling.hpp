#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>

class Downsampling
{
private:

    ros::NodeHandle nh_;
    ros::NodeHandle np_;

    ros::Subscriber raw_sub_;
    ros::Publisher downsampled_pub_;

    void downsampling( const sensor_msgs::PointCloud2 & cloud );

    bool downsampling_;
    double voxel_size_;

    void Passed_through_callback( const sensor_msgs::PointCloud2 & msg );

public:

    Downsampling():
        np_( "~" ),
        raw_sub_( nh_.subscribe( "/Passed_through", 10, &Downsampling::Passed_through_callback, this ) ),
        downsampled_pub_( nh_.advertise<sensor_msgs::PointCloud2>( "/Downsampled", 10 ) )      
    {
        np_.getParam( "voxel_size", voxel_size_ );
        np_.getParam( "downsampling", downsampling_ );
    };



};


