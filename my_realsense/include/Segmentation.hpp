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
#include <../include/PCL_common.hpp>
#include <my_realsense/PointIndices.h>

class Segmentation
{
private:

    ros::NodeHandle nh_;
    ros::NodeHandle np_;

    ros::Publisher pub_segmented_;
    ros::Subscriber sub_raw_;

    PointCloud::Ptr cloud_ptr_;

    pcl::SACSegmentation<PointT> seg_;

    int loop_;
    double dist_th_;


    void segmentation_config
    (
        double dist_th,
        PointCloud::Ptr & cloud_ptr,
        bool Optimize,
        int model,
        int method
    );


    void Downsampled_callback( const sensor_msgs::PointCloud2 & msg );

    void reset( void );

public:

    Segmentation():
        np_( "~" ),
        sub_raw_( nh_.subscribe( "/Downsampled", 10, &Segmentation::Downsampled_callback, this ) ),
        pub_segmented_( nh_.advertise< sensor_msgs::PointCloud2 >( "/Segmented", 10 ) ),
        cloud_ptr_( new PointCloud )
    {
        np_.getParam( "loop", loop_ );
        np_.getParam( "dist_th", dist_th_ );

    }
};