#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
// PCL specific includes

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>  
#include <tf2_ros/static_transform_broadcaster.h>
#include <../include/PCL_common.hpp>


class Pass_Through
{
private:

    ros::NodeHandle nh_;
    ros::NodeHandle np_;

    ros::Publisher pub_passed_through_;
    ros::Publisher pubdownsampled_;

    ros::Subscriber sub_raw_;

    double width_min_x_;
    double width_max_x_;
    double width_min_y_;
    double width_max_y_;
    double width_min_z_;
    double width_max_z_;

    bool pass_x_, pass_y_, pass_z_;



    void raw_callback( const sensor_msgs::PointCloud2 & msg );

    void pass_through
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
    );

public:

    std::array< double, 3UL > translation_;

    Pass_Through():
        np_( "~" ),
        sub_raw_( nh_.subscribe( "/camera/depth_registered/points", 10, &Pass_Through::raw_callback, this ) ),
        pub_passed_through_( nh_.advertise<sensor_msgs::PointCloud2>( "/Passed_through", 10 ) )

    {

        np_.getParam( "camera_pos_x", translation_[ 0 ] );
        np_.getParam( "camera_pos_y", translation_[ 1 ] );
        np_.getParam( "camera_pos_z", translation_[ 2 ] );

        np_.getParam( "width_max_y", width_max_y_ );
        np_.getParam( "width_min_y", width_min_y_ );
        np_.getParam( "width_max_x", width_max_x_ );
        np_.getParam( "width_min_x", width_min_x_ );
        np_.getParam( "width_max_z", width_max_z_ );
        np_.getParam( "width_min_z", width_min_z_ );

        np_.getParam( "pass_x", pass_x_ );
        np_.getParam( "pass_y", pass_y_ );
        np_.getParam( "pass_z", pass_z_ ); 


        

    }


};