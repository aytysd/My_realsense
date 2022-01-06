#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>


class Follow
{
private:

    ros::NodeHandle nh_;
    ros::NodeHandle np_;

    ros::Publisher pub_command_;
    ros::Subscriber sub_point_;

    double follow_radius_th_;

    double linear_kp_;
    double angular_kp_;

    double max_rotate_speed_;
    double max_speed_;

    void point_callback( const geometry_msgs::Point& msg );


public:

    Follow():
        np_( "~" ),
        pub_command_( nh_.advertise<geometry_msgs::Twist>( "/command", 10 ) ),
        sub_point_( nh_.subscribe( "/centroid_point", 10, &Follow::point_callback, this ) )
    {
        np_.getParam( "follow_radius_th", follow_radius_th_ );
        np_.getParam( "linear_kp", linear_kp_ );
        np_.getParam( "angular_kp", angular_kp_ );
        np_.getParam( "max_speed", max_speed_ );
        np_.getParam( "max_rotate_speed", max_rotate_speed_ );
    };
};