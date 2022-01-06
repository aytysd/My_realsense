#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <../include/Follow.hpp>

int count = 0;

void Follow::point_callback( const geometry_msgs::Point& msg )
{
    geometry_msgs::Twist move;

    if( abs( msg.x ) > max_rotate_speed_ )
    {
        if( msg.x > 0 )
            move.angular.z = max_rotate_speed_;
        else
            move.angular.z = -max_rotate_speed_;
    }    
    else   
        move.angular.z = msg.x;

    if( abs( msg.y - follow_radius_th_ ) < max_speed_ )
        move.linear.y = msg.y - follow_radius_th_;
    else
    {
        if( msg.y - follow_radius_th_ < 0 )
            move.linear.y = -max_speed_;
        else
            move.linear.y = max_speed_;
    }


    ROS_INFO( "x:%f", msg.x );
    ROS_INFO( "y:%f", msg.y );
    ROS_INFO( "rotate:%f", move.angular.z );
    ROS_INFO( "linear:%f", move.linear.y );
        
    count++;
    if( count >= 45 )
    {
        pub_command_.publish( move );
        count = 0;
    }
    
    

}


int main( int argc, char** argv )
{

    ros::init( argc, argv, "Follow" );
    Follow follow;
    ros::spin();

}