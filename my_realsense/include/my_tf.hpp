#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


class my_tf
{
private:

    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    const std::string frame_id_;
    const std::string child_frame_id_;
    const std::array<double, 3UL> translation_;
    const double yaw_;


public:



    void broadcast_static_tf( std::string frame_id, std::string child_frame_id, std::array<double, 3UL> translation, double yaw, tf2_ros::StaticTransformBroadcaster& static_tf_broadcaster );

    my_tf(
        std::string frame_id,
        std::string child_frame_id,
        std::array<double, 3UL> translation,
        double yaw
    ):
        frame_id_( frame_id ),
        child_frame_id_( child_frame_id ),
        translation_( translation ),
        yaw_( yaw )
    {
        my_tf::broadcast_static_tf(
            frame_id_,
            child_frame_id_,
            translation_,
            yaw_,
            static_tf_broadcaster_
        );
    };
};


