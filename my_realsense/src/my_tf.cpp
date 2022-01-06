#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <../include/my_tf.hpp>



void my_tf::broadcast_static_tf(
    std::string frame_id,
    std::string child_frame_id,
    std::array<double, 3UL> translation,
    double yaw,
    tf2_ros::StaticTransformBroadcaster& static_tf_broadcaster
    )   
{
    geometry_msgs::TransformStamped static_tf;

    static_tf.header.stamp = ros::Time::now(); //時間
    static_tf.header.frame_id = frame_id; //親フレーム
    static_tf.child_frame_id = child_frame_id; //子フレーム
    static_tf.transform.translation.x = translation.at(0);
    static_tf.transform.translation.y = translation.at(1);
    static_tf.transform.translation.z = translation.at(2);
    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0.0, yaw); //リンクを回転させることができる
    static_tf.transform.rotation.x = quat.x();
    static_tf.transform.rotation.y = quat.y();
    static_tf.transform.rotation.z = quat.z();
    static_tf.transform.rotation.w = quat.w();

    static_tf_broadcaster.sendTransform(static_tf); //TFを発行
};