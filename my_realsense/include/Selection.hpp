#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <../include/PCL_common.hpp>


class Selection
{
private:

    PointCloud::Ptr& src_ptr_;
    std::vector<pcl::PointIndices> inliers_;
    
    double ideal_r_, ideal_g_, ideal_b_;
    double ideal_size_;

    size_t colour_index_num_;
    size_t size_index_num_;

    double size_diff_score_;
    double colour_diff_score_;

    void extract_ideal_size();
    void extract_ideal_colour();


public:
    Selection
    (
        PointCloud::Ptr& src_ptr,
        std::vector<pcl::PointIndices> inliers,
        double ideal_size,
        double ideal_r,
        double ideal_g,
        double ideal_b
    ):
        src_ptr_( src_ptr ),
        inliers_( inliers ),
        ideal_size_( ideal_size ),
        ideal_r_( ideal_r ),
        ideal_g_( ideal_g ),
        ideal_b_( ideal_b )
    {
        extract_ideal_size();
        extract_ideal_colour();
    };

    geometry_msgs::Point selected_centroid_;
    visualization_msgs::Marker select( void );


};

enum class object_type
{
    cube,
    ball
};

