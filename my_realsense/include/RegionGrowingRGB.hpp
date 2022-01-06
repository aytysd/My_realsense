#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <../include/PCL_common.hpp>
#include <pcl/visualization/pcl_visualizer.h>


class RegionGrowing_RGB
{
private:

    ros::NodeHandle nh_;
    ros::NodeHandle np_;

    ros::Publisher pub_clusters_;
    ros::Publisher pub_cluster_indices_;

    ros::Subscriber sub_Downsampled_;


    pcl::search::KdTree<PointT>::Ptr tree_;
    pcl::RegionGrowingRGB<PointT, pcl::Normal> clustering_;


    double distance_threshold_;
    double diff_threshold_;
    double merge_threshold_;


    std::vector<pcl::PointIndices> cluster_indices_;
    PointCloud::Ptr cloud_ptr_;

    Eigen::Vector4f min_pt_, max_pt_;
   

    void Downsampled_callback(const sensor_msgs::PointCloud2 & msg);

    void clustering_config
    (
        PointCloud::Ptr & cloud_ptr,
        double distance_threshold,
        double diff_threshold,
        double merge_threshold,
        double min_size
    );


    visualization_msgs::MarkerArray marker_array_;
    void make_Marker_array
    (
        PointCloud::Ptr & cloud_ptr,
        std::vector<pcl::PointIndices>& cluster_indices,
        Eigen::Vector4f &max_pt,
        Eigen::Vector4f &min_pt,
        double r,
        double g,
        double b,
        double a
    );


    void reset( void );

    


public:
    RegionGrowing_RGB():
        np_( "~" ),
        sub_Downsampled_( nh_.subscribe( "/Downsampled", 10, &RegionGrowing_RGB::Downsampled_callback, this ) ),
        pub_clusters_( nh_.advertise<visualization_msgs::MarkerArray>( "/marker_array_", 100 ) ),
        cloud_ptr_( new PointCloud )
    {

        np_.getParam( "distance_threshold", distance_threshold_ );
        np_.getParam( "diff_threshold", diff_threshold_ );
        np_.getParam( "merge_threshold", merge_threshold_ );


        tree_.reset( new pcl::search::KdTree<PointT> );
        cluster_indices_.clear();


    };
        

};


