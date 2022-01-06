#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <../include/PCL_common.hpp>
#include <pcl/visualization/pcl_visualizer.h>


class RegionGrowing
{
private:

    ros::NodeHandle nh_;
    ros::NodeHandle np_;

    ros::Publisher pub_centroid_point_;
    ros::Publisher pub_clusters_;

    ros::Subscriber sub_Downsampled_;


    pcl::search::KdTree<PointT>::Ptr tree_;
    pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation_;
    pcl::RegionGrowing<PointT, pcl::Normal> clustering_;



    double radius_;
    int number_of_neighbours_;
    double smoothness_threshold_;
    double curvature_threshold_;

    std::vector<pcl::PointIndices> cluster_indices_;
    PointCloud::Ptr cloud_ptr_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    Eigen::Vector4f min_pt_, max_pt_;
   

    void Downsampled_callback(const sensor_msgs::PointCloud2 & msg);

    void clustering_config
    (
        PointCloud::Ptr & cloud_ptr,
        double radius,
        int NumberOfNeighbours,
        pcl::PointCloud<pcl::Normal>::Ptr input_normals,
        double smoothness_Threshold,
        double CurvatureThreshold,
        double min_size,
        double max_size
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
    RegionGrowing():
        np_( "~" ),
        sub_Downsampled_( nh_.subscribe( "/Downsampled", 10, &RegionGrowing::Downsampled_callback, this ) ),
        pub_clusters_( nh_.advertise<visualization_msgs::MarkerArray>( "/marker_array_", 100 ) ),
        pub_centroid_point_( nh_.advertise<geometry_msgs::Point>( "/centroid_point", 10 ) ),
        cloud_ptr_( new PointCloud ),
        normals_( new pcl::PointCloud<pcl::Normal> ),
        viewer_( new pcl::visualization::PCLVisualizer( "Normals" ) )
    {

        np_.getParam( "radius", radius_ );
        np_.getParam( "number_of_neighbours", number_of_neighbours_ );
        np_.getParam( "smoothness_threshold", smoothness_threshold_ );
        np_.getParam( "curvature_threshold", curvature_threshold_ );

        tree_.reset( new pcl::search::KdTree<PointT> );
        cluster_indices_.clear();


    };
        

};


