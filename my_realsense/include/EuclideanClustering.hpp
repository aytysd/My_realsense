#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <../include/PCL_common.hpp>

class EuclideanClustering
{
private:

    ros::NodeHandle nh_;
    ros::NodeHandle np_;

    double max_r_;
    double max_g_;
    double max_b_;
    double min_r_;
    double min_g_;
    double min_b_;

    

    ros::Publisher pub_centroid_point_;
    ros::Publisher pub_clusters_;
    ros::Publisher pub_ideal_;
    
    ros::Subscriber sub_Downsampled_;

    double max_ball_radius_;
    double min_ball_radius_;

    double max_size_x_;
    double max_size_y_;
    double max_size_z_;
    double min_size_x_;
    double min_size_y_;
    double min_size_z_;

    double ideal_size__;
    double ideal_r__, ideal_g__, ideal_b__;

    pcl::search::KdTree<PointT>::Ptr tree_;
    pcl::EuclideanClusterExtraction<PointT> ec_;

    std::vector<pcl::PointIndices> cluster_indices_;
    PointCloud::Ptr cloud_ptr_;

    Eigen::Vector4f min_pt_, max_pt_;

    int object_type_;
    

    void Downsampled_callback(const sensor_msgs::PointCloud2 & msg);
    bool my_condition( const PointT& seedPoint, const PointT& canditate, float squaredDistance );


    void clustering_config
    (
        PointCloud::Ptr & cloud_ptr,
        double Tolerance,
        double min_size,
        double max_size
    );

    bool check_size( 
        PointCloud::Ptr & cloud_ptr,
        pcl::PointIndices cluster_index,
        double max_size_x, 
        double max_size_y, 
        double max_size_z,
        double min_size_x,
        double min_size_y,
        double min_size_z
    );

    bool check_colour(
        PointCloud::Ptr & cloud_ptr,
        pcl::PointIndices cluster_index,
        double min_r, 
        double min_g,
        double min_b,
        double max_r,
        double max_g,
        double max_b
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
    EuclideanClustering():
        np_( "~" ),
        sub_Downsampled_( nh_.subscribe( "/Downsampled", 10, &EuclideanClustering::Downsampled_callback, this ) ),
        pub_clusters_( nh_.advertise<visualization_msgs::MarkerArray>( "/marker_array_", 100 ) ),
        pub_centroid_point_( nh_.advertise<geometry_msgs::Point>( "/centroid_point", 10 ) ),
        pub_ideal_( nh_.advertise<visualization_msgs::Marker>( "/ideal", 100 ) ),
        cloud_ptr_( new PointCloud )
    {
        np_.getParam( "max_ball_radius", max_ball_radius_ );
        np_.getParam( "min_ball_radius", min_ball_radius_ ); 
        np_.getParam( "max_r", max_r_ );
        np_.getParam( "max_g", max_g_ );
        np_.getParam( "max_b", max_b_ );
        np_.getParam( "min_r", min_r_ );
        np_.getParam( "min_g", min_g_ );
        np_.getParam( "min_b", min_b_ );
        np_.getParam( "ideal_r", ideal_r__ );
        np_.getParam( "ideal_g", ideal_g__ );
        np_.getParam( "ideal_b", ideal_b__ );
        np_.getParam( "ideal_size", ideal_size__ );
        np_.getParam( "type", object_type_ );


        max_size_x_ = max_ball_radius_;
        max_size_y_ = max_ball_radius_;
        max_size_z_ = max_ball_radius_;
        min_size_x_ = min_ball_radius_;
        min_size_y_ = min_ball_radius_;
        min_size_z_ = min_ball_radius_;

        tree_.reset( new pcl::search::KdTree<PointT>() );
        cluster_indices_.clear();

    };
        

};


