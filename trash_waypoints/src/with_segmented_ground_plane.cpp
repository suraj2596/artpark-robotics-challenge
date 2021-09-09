#include <iostream>

#include <ros/ros.h>

#include<math.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
// #include "cluster.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>

// ROS publishers
ros::Publisher closest_cluster_pub;
ros::Publisher segmented_ground_pub;

// params for finding closest clusters
static float MAX_CLUSTER_SIZE;
static int MIN_CLUSTER_SIZE;
static float CLUSTER_TOLERANCE;

// params for removing ground plane
static float MAX_HEIGHT;
static float FLOOR_MAX_ANGLE;
int PLANAR_DIST_THRESHOLD;
float PLANAR_MINIMUM_PERCENTAGE;

// params for ROS
// static const char* TOPIC_TO_SUBSCRIBE_TO = "/rtabmap/cloud_ground";
static std::string TOPIC_TO_SUBSCRIBE_TO;

static std::string TOPIC_TO_PUBLISH;

// pointer to node handler
ros::NodeHandle *nhTemp;

tf2_ros::Buffer tfBuffer;

void pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){
    // lets put this inside of ros params 
    int PLANAR_DIST_THRESHOLD;
    float PLANAR_MINIMUM_PERCENTAGE;

    // Get segmentation ready
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // model coeffs object
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // indices of inliers
    pcl::SACSegmentation<pcl::PointXYZ> seg; // seg object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(PLANAR_DIST_THRESHOLD);

    int original_size = input_cloud->height * input_cloud->width;
    int n_planes = 0;
    while ((input_cloud->height * input_cloud->width) > (original_size * PLANAR_MINIMUM_PERCENTAGE / 100)){

        // Fit a plane
        seg.setInputCloud(input_cloud);
        seg.segment(*inliers, *coefficients);

        // Check result
        if (inliers->indices.size() == 0)
            break;

        // Extract inliers
        extract.setInputCloud(input_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true); // everything BUT the inliers remains
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        extract.filter(temp_cloud);
        input_cloud->swap(temp_cloud);

        n_planes++;
    }
    std::cout << "FOUND " << n_planes << " PLANES" << std::endl;
}

static void RemoveGroundPlane (const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr out_filtered_ground_cloud)
{
    pcl::SACSegmentation<pcl::PointXYZ> seg; //segmentation object
    pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices); //to store the indices of the inliers
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //coeffs in the form ax+by+cz+d=0

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // want to find a perpendicular plane to the z axis
    seg.setMethodType(pcl::SAC_RANSAC); // RANSAC robust estimator
    seg.setMaxIterations(100);
    seg.setAxis(Eigen::Vector3f(0, 0, 1)); // setting z axis
    seg.setEpsAngle(FLOOR_MAX_ANGLE); // max angular deviation

    seg.setDistanceThreshold(MAX_HEIGHT);// floor distance
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(in_cloud);
    seg.segment(*inlier_indices, *coefficients);

    if (inlier_indices->indices.size() == 0)
    {
        std::cout << "Ground plane not found" << std::endl;
    }

    // Removing floor points from point cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud);
    extract.setIndices(inlier_indices);
    extract.setNegative(true);  // to remove the inliers AKA ground points
    extract.filter(*out_filtered_ground_cloud);
}

//static void FindClosestCluster (const pcl::PointCloud<pcl::PointXYZ>::Ptr in_point_cloud,
//                              pcl::PointCloud<pcl::PointXYZ>::Ptr out)
//{
//  // creating a KDTree object for the search method of the extraction algo
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//  tree->setInputCloud(in_point_cloud);
//
//  // vector of point indices --> one instance of point indices contains the PointIndices
//  std::vector<pcl::PointIndices> cluster_indices;
//  // pointcloud euclidean cluster extraction object
//  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//  // distance > CLUSTER_TOLERANCE meters then not considered in the cluster
//  ec.setClusterTolerance (CLUSTER_TOLERANCE);
//  // cluster MUST be greater than or equal to MIN_CLUSTER_SIZE points
//  ec.setMinClusterSize (MIN_CLUSTER_SIZE);
//  // cluster MUST be less than or equal to MAX_CLUSTER_SIZE points
//  ec.setMaxClusterSize (MAX_CLUSTER_SIZE);
//  ec.setSearchMethod (tree);
//  ec.setInputCloud (in_point_cloud);
//  // extracted the clusters out of the point cloud and saved the indices in the cluster indices
//  ec.extract (cluster_indices);
//
//    // pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centroids_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    int cluster_count = 0;
//    // iterating theough each of clusters
//    for (auto &cluster : cluster_indices) {
//        pcl::PointXYZ cluster_centroid;
//        // centroid object
//        pcl::CentroidPoint<pcl::PointXYZ> centroid;
//        for (auto &indx : cluster.indices) {
//            // iterating through each Unfilteredindx and adding it to the centroid object
//            centroid.add(in_point_cloud->points[indx]);
//        }
//        // fetching centroid with .get
//        centroid.get(cluster_centroid);
//        // insert centroid to cluster_centroids_vector
//      // std::cout << cluster_centroid << std::endl;
//        out->push_back(cluster_centroid);
//        cluster_count += 1;
//    }
//    printf ("Number of clusters found: %d \n", cluster_count);
//}

float getDistance(int x1, int x2, int y1, int y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

static void FindClosestCluster (const pcl::PointCloud<pcl::PointXYZ>::Ptr in_point_cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    // creating a KDTree object for the search method of the extraction algo
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(in_point_cloud);

    // vector of point indices --> one instance of point indices contains the PointIndices
    std::vector<pcl::PointIndices> cluster_indices;
    // pointcloud euclidean cluster extraction object
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // distance > CLUSTER_TOLERANCE meters then not considered in the cluster
    ec.setClusterTolerance (CLUSTER_TOLERANCE);
    // cluster MUST be greater than or equal to MIN_CLUSTER_SIZE points
    ec.setMinClusterSize (MIN_CLUSTER_SIZE);
    // cluster MUST be less than or equal to MAX_CLUSTER_SIZE points
    ec.setMaxClusterSize (MAX_CLUSTER_SIZE);
    ec.setSearchMethod (tree);
    ec.setInputCloud (in_point_cloud);
    // extracted the clusters out of the point cloud and saved the indices in the cluster indices
    ec.extract (cluster_indices);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centroids_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    int cluster_count = 0;

    // iterating through each of clusters
    // std::vector<pcl::PointXYZ> centroids;

    // list of trash centroids (saved on rosparam)
    std::vector<float> centroids;

    nhTemp->getParam("/TRASH_CENTROIDS", centroids);

    geometry_msgs::TransformStamped transformStamped;

    for (auto &cluster : cluster_indices) {
        pcl::PointXYZ cluster_centroid;
        // centroid object
        pcl::CentroidPoint<pcl::PointXYZ> centroid;
        for (auto &indx : cluster.indices) {
            // iterating through each Unfilteredindx and adding it to the centroid object
            centroid.add(in_point_cloud->points[indx]);
        }
        // fetching centroid with .get
        centroid.get(cluster_centroid);
        // insert centroid to cluster_centroids_vector
        std::cout << cluster_centroid << " " << typeid(cluster_centroid).name() << std::endl;
        out->push_back(cluster_centroid);

        try
        {
            transformStamped = tfBuffer.lookupTransform("odom", "camera_link", ros::Time(0));
        }
        catch(tf2::TransformException &exception)
        {
            ROS_ERROR("%s", exception.what());
        }

        // transformation matrix
        Eigen::Affine3d transform_matrix = tf2::transformToEigen(transformStamped);

        // find transformed centroid
        Eigen::Vector4d new_centroid(cluster_centroid.x, cluster_centroid.y ,cluster_centroid.z, 1);
        Eigen::Vector4d transformed_centroid = transform_matrix * new_centroid;

        // include found centroid
        if (transformed_centroid(2) < 0.5)
        {
            // Checking if there exists a centroid within 0.1 m of the detected centroid.
            // If not, append to the list. Else, skip.
            // NOTE: declared TRASH CENTROIDS AS [] - previously it was: [0,0,0]

            int num_waypoints = centroids.size();
            if(num_waypoints == 0)
            {
                centroids.push_back(transformed_centroid(0));
                centroids.push_back(transformed_centroid(1));
                centroids.push_back(transformed_centroid(2));   
            }
            else
            {
                int totalCentroids = num_waypoints / 3; // Total number of centroids
                int isClose = 0;                       // To check if the detected centroid is new or not

                for(int i = 0 ; i < num_waypoints ; i += 3)
                {
                    if(getDistance(centroids[0], transformed_centroid[0], centroids[1], transformed_centroid[1]) < 0.1)
                    {
                        isClose += 1;
                    }
                }
                if(isClose == totalCentroids)
                {
                    ;
                }
                else
                {
                    for(int i = 0 ; i < num_waypoints ; i += 3)
                    {
                        if(getDistance(centroids[0], transformed_centroid[0], centroids[1], transformed_centroid[1]) < 0.1)
                        {
                            ;
                        }
                        else
                        {
                            centroids.push_back(transformed_centroid(0));
                            centroids.push_back(transformed_centroid(1));
                            centroids.push_back(transformed_centroid(2));
                        }
                    }
                }
            }
        }

        cluster_count += 1;
    }

    nhTemp->setParam("/TRASH_CENTROIDS", centroids);
    printf ("Number of clusters found: %d \n", cluster_count);
}

static void VelodyneCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud_msg, *original_point_cloud);
    
    // Removing ground plane, if subscribing to cloud_ground isn't working optimially (segmenting out
    // the trash objects as they're pretty small, adjusting the above params and running this code should work)
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_without_ground(new pcl::PointCloud<pcl::PointXYZ>);
    RemoveGroundPlane(original_point_cloud, point_cloud_without_ground);    
    // pointCloudCb(point_cloud_without_ground);
    // Finding closest Cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centroids_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    FindClosestCluster(point_cloud_without_ground, cluster_centroids_cloud);
    // FindClosestCluster(original_point_cloud, cluster_centroids_cloud);

    // std::cout << closest_y_dist << std::endl;

    // Convert to ROS data type --> pc to ros msg pc2 
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cluster_centroids_cloud, cloud_publish);
    // output the centriods of the trash objects to give as waypoints to the bot
    cloud_publish.header = cloud_msg->header;
    closest_cluster_pub.publish(cloud_publish);

    // Convert to ROS data type --> pc to ros msg pc2 
    sensor_msgs::PointCloud2 cloud_publish_2;
    pcl::toROSMsg(*point_cloud_without_ground, cloud_publish_2);
    // output the centriods of the trash objects to give as waypoints to the bot
    cloud_publish_2.header = cloud_msg->header;
    segmented_ground_pub.publish(cloud_publish_2);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "with_segmented_ground_plane");
    ros::NodeHandle nh;
    nhTemp = &nh;

    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(0.1).sleep();

    nh.getParam("/MAX_HEIGHT", MAX_HEIGHT);
    nh.getParam("MAX_CLUSTER_SIZE", MAX_CLUSTER_SIZE);
    nh.getParam("MIN_CLUSTER_SIZE", MIN_CLUSTER_SIZE); 
    nh.getParam("CLUSTER_TOLERANCE", CLUSTER_TOLERANCE);

    nh.getParam("MAX_HEIGHT", MAX_HEIGHT); 
    nh.getParam("FLOOR_MAX_ANGLE", FLOOR_MAX_ANGLE);

    nh.getParam("TOPIC_TO_SUBSCRIBE_TO", TOPIC_TO_SUBSCRIBE_TO); 

    nh.getParam("TOPIC_TO_PUBLISH", TOPIC_TO_PUBLISH); 
    // ROS subscriber for the input point cloud
    // topic, q_size, callback
    // ros::Subscriber sub = nh.subscribe ("/cloud", 1, VelodyneCallback);
    ros::Subscriber sub = nh.subscribe (TOPIC_TO_SUBSCRIBE_TO, 1, VelodyneCallback);

    //q_size = 1 as RTABMAP is running at 1 Hz

    // ros::Subscriber sub = nh.subscribe ("/lane_line_equations", 1, LaneLineCallback);

    // ROS publisher for the output point cloud
    // (topic, q_size)
    closest_cluster_pub = nh.advertise<sensor_msgs::PointCloud2> (TOPIC_TO_PUBLISH, 1); // publishing point cloud 2
    segmented_ground_pub = nh.advertise<sensor_msgs::PointCloud2> ("/no_ground", 1); // publishing point cloud 2.1

    // spin
    ros::spin();
}