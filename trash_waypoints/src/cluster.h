#pragma once

static void RemoveGroundPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr out_filtered_ground_cloud)
/****
	* @brief This function filters out the ground plane from an unfiltered point cloud
	* @param in_cloud Unfiltered input point cloud from Lidar data
	* @param out_filtered_ground_cloud output point cloud with the ground plane filtered out
****/

static void GetPointsInBetweenLaneLine (float m1,
										float m2,
										float c1,
										float c2,
										pcl::PointCloud<pcl::PointXYZ>::Ptr in_point_cloud,
										pcl::PointCloud<pcl::PointXYZ>::Ptr out_filtered_cloud)
/****
	* @brief This function gets the points in between of the lane lines, only getting the points in front of the car
	* @param m1 slope of line 1
	* @param m2 slope of line 2
	* @param c1	y int of line 1
	* @param c2 y int of line 2
	* @param in_point_cloud input unfiltered point cloud
	* @param out_filtered_cloud output filtered cloud, consisting only of the area in front of the car and in between the lane lines
****/

static void FindClosestCluster (const pcl::PointCloud<pcl::PointXYZ>::Ptr in_point_cloud,
								float* closest_y_dist,
								pcl::PointCloud<pcl::PointXYZ>::Ptr out)
/****
	* @brief This function finds the closest cluster to the car and returns the distance from the 
	* @param in_point_cloud
	* @param closest_y_dist
	* @param out
****/

static void VelodyneCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
/****
	* @brief This function reads the point cloud from the ros topic and publishes the minimum distance to the object
	* @param cloud_msg ros PointCloud2 msg input
****/
