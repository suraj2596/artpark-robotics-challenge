#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "nav_msgs/Odometry.h"
#include <vector>

#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_eigen/tf2_eigen.h>

double fx = 1206.8897719532354;
double fy = 1206.8897719532354;
double cx = 960.5;
double cy = 540.5;

Eigen::Vector3d centroid(0, 0, 0);
Eigen::Vector4d transformed_centroid(0, 0, 0, 1);

// pointer to nh
ros::NodeHandle *nhTemp;

tf2_ros::Buffer tfBuffer;

Eigen::Vector3d backproject_to_3d(const cv::Point2f &p, const double &depth)
{
    Eigen::Vector3d X;
    X(0) =(p.x - cx)*depth/fx;
    X(1) = (p.y - cy)*depth/fy;
    X(2) = depth;
    return X;
}

// find 3D world position of centroid of trash can
void find3DWorldPosition(const sensor_msgs::ImageConstPtr &depth_msg,
                         const sensor_msgs::ImageConstPtr &mask_msg,
                         const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    Eigen::Vector3d zero_vec(0, 0, 0);

    // if centroid is not previously calculated, calculate it
    if (centroid == zero_vec)
    {
        cv_bridge::CvImageConstPtr depth_ptr;
        try
        {
            depth_ptr = cv_bridge::toCvCopy(depth_msg);
        }
        catch(cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception :%s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr mask_ptr;
        try
        {
            mask_ptr = cv_bridge::toCvCopy(mask_msg);
        }
        catch(cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception :%s", e.what());
            return;
        }

        ROS_INFO("%d %d\n", depth_ptr->image.rows, depth_ptr->image.cols);
        ROS_INFO("%d %d\n", mask_ptr->image.rows, mask_ptr->image.cols);

        cv::patchNaNs(depth_ptr->image, 0);

        int offset = 5;
        std::vector<Eigen::Vector3d> points;
        for (int i=offset; i<depth_ptr->image.rows - offset; i++)
        {
            for (int j=offset; j<depth_ptr->image.cols - offset; j++)
            {
                if (mask_ptr->image.at<float>(i, j))
                {
                    double depth = depth_ptr->image.at<float>(i, j);
                    cv::Point point = cv::Point2f(i, j);
                    Eigen::Vector3d P = backproject_to_3d(point, depth);
                    points.push_back(P);
                }
            }
        }

        int n = points.size();

        // threshold value
        if (n > 70000)
        {
            for (auto &point : points)
                centroid += point / n;
        }

        ROS_INFO("n = %d\n", n);
        ROS_INFO("Centroid found at %f %f %f", centroid(0), centroid(1), centroid(2));

        // convert centroid to world coordinates

//        tf2_ros::Buffer tfBuffer;
//        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;

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
        Eigen::Vector4d new_centroid(centroid(0), centroid(1), centroid(2), 1);
        transformed_centroid = transform_matrix * new_centroid;

        ROS_INFO("Transformed Centroid found at %f %f %f", transformed_centroid(0), transformed_centroid(1), transformed_centroid(2));
    }

    // Construct a vector
    std::vector<float> cent(3);
    cent[0] = transformed_centroid(0);
    cent[1] = transformed_centroid(1);
    cent[2] = transformed_centroid(2);

    // Set and get a map of strings
    nhTemp->setParam("/TRASH_CAN_CENTROID", cent);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trash_can_detector");

    ros::NodeHandle nh;
    nhTemp = &nh;

    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(0.1).sleep();

    // subscribe to topic /camera/depth/image_raw
    // type: sensor_msgs/Image
    // returns: image information (with depth info) from RGBD sensor
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/color/image_raw", 1);

    // subscribe to topic /histogram_segmentation/mask
    // type: sensor_msgs/Image
    // returns: 2D mask of detected trash can
    message_filters::Subscriber<sensor_msgs::Image> mask_sub(nh, "/histogram_segmentation/mask", 1);

    // subscribe to topic /odom
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 1);

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, nav_msgs::Odometry> sync(image_sub, mask_sub, odom_sub, 10);
    sync.registerCallback(boost::bind(&find3DWorldPosition, _1, _2, _3));

    // spin
    ros::spin();

    return 0;
}