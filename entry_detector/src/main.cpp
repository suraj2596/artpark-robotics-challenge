#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/line_descriptor.hpp>

#include <Eigen/Dense>

double fx = 1206.8897719532354;
double fy = 1206.8897719532354;
double cx = 960.5;
double cy = 540.5;

Eigen::Vector3d backproject_to_3d(const cv::Point2f &p, const double &depth)
{
    Eigen::Vector3d X;
    X(0) =(p.x - cx)*depth/fx;
    X(1) = (p.y - cy)*depth/fy;
    X(2) = depth;
    return X;
}

double distance_between_lines(const std::pair<Eigen::Vector3d, Eigen::Vector3d> &l1,
    const std::pair<Eigen::Vector3d, Eigen::Vector3d> &l2)
{
    //if((std::fabs(l1.second(0)/l2.second(0)
    //    - l1.second(1)/l2.second(1)) < 0.01)
    //    && (std::fabs(l1.second(0)/l2.second(0)
    //        - l1.second(2)/l2.second(2)) < 0.01))
    //        return ((l2.first - l1.first).cross(l1.second.normalized())).norm();

    Eigen::Vector3d ground_normal(0, 1, 0);

    if(((std::fabs(l1.second.dot(l2.second)) - 1) < 0.001) && 
            (std::fabs(l1.second.dot(ground_normal)) - 1) < 0.001)
        return ((l2.first - l1.first).cross(l1.second)).norm();

    else
        return 0;
}

bool find_entry_lines(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &lines_3d,
    std::pair<int, int> &entry_line_ids)
{
    for(int i = 0; i < lines_3d.size(); i++)
    {
        for(int j = i+1; j < lines_3d.size(); j++)
        {
            double d = distance_between_lines(lines_3d[i], lines_3d[j]);
            if(d >= 0.90 && d <= 1.25) 
            {
                ROS_INFO_STREAM("Entry found. Esimated width: " << d);
                entry_line_ids = std::make_pair(i, j);
                return true;
            }
        }
    }
    return false;
}

void draw_line(cv::Mat &img, const cv::line_descriptor::KeyLine &line, bool entry=false)
{
    cv::Point pt1 = cv::Point2f(line.startPointX, line.startPointY);
    cv::Point pt2 = cv::Point2f(line.endPointX, line.endPointY);
    if(entry)
        cv::line(img, pt1, pt2, cv::Scalar(255,0,0), 3);
    else
        cv::line(img, pt1, pt2, cv::Scalar(0,255,0), 3);
}

void handleImages(const sensor_msgs::ImageConstPtr &image_msg,
    const sensor_msgs::ImageConstPtr &depth_msg)
{
    cv_bridge::CvImagePtr img_ptr;
    try
    {
        img_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception :%s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr depth_ptr;
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg);
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception %s", e.what());
        return;
    }

    cv::Mat depth_img;
    depth_ptr->image.convertTo(depth_img, CV_8UC1, 255, 0);

    cv::Mat mask = cv::Mat::ones(depth_img.size(), CV_8UC1);
    cv::Ptr<cv::line_descriptor::LSDDetector> ld = cv::line_descriptor::LSDDetector::createLSDDetector();
    std::vector<cv::line_descriptor::KeyLine> lines;
    cv::Mat output_img = img_ptr->image.clone();
    ld->detect(depth_img, lines, 2, 1, mask);

    // 3D line - (point, direction)
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> lines_3d;

    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::line_descriptor::KeyLine kl = lines[i];
        if(kl.octave == 0)
        {
            cv::Point pt1 = cv::Point2f(kl.startPointX, kl.startPointY);
            cv::Point pt2 = cv::Point2f(kl.endPointX, kl.endPointY);

            double d1 = depth_ptr->image.at<float>(pt1);
            double d2 = depth_ptr->image.at<float>(pt2);

            Eigen::Vector3d P1 = backproject_to_3d(pt1, d1);
            Eigen::Vector3d P2 = backproject_to_3d(pt2, d2);

            lines_3d.push_back(std::make_pair(P1, (P2 - P1).normalized()));

            draw_line(output_img, lines[i]);
        }
    }

    std::pair<int, int> entry_line_ids;
    if(find_entry_lines(lines_3d, entry_line_ids))
    {
        Eigen::Vector3d P1 = lines_3d[entry_line_ids.first].first;
        Eigen::Vector3d P2 = lines_3d[entry_line_ids.second].first;

        Eigen::Vector3d P((P1(0) + P2(0)/2), (P1(1) + P2(1))/2, (P1(2) + P2(2))/2);
        cv::Point2f entry_2d;
        entry_2d.x = fx*P(0)/P(2) + cx;
        entry_2d.y = fy*P(1)/P(2) + cy;
        cv::circle(output_img, entry_2d, 10, cv::Scalar(0, 0, 255), -1);

        draw_line(output_img, lines[entry_line_ids.first], true);
        draw_line(output_img, lines[entry_line_ids.second], true);
    }

    else
        ROS_INFO_STREAM("No entry found!");

    cv::Mat output_resized;
	cv::resize(output_img, output_resized, cv::Size(1080,720));
    cv::imshow("output", output_resized);
    cv::waitKey(3);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "entry_detector");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub, depth_sub, 10);
    sync.registerCallback(boost::bind(&handleImages, _1, _2));

    ros::spin();
}
