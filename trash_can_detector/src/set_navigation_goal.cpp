#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <math.h>
tf2_ros::Buffer tfBuffer;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void setNavGoal(float x, float y, geometry_msgs::Quaternion orientation, MoveBaseClient &ac)
{
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved");
    else
        ROS_INFO("The base failed to move for some reason");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle nh;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // set nav goals for waypoints
    std::vector<float> waypoints;
    nh.getParam("/WAYPOINTS", waypoints);

    int n = waypoints.size();
    for (int i=0; i<n; i+=2) {

        for(int j = 0; j < 20; j ++)
        {
            move_base_msgs::MoveBaseGoal goal;

            // transform robots position to map frame
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0));
            }
            catch(tf2::TransformException &exception)
            {
                ROS_ERROR("%s", exception.what());
            }

            // transformation matrix
            Eigen::Affine3d transform_matrix = tf2::transformToEigen(transformStamped);

            // transform origin
            Eigen::Vector4d origin(0, 0, 0, 1);
            Eigen::Vector4d robot_position = transform_matrix * origin;
            float x = (j+1)*waypoints[i]/20;
            float y = (j+1)*waypoints[i+1]/20;
            // find orientation using robot position and trash item position
            float roll = 0.0, pitch = 0.0;
            float yaw = atan(y - robot_position(1) / x - robot_position(0));
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);

            ROS_INFO("Hereeeeee %f %f", robot_position(0), robot_position(1));
            ROS_INFO("Destination %f %f", x, y);
            setNavGoal(x,y, quat_msg, ac);
        }

    }
}