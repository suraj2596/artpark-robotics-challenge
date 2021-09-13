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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
tf2_ros::Buffer tfBuffer;

// void getThereSlowly(float goalX, float goalY, float currX, float currY)
// {
//         for(int j = 0; j < 10; j ++)
//         {
//             move_base_msgs::MoveBaseGoal goal;

//             // transform robots position to map frame
//             geometry_msgs::TransformStamped transformStamped;
//             try
//             {
//                 transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
//             }
//             catch(tf2::TransformException &exception)
//             {
//                 ROS_ERROR("%s", exception.what());
//             }

//             // transformation matrix
//             Eigen::Affine3d transform_matrix = tf2::transformToEigen(transformStamped);

//             // transform origin
//             Eigen::Vector4d origin(0, 0, 0, 1);
//             Eigen::Vector4d robot_position = transform_matrix * origin;
//             x = (j+1)*waypoints[i];
//             y = (j+1)*waypoints[i+1]
//             // find orientation using robot position and trash item position
//             float roll = 0.0, pitch = 0.0;
//             float yaw = atan(y - robot_position(1) / x - robot_position(0));
//             tf2::Quaternion quat_tf;
//             quat_tf.setRPY(roll, pitch, yaw);
//             geometry_msgs::Quaternion quat_msg;
//             tf2::convert(quat_tf, quat_msg);

//             ROS_INFO("Hereeeeee %f %f", waypoints[i], waypoints[i+1]);
//             setNavGoal(waypoints[i], waypoints[i+1], 0, ac);
//         }

// }

void setNavGoal(float x, float y, geometry_msgs::Quaternion orientation, MoveBaseClient &ac)
{
    move_base_msgs::MoveBaseGoal goal;

    // transform robots position to map frame
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch(tf2::TransformException &exception)
    {
        ROS_ERROR("%s", exception.what());
    }

    // transformation matrix
    // Eigen::Affine3d transform_matrix = tf2::transformToEigen(transformStamped);

    // transform origin
    // Eigen::Vector4d origin(0, 0, 0, 1);
    // Eigen::Vector4d robot_position = transform_matrix * origin;

    // ROS_INFO("Robot position: %f %f", robot_position(0), robot_position(1));
    // ROS_INFO("Other position: %f %f", x, y);

    // find orientation using robot position and trash item position
    // float roll = 0.0, pitch = 0.0;
    // float yaw = atan(y - robot_position(1) / x - robot_position(0));
    // tf2::Quaternion quat_tf;
    // quat_tf.setRPY(roll, pitch, yaw);
    // geometry_msgs::Quaternion quat_msg;
    // tf2::convert(quat_tf, quat_msg);

    float dist = 0.4;
    // stop 0.65m before goal (based on quadrants)
    if (x >= 0 && y >= 0)
        x -= dist, y -= dist;
    else if (x < 0 && y >= 0)
        x += dist, y -=dist;
    else if (x < 0 && y < 0)
        x += dist, y +=dist;
    else
        x -= dist, y +=dist;

    //we'll send a goal to the robot
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = orientation;

    ROS_INFO("Sending goal");
    ROS_INFO("%f %f", x, y);
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

    ros::Publisher pick_topic = nh.advertise<std_msgs::Float64>("/artpark/grip_signal", 1000);
    ros::Publisher drop_topic = nh.advertise<std_msgs::Float64>("/artpark/drop_signal", 1000);
    ros::Publisher stem_topic = nh.advertise<std_msgs::Float64>("/artpark/stem_ctrl/command", 10);

    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(0.1).sleep();

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // get trash can centroid
    std::vector<float> trash_can_centroid;
    nh.getParam("/TRASH_CAN_CENTROID", trash_can_centroid);

    // set nav goals for waypoints
    std::vector<float> waypoints;
    nh.getParam("/TRASH_CENTROIDS", waypoints);
    //for(int i = 0; i < waypoints.size(); i++)
    //    ROS_INFO("waypoints vec %f", waypoints[i]);

    int n = waypoints.size();
    int steps = 1;
    for (int i=0; i<n; i+=3) {
        move_base_msgs::MoveBaseGoal goal;

        

        // transform robots position to map frame
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
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
        float x = (waypoints[i])/steps;
        float y = (waypoints[i+1])/steps;
        // float x = (j+1)*(waypoints[i] - robot_position[0])/steps;
        // float y = (j+1)*(waypoints[i+1] - robot_position[1])/steps;
        // find orientation using robot position and trash item position
        // float roll = 0.0, pitch = 0.0;
        // float yaw = atan(y - robot_position(1) / x - robot_position(0));
        // tf2::Quaternion quat_tf;
        // quat_tf.setRPY(roll, pitch, yaw);
        // geometry_msgs::Quaternion quat_msg;
        // tf2::convert(quat_tf, quat_msg);
        float roll = 0.0, pitch = 0.0;
        float yaw = 0;
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);

        ROS_INFO("Hereeeeee %f %f", robot_position(0), robot_position(1));
        setNavGoal(x, y, quat_msg, ac);
        ros::Duration(1).sleep();


        // navigate to waypoint
        // ROS_INFO("Trash item at %f %f", waypoints[i], waypoints[i+1]);
        // setNavGoal(waypoints[i], waypoints[i+1], 0, ac);

        try
        {
            transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
        }
        catch(tf2::TransformException &exception)
        {
            ROS_ERROR("%s", exception.what());
        }

        
        // transform origin
        robot_position = transform_matrix * origin;
        
        roll = 0.0, pitch = 0.0;
        yaw = atan(y - robot_position(1) / x - robot_position(0));
        quat_tf.setRPY(roll, pitch, yaw);
        tf2::convert(quat_tf, quat_msg);
        setNavGoal(robot_position(0), robot_position(1), quat_msg, ac);
        ros::Duration(1).sleep();


        // pick/drop
        std_msgs::Float64 zero;
        zero.data = 0;
        pick_topic.publish(zero);
        ros::Duration(5).sleep();
        drop_topic.publish(zero);
        ros::Duration(5).sleep();

        stem_topic.publish(zero);
        ros::Duration(0.5).sleep();


        // go to trash can
        ROS_INFO("Trash can at %f %f", trash_can_centroid[0], trash_can_centroid[1]);

        for(int j = 0; j < steps; j ++)
        {
            move_base_msgs::MoveBaseGoal goal;

            // transform robots position to map frame
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
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
            float x = (j+1)*(trash_can_centroid[0])/steps;
            float y = (j+1)*(trash_can_centroid[1])/steps;
            // float x = (j+1)*(trash_can_centroid[0] - robot_position[0])/steps;
            // float y = (j+1)*(trash_can_centroid[1] - robot_position[1])/steps;
            // find orientation using robot position and trash item position
            float roll = 0.0, pitch = 0.0;
            float yaw = atan(y - robot_position(1) / x - robot_position(0));
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);

            ROS_INFO("Hereeeeee %f %f", robot_position(0), robot_position(1));
            setNavGoal(x, y, quat_msg, ac);
            ros::Duration(1).sleep();

        }
    
        // setNavGoal(trash_can_centroid[0], trash_can_centroid[1], 0, ac);
    }
}
