#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void setNavGoal(float x, float y, float orientation, MoveBaseClient &ac)
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
        ROS_INFO("Hereeeeee %f %f", waypoints[i], waypoints[i+1]);
        setNavGoal(waypoints[i], waypoints[i+1], 0, ac);
    }
}