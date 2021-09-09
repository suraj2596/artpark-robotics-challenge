#include <bits/stdc++.h>
#include <vector>
// #include <Eigen/Eigen>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
// #include "gazebo_ros_link_attacher.h"
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"
#include "gazebo_msgs/GetWorldProperties.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetLinkState.h"

class BaseController{
private:
	ros::NodeHandle nh;
	// ros::Rate sleep_rate(10);
	ros::Subscriber wipe_sub;
	ros::Subscriber grip_sub;
	ros::Subscriber drop_sub;
	ros::Subscriber spry_sub;
	ros::Subscriber init_sub;
	ros::Subscriber topic_sub;

	ros::Publisher sci_pub_1;
	ros::Publisher sci_pub_2;
	ros::Publisher spine_pub;
	ros::Publisher dry_pub;
	ros::Publisher wet_pub;
	ros::Publisher disc_pub;
	ros::Publisher grip_pub;
	ros::Publisher orient_pub;
	
	ros::ServiceClient attach_client;
	ros::ServiceClient detach_client;
	ros::ServiceClient world_prop_client;
	ros::ServiceClient model_state_client;
	ros::ServiceClient link_state_client;
	
	std_msgs::Float64 spine_msg;
	std_msgs::Float64 wiper_msg;
	std_msgs::Float64 sci_msg;
	std_msgs::Float64 disc_msg;
	std_msgs::Float64 grip_msg;
	std_msgs::Float64 orient_msg;

	gazebo_ros_link_attacher::Attach srv;
	gazebo_msgs::GetWorldProperties world_prop_srv;
	gazebo_msgs::GetModelState model_state_srv;
	gazebo_msgs::GetLinkState link_state_srv;

	double grip1_pos[3];
	double grip2_pos[3];
	


	void sciPub1(float data){
		// sci_msg.header.stamp = ros::Time::now();
		sci_msg.data = data;
		sci_pub_1.publish(sci_msg);
		ros::Duration(0.01).sleep();
	}

	void sciPub2(float data){
		// sci_msg.header.stamp = ros::Time::now();
		sci_msg.data = data;
		sci_pub_2.publish(sci_msg);
		ros::Duration(0.01).sleep();
	}

	void spinePub(float data){
		// spine_msg.header.stamp = ros::Time::now();
		spine_msg.data = data;
		spine_pub.publish(spine_msg);
		ros::Duration(0.01).sleep();
	}

	void wetWiperPub(float data){
		// wiper_msg.header.stamp = ros::Time::now();
		wiper_msg.data = data;
		wet_pub.publish(wiper_msg);
		ros::Duration(0.01).sleep();
	}

	void dryWiperPub(float data){
		// wiper_msg.header.stamp = ros::Time::now();
		wiper_msg.data = data;
		dry_pub.publish(wiper_msg);
		ros::Duration(0.01).sleep();
	}

	void discPub(float data){
		// disc_msg.header.stamp = ros::Time::now();
		disc_msg.data = data;
		disc_pub.publish(disc_msg);
		ros::Duration(0.01).sleep();
	}

	void gripPub(float data){
		// grip_msg.header.stamp = ros::Time::now();
		grip_msg.data = data;
		grip_pub.publish(grip_msg);
		ros::Duration(0.01).sleep();
	}

	void orientPub(float data){
		// orient_msg.header.stamp = ros::Time::now();
		orient_msg.data = data;
		orient_pub.publish(orient_msg);
		ros::Duration(0.01).sleep();
	}


public:
	BaseController(const ros::NodeHandle& n){
		nh = n;
		// sleep_rate(10);
		wipe_sub = nh.subscribe("/artpark/wipe_signal", 10, &BaseController::wipeCallback, this);
		grip_sub = nh.subscribe("/artpark/grip_signal", 10, &BaseController::gripCallback, this);
		drop_sub = nh.subscribe("/artpark/drop_signal", 10, &BaseController::dropCallback, this);
		spry_sub = nh.subscribe("/artpark/spry_signal", 10, &BaseController::spryCallback, this);
		init_sub = nh.subscribe("/artpark/init_signal", 10, &BaseController::initCallback, this);
		topic_sub = nh.subscribe("/artpark/object_topic", 10, &BaseController::objTopicCallback, this);

		sci_pub_1 = nh.advertise<std_msgs::Float64>("/artpark/scissor_1_ctrl/command", 10);
		sci_pub_2 = nh.advertise<std_msgs::Float64>("/artpark/scissor_2_ctrl/command", 10);
		spine_pub = nh.advertise<std_msgs::Float64>("/artpark/spine_ctrl/command", 10);
		dry_pub = nh.advertise<std_msgs::Float64>("/artpark/dry_wiper_ctrl/command", 10);
		wet_pub = nh.advertise<std_msgs::Float64>("/artpark/wet_wiper_ctrl/command", 10);
		disc_pub = nh.advertise<std_msgs::Float64>("/artpark/disc_ctrl/command", 10);
		grip_pub = nh.advertise<std_msgs::Float64>("/artpark/grip_ctrl/command", 10);
		orient_pub = nh.advertise<std_msgs::Float64>("/artpark/orient_ctrl_top_right/command", 10);

		ros::Duration(0.1).sleep();
		spinePub(0.0);
		ros::Duration(0.1).sleep();
		spinePub(0.0);
		sciPub1(-0.8);
		sciPub2(0.8);
		orientPub(-0.01);
		discPub(-0.01);
		ros::Duration(0.5).sleep();
		spinePub(0.0);

  	attach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");
  	detach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/detach");
	world_prop_client = nh.serviceClient<gazebo_msgs::GetWorldProperties>("gazebo/get_world_properties");
	model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	link_state_client = nh.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state");

  	srv.request.model_name_1 = "artpark_bot";
  	srv.request.link_name_1 = "link_3_0";
  	srv.request.model_name_2 = "box";
  	srv.request.link_name_2 = "box";

	link_state_srv.request.link_name = "link_3_0";
	updateModelName();
	}

	~BaseController(){}

	void wipeCallback(const std_msgs::Float64::ConstPtr& msg){		
		// wiper_msg.header.stamp = ros::Time:now();

		if(msg->data == 1.0){
			gripPub(-0.6);
			spinePub(0.0);
			sciPub1(-0.8);
			sciPub2(0.8);
			discPub(-0.01);
			orientPub(-0.01);
			ros::Duration(2.0).sleep();

			for(int i=0; i<100; i++){
				// gripPub(-0.6 + float(i)*(-0.4 + 0.6)/10);
				// spinePub(0.42 + float(i)*(0.22 - 0.42)/100);
				// sciPub1(-0.8 + float(i)*(-0.3 + 0.8)/100);
				sciPub2(0.8 + float(i)*(-0.8 - 0.8)/100);
				orientPub(-0.01 + float(i)*(-0.15 + 0.01)/100);
			}
			discPub(1.67);
			ros::Duration(5.0).sleep();
			spinePub(-0.2);
			ros::Duration(0.05).sleep();

			for(int i=0; i<3; i++){
				wetWiperPub(10);
				ros::Duration(2.0).sleep();
				wetWiperPub(-10);
				ros::Duration(2.0).sleep();
			}

			ROS_INFO_STREAM("Wet publisher is turned ON");
		}
		else if(msg->data == 2.0){
			wetWiperPub(0);
			spinePub(-0.0);
			ros::Duration(0.05).sleep();
			ROS_INFO_STREAM("Wet publisher is turned OFF");
		}
		else if(msg->data == 3.0){
			spinePub(0.02);
			discPub(-1.67);
			ros::Duration(10.0).sleep();
			spinePub(-0.2);
			ros::Duration(0.05).sleep();

			for(int i=0; i<3; i++){
				dryWiperPub(10);
				ros::Duration(2.0).sleep();
				dryWiperPub(-10);
				ros::Duration(2.0).sleep();
			}
			ROS_INFO_STREAM("Dry publisher is turned ON");
		}
		else if(msg->data == 4.0){
			dryWiperPub(0);
			spinePub(-0.08);
			ros::Duration(0.05).sleep();
			ROS_INFO_STREAM("Dry publisher is turned OFF");
		}
		else{
			ROS_INFO_STREAM("Invalid argument");
		}
	}



	void initCallback(const std_msgs::Float64::ConstPtr& msg){
		gripPub(-0.6);
		spinePub(-0.08);
		ros::Duration(0.05).sleep();
		sciPub1(-0.8);
		sciPub2(0.8);
		discPub(-0.01);
		orientPub(-0.01);
		ros::Duration(2.0).sleep();
	}

	void gripCallback(const std_msgs::Float64::ConstPtr& msg){
		gripPub(-0.6);
		spinePub(-0.08);
		ros::Duration(0.05).sleep();
		sciPub1(-0.8);
		sciPub2(0.8);
		discPub(-0.01);
		orientPub(-0.01);
		ros::Duration(2.0).sleep();

		for(int i=0; i<100; i++){
			// gripPub(-0.6 + float(i)*(-0.4 + 0.6)/10);
			// spinePub(0.42 + float(i)*(0.22 - 0.42)/100);
			// sciPub1(-0.8 + float(i)*(-0.3 + 0.8)/100);
			sciPub2(0.8 + float(i)*(-0.8 - 0.8)/100);
			orientPub(-0.01 + float(i)*(-0.15 + 0.01)/100);
		}
		updateModelName();
		

		for(int i=0; i<100; i++){
			// gripPub(-0.6 + float(i)*(-0.4 + 0.6)/10);
			spinePub(-0.08 + float(i)*(-0.18 + 0.08)/100);
			// ros::Duration(0.05).sleep();
			sciPub1(-0.8 + float(i)*(-0.55 + 0.8)/100);
			sciPub2(-0.8 + float(i)*(-0.55 + 0.8)/100);
			orientPub(-0.15 + float(i)*(-0.1 + 0.15)/100);
		}
		// ROS_INFO_STREAM(srv.request.model_name_2 << ":" << srv.request.link_name_2);
		if(attach_client.call(srv)){
			ROS_INFO_STREAM("Grabbed the object. Moving up");
		}
		else{
			ROS_INFO_STREAM("There is an error while grabbing the object");
		}
		gripPub(0.2);
		ros::Duration(2.0).sleep();

		for(int i=0; i<100; i++){
			// gripPub(-0.6 + float(i)*(-0.4 + 0.6)/10);
			spinePub(-0.18 + float(i)*(-0.08 + 0.18)/100);
			sciPub1(-0.55 + float(i)*(-1.0 + 0.55)/100);
			sciPub2(-0.55 + float(i)*(1.0 + 0.55)/100);
			orientPub(-0.1 + float(i)*(-0.01 + 0.1)/100);
		}

	}

	void dropCallback(const std_msgs::Float64::ConstPtr& msg){
		spinePub(0.1);
		ros::Duration(2.0).sleep();

		for(int i=0; i<100; i++){
			// gripPub(-0.6 + float(i)*(-0.4 + 0.6)/10);
			// spinePub(0.42 + float(i)*(0.22 - 0.42)/10);
			sciPub1(-1.0 + float(i)*(0.3 + 1.0)/100);
			sciPub2(1.0 + float(i)*(-0.3 - 1.0)/100);
			orientPub(-0.01 + float(i)*(0.03 + 0.01)/100);
		}

		if(detach_client.call(srv)){
			ROS_INFO_STREAM("Dropped the object");
		}
		else{
			ROS_INFO_STREAM("There is an error while dropping the object");
		}
//		ros::Duration(1.0).sleep();
		gripPub(-0.6);
	}

	void objTopicCallback(const std_msgs::String::ConstPtr& msg){
		std::stringstream ss(msg->data);
		std::string name;
		std::vector<std::string> names;
		int count = 0;

		while(ss >> name){
			if(count < 2){
				names.push_back(name);
				count++;
			}
			else{
				ROS_INFO_STREAM("Invalid number of words");
				return;
			}
		}
		if(count == 0){
			ROS_INFO_STREAM("Topic not specified");
			return;
		}
		else{
			ROS_INFO_STREAM("\nModel name: " << names[0] << "\n Link name: " << names[1]);
			srv.request.model_name_2 = names[0];
			srv.request.link_name_2 = names[1];
		}
	}



	void spryCallback(const std_msgs::Float64::ConstPtr& msg){
		gripPub(-0.6);
		spinePub(-0.0);
		ros::Duration(0.05).sleep();
		sciPub1(-0.8);
		sciPub2(0.8);
		discPub(-0.01);
		orientPub(-0.01);
		ros::Duration(2.0).sleep();

		for(int i=0; i<100; i++){
			// gripPub(-0.6 + float(i)*(-0.4 + 0.6)/10);
			// spinePub(0.42 + float(i)*(0.22 - 0.42)/100);
			// sciPub1(-0.8 + float(i)*(-0.3 + 0.8)/100);
			sciPub2(0.8 + float(i)*(-0.8 - 0.8)/100);
			orientPub(-0.01 + float(i)*(-0.15 + 0.01)/100);
		}

		discPub(3.14);
		ros::Duration(3.0).sleep();


		for(int i=0; i<100; i++){
			// gripPub(-0.6 + float(i)*(-0.4 + 0.6)/10);
			spinePub(-0.0 + float(i)*(-0.1 + 0.0)/100);
			sciPub1(-0.8 + float(i)*(-0.55 + 0.8)/100);
			sciPub2(-0.8 + float(i)*(-0.55 + 0.8)/100);
			orientPub(-0.15 + float(i)*(-0.1 + 0.15)/100);
			ros::Duration(0.01).sleep();
		}

		ros::Duration(3.0).sleep();
		discPub(0.0);
		ros::Duration(7.0).sleep();

		for(int i=0; i<100; i++){
			// gripPub(-0.6 + float(i)*(-0.4 + 0.6)/10);
			spinePub(-0.00 + float(i)*(-0.0 + 0.1)/100);
			sciPub1(-0.55 + float(i)*(-1.0 + 0.55)/100);
			sciPub2(-0.55 + float(i)*(1.0 + 0.55)/100);
			orientPub(-0.1 + float(i)*(-0.01 + 0.1)/100);
		}

	}

	void updateModelName(){
	link_state_srv.request.link_name = "link_3_0";

	if(link_state_client.call(link_state_srv)){
		grip1_pos[0] = link_state_srv.response.link_state.pose.position.x;
		grip1_pos[1] = link_state_srv.response.link_state.pose.position.y;
		grip1_pos[2] = link_state_srv.response.link_state.pose.position.z;
		// ROS_INFO_STREAM("Gripper position: " << grip1_pos[0] << grip1_pos[1] << grip1_pos[2]);
	}

	link_state_srv.request.link_name = "link_3_0_clone";

	if(link_state_client.call(link_state_srv)){
		grip2_pos[0] = link_state_srv.response.link_state.pose.position.x;
		grip2_pos[1] = link_state_srv.response.link_state.pose.position.y;
		grip2_pos[2] = link_state_srv.response.link_state.pose.position.z;
		// ROS_INFO_STREAM("Gripper position: " << grip1_pos[0] << grip1_pos[1] << grip1_pos[2]);
	}
		
	if(world_prop_client.call(world_prop_srv)){
		std::vector<std::string> models;
		std::string coke =  "coke";
		std::string cup = "cup";
		models = world_prop_srv.response.model_names;
		// ROS_INFO_STREAM(models.size());

		double min_dist = 10.0;

		for( int i = 0; i < models.size(); i++){
			size_t pos = (models[i].find(coke) != std::string::npos) || (models[i].find(cup) != std::string::npos);
			// ROS_INFO_STREAM(models[i] << ":" << pos);
			if(pos){
				model_state_srv.request.model_name = models[i];
				if(model_state_client.call(model_state_srv)){
					// ROS_INFO_STREAM("State: " << model_state_srv.response.pose.position);
					double object_pos[3];
					object_pos[0] = model_state_srv.response.pose.position.x;
					object_pos[1] = model_state_srv.response.pose.position.y;
					object_pos[2] = model_state_srv.response.pose.position.z;

					double dist1[3];
					dist1[0] = object_pos[0] - grip1_pos[0];
					dist1[1] = object_pos[1] - grip1_pos[1];
					dist1[2] = object_pos[2] - grip1_pos[2];
					double dist2[3];
					dist2[0] = object_pos[0] - grip2_pos[0];
					dist2[1] = object_pos[1] - grip2_pos[1];
					dist2[2] = object_pos[2] - grip2_pos[2];

					double total_dist = dist1[0]*dist1[0] + dist1[1]*dist1[1] + dist1[2]*dist1[2] + 
											dist2[0]*dist2[0] + dist2[1]*dist2[1] + dist2[2]*dist2[2];
					// ROS_INFO_STREAM("Distance: " << total_dist);
					if(total_dist < min_dist){
						min_dist = total_dist;
						srv.request.model_name_2 = models[i];
						pos = models[i].find(coke) != std::string::npos;
						if(pos){
							srv.request.link_name_2 = "link";
						}
						else{
							srv.request.link_name_2 = "link_2";
						}
					}
					// ROS_INFO_STREAM("Distance: " << dist[0] << " : " << dist[1] << " : " << dist[2]);
				}
			}
		}



		// for i in 
		// ROS_INFO_STREAM(world_prop_srv.response.model_names[0]);
	}
	else{
		ROS_INFO_STREAM("Service unsuccessful");
	}
		
	}

};






int main(int argc, char** argv){
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle nh;

  BaseController controller_(nh);


  ros::spin();
}
