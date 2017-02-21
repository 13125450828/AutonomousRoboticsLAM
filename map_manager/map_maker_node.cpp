//  ///////////////////////////////////////////////////////////
//
// turtlebot_example_node.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos. 2012 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

//Callback function for the Position topic 
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	//This function is called when a new pose message is received

	double X = msg.pose.pose.position.x; // Robot X psotition
	double Y = msg.pose.pose.position.y; // Robot Y psotition
	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"main_control");
	ros::NodeHandle n;

	//Subscribe to the desired topics and assign callbacks

	// Si Te's Code file:	mapper_ray_trace.cpp
	// publishes to this message which I use 
	ros::Subscriber ray_trace_sub = n.subscribe("/ray_trace_output")

	//Setup topics to Publish from this node
	ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map_publisher",1);
	ros::Publisher map_meta_pub = n.advertise<nav_msgs::MapMetaData>("map_meta_data_pub",1);

	//Creating updatable variables
	nav_msgs::MapMetaData map_meta_data;
	nav_msgs::OccupancyGrid map;

	// Initialize the map_meta_data
	map_meta_data.map_load_time = 0;
	map_meta_data.resolution = 0.1; // [m/cell]
	map_meta_data.width = uint(10 / map_meta_data.resolution); // [cells]
	map_meta_data.height = uint(10 / map_meta_data.resolution); // [cells]

 	// Publish the map_meta_data
	map_meta_pub.publish(map_meta_data);

	// Setup the map
	map.info = map_meta_data;
	map.data = 

	//Set the loop rate
	ros::Rate loop_rate(20);    // 20Hz update rate
    
	while (ros::ok())
	{
		loop_rate.sleep(); //Maintain the loop rate
		ros::spinOnce();   //Check for new messages
    
		//Main loop code goes here:






		vel.linear.x = 0.2; // set linear speed
		vel.angular.z = 0.2; // set angular speed

		velocity_publisher.publish(vel); // Publish the command velocity



		ROS_DEBUG("Main - Velocity commands: v - %f, w - %f", vel.linear.x, vel.angular.z);
	}

	return 0;
}