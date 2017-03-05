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
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include "lab2_msgs/occupancy_update.h"

const float MAP_RESOLUTION = 0.1;
const int MAP_SIDE_LENGTH = 10;
const int MAP_WIDTH = uint(MAP_SIDE_LENGTH / MAP_RESOLUTION);
const int MAP_HEIGHT = MAP_WIDTH;
const int MAP_SIZE = MAP_WIDTH * MAP_HEIGHT;

const double empty_L = log(0.499999 / (1-0.499999));
const double occupied_L = log(0.999999 / (1-0.999999));
const double initial_L = log(0.5 / (1-0.5)); 

// Initialize the map
//char map_data[MAP_WIDTH * MAP_HEIGHT];
nav_msgs::OccupancyGrid map;

double log_odd (int P)
{
	P = P / 100; // convert to value between 0 and 1
	return log(P / (1-P));
}

int inv_log_odd (double L)
{
	double exp_L = exp(L);
	return 100 * exp_L / (1+exp_L);
}

//Callback function for the Position topic 
void occupancy_update_callback(const lab2_msgs::occupancy_update& msg)
{
	//This function is called when a new occupancy_update_out message is received
	int i = 0, j = 0, data_cell = 0;
	double prev_L;

	int length_occupied = msg.filled.size();
	int length_empty = msg.unfilled.size();
	char temp;


	for (int intI = 0; intI < length_empty; intI++)
	{
		j = MAP_WIDTH + msg.unfilled[intI].row -1;
		i = MAP_WIDTH + msg.unfilled[intI].col -1;
		
		if (i>=0 && j>=0  && i < MAP_WIDTH && j<MAP_HEIGHT)
		{
			data_cell = i*MAP_WIDTH + j;

			if (map.data[data_cell] == 50)
				prev_L = initial_L;
			else
				prev_L = log_odd(map.data[data_cell]);
		}

		// Update
		temp = char(inv_log_odd(empty_L + prev_L + initial_L));
		if (temp < 100)
			map.data[data_cell] = temp;
	}

	for (int intI = 0; intI < length_occupied; intI++)
	{
		j = MAP_WIDTH + msg.filled[intI].row -1;
		i = MAP_WIDTH + msg.filled[intI].col -1;

		if (i>=0 && j>=0 && i < MAP_WIDTH && j < MAP_HEIGHT)
		{
			data_cell = i*MAP_WIDTH + j;

			if (map.data[data_cell] == 50)
				prev_L = initial_L;
			else
				prev_L = log_odd(map.data[data_cell]);
		}

		// Update
		temp = 99;//char(inv_log_odd(occupied_L + prev_L + initial_L));
		if (temp < 100)
			map.data[data_cell] = temp;
	}
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
	ros::init(argc,argv,"map_maker");
	ros::NodeHandle n;

	//Subscribe to the desired topics and assign callbacks
	
	// Create Publishers
	ros::Publisher map_pub;
	ros::Publisher map_meta_pub;

	// Si Te's code publishes to this message which I use 
	ros::Subscriber occupancy_update_sub = n.subscribe("raytrace_output", 1, occupancy_update_callback);

	// Setup topics to Publish from this node
	map_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_map",1);
	map_meta_pub = n.advertise<nav_msgs::MapMetaData>("map_meta_data",1);

	// Creating updatable variables
	nav_msgs::MapMetaData map_meta_data;

	// Initialize the map_meta_data
	map_meta_data.resolution = MAP_RESOLUTION; // [m/cell]
	map_meta_data.width = MAP_WIDTH; // [cells]
	map_meta_data.height = MAP_HEIGHT; // [cells]
	map_meta_data.origin.position.x = -MAP_SIDE_LENGTH/2; // Position the grid at -5 [m]
	map_meta_data.origin.position.y = -MAP_SIDE_LENGTH/2; // Position the grid at -5 [m]

	// Initalize the map
	std::vector<signed char> map_data (MAP_SIZE, 50);
	
	//Setting to map struct
	map.info = map_meta_data;
	map.data = map_data;
	map.header.frame_id = "/odom";

	//Set the loop rate
	ros::Rate loop_rate(10);    // 20Hz update rate
    
	int iCount = 0;

	while (ros::ok())
	{
		loop_rate.sleep(); //Maintain the loop rate
		ros::spinOnce();   //Check for new messages
    
    	//Main loop code goes here:
		map_meta_pub.publish(map_meta_data);	

		if (iCount == 10)
		{
			// Publish the map once every 10 cycles
			iCount = 0;	
			map_pub.publish(map);
			ROS_INFO("Published the Map");
		}
		else iCount ++;
	}

	return 0;
}