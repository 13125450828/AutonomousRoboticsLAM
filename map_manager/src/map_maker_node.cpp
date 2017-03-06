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

const float MAP_RESOLUTION = 0.01; // 
const int MAP_SIDE_LENGTH = 10;
const int MAP_WIDTH = uint(MAP_SIDE_LENGTH / MAP_RESOLUTION);
const int MAP_HEIGHT = MAP_WIDTH;
const int MAP_SIZE = MAP_WIDTH * MAP_HEIGHT;

#define EMPTY_ODD 0.48
#define FULL_ODD 0.999
#define INIT_ODD 0.5

const double empty_L = log(EMPTY_ODD / (1-EMPTY_ODD));
const double occupied_L = log(FULL_ODD / (1-FULL_ODD));
const double initial_L = log(INIT_ODD / (1-INIT_ODD)); 

// Initialize the map
//char map_data[MAP_WIDTH * MAP_HEIGHT];
nav_msgs::OccupancyGrid map;

double log_odd (int P)
{
	float p = P / 100.0; // convert to value between 0 and 1
	return log(p / (1-p));
}

int inv_log_odd (double L)
{
	double exp_L = exp(L);
	int temp = 100 * exp_L / (1+exp_L);
	if (temp == 100)
		temp = 99;
	else if(temp == 0)
		temp = 1;
	return temp;

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
		j = MAP_WIDTH + msg.unfilled[intI].row;
		i = MAP_WIDTH + msg.unfilled[intI].col;
		//i = msg.unfilled[intI].row;
		//j = msg.unfilled[intI].col;
		
		if (i>=0 && j>=0  && i < MAP_WIDTH && j<MAP_HEIGHT)
		{
			data_cell = i*MAP_WIDTH + j;

			if (map.data[data_cell] == 50)
				prev_L = initial_L;
			else
				prev_L = log_odd(map.data[data_cell]);
		}

		// Update
		int test = inv_log_odd(empty_L + prev_L + initial_L);
		temp = char(inv_log_odd(empty_L + prev_L + initial_L));
		//ROS_INFO_STREAM("UF " << (int)temp << " " << test << " x");
		if (temp < 100)
			map.data[data_cell] = temp;
	}

	for (int intI = 0; intI < length_occupied; intI++)
	{
		j = MAP_WIDTH + msg.filled[intI].row;
		i = MAP_WIDTH + msg.filled[intI].col;
		//i = msg.filled[intI].row;
		//j = msg.filled[intI].col;

		if (i>=0 && j>=0 && i < MAP_WIDTH && j < MAP_HEIGHT)
		{
			data_cell = i*MAP_WIDTH + j;

			if (map.data[data_cell] == 50)
				prev_L = initial_L;
			else
				prev_L = log_odd(map.data[data_cell]);
		}

		// Update
		int test = inv_log_odd(empty_L + prev_L + initial_L);
		temp = char(inv_log_odd(occupied_L + prev_L + initial_L));
		//ROS_INFO_STREAM("F  " << (int)temp << " "<< test << " x");
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
		//loop_rate.sleep(); //Maintain the loop rate
		sleep(0.1);
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