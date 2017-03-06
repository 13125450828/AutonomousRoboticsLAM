//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
// Edited: Nima Mohajerin
    //
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>
#include <Eigen/Dense>
#include <math.h>
#include <tf/transform_datatypes.h>

using namespace Eigen;

//Measurements
double ips_x;
double ips_y;
double ips_yaw;
double lin_v;
double ang_v;

//constants for particle filter 
const double ROSLOOPRATE = 10; // HZ
const double dt = 1/ROSLOOPRATE;
const int NUM_PARTICLES = 100;
Matrix <float, 4, NUM_PARTICLES, RowMajor> particleMatrix = Matrix<float,4,NUM_PARTICLES>::Zero(); //Particles for our states
Matrix <float, 4, NUM_PARTICLES, RowMajor> bestPDFParticles = Matrix<float, 4, NUM_PARTICLES>::Zero(); //Matrix used to check weighting 
bool motion = false; //flag for if the turtlebot actually moved
bool ipsUpdate = false;
bool firstIPSUpdate = false; 

double motionthreshold = 0.0;

short sgn(int x) { return x >= 0 ? 1 : -1; }

//Callback function for the Position topic (SIMULATION)
//Given code, need to switch to real IPS 

/*
void pose_callback(const gazebo_msgs::ModelStates& msg) 
{
    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;
    //if (abs(lin_v) > motionthreshold || abs(ang_v) > motionthreshold)
    //{
        motion = true;
        ips_x = msg.pose[i].position.x ;
        ips_y = msg.pose[i].position.y ;
        ips_yaw = tf::getYaw(msg.pose[i].orientation);
        ipsUpdate = true;
//       if (firstIPSUpdate == false){
        firstIPSUpdate = true;  
        for (int i = 0; i < NUM_PARTICLES; i++){
                particleMatrix(0,i) = ips_x;
                particleMatrix(1,i) = ips_y;
                particleMatrix(2,i) = ips_yaw; 
        }
    
    //}
}
*/
float sampleNormdist(float mu, float sigma){
    std::random_device rd; //todo move to man to run only once
    std::mt19937 e2(rd());
    std::normal_distribution<> dist(mu, sigma);
    return dist(e2);
}



//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    ips_x = msg.pose.pose.position.x; // Robot X psotition
    ips_y = msg.pose.pose.position.y; // Robot Y psotition
    ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
    ipsUpdate = true; 
    if (firstIPSUpdate == false){
        firstIPSUpdate = true;  
        for (int i = 0; i < NUM_PARTICLES; i++){
                particleMatrix(0,i) = ips_x;
                particleMatrix(1,i) = ips_y;
                particleMatrix(2,i) = ips_yaw; 
        }
    }
    //ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}


//Callback function for the Oomtry topic
void odometry_calback(const nav_msgs::Odometry& msg){
    lin_v = msg.twist.twist.linear.x;
    ang_v = msg.twist.twist.angular.z;
    //state_particle_filter(msg);
        if (abs(lin_v) > motionthreshold || abs(ang_v) > motionthreshold)
            motion = true;
}

//Simulation call back function for IPS data
void IPS_callback_sim(const geometry_msgs::PoseStamped& msg){
    ips_x = msg.pose.position.x;
    ips_y = msg.pose.position.y;
    ips_yaw = tf::getYaw(msg.pose.orientation);
    ipsUpdate = true;
}


//Stackoverflow 10847007
float normal_pdf (float x, float m, float s){
    static const float inv_sqrt_2pi = 0.3989422804014327;
    float a = (x-m)/s;
    return inv_sqrt_2pi/s*std::exp(-0.5f*a*a);
}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received
    
    //you probably want to save the map into a form which is easy to work with
}

//Subscribe to the desired topics and assign callbacks
ros::Subscriber pose_sub;
ros::Subscriber map_sub;
ros::Subscriber odom_sub;
//ros::subscriber pose_sub = n.subscribe("/indoor_pos", 1, )
//Setup topics to Publish from this node
//ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
//ros::Publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
ros::Publisher marker_pub;
ros::Publisher path_pub;
ros::Publisher pose_pub;

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//    vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    
    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

//Example of drawing a curve
void drawPoint(int k, double x, double y, double z)
{
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points
   visualization_msgs::Marker thePoints;
   thePoints.header.frame_id = "/odom";
   thePoints.id = k; //each curve must have a unique id or you will overwrite an old ones
   thePoints.type = visualization_msgs::Marker::POINTS;
   thePoints.scale.x = 0.1;
   thePoints.scale.y = 0.1;
   thePoints.color.r = 1.0f ;
   thePoints.color.a = 1.0;
   //generate curve points
   geometry_msgs::Point p;
   p.x = x;
   p.y = y;
   p.z = 0; //not used
   thePoints.points.push_back(p);
   //publish new curve
   marker_pub.publish(thePoints);
}

//Example of drawing a curve
void drawArrow(int k, double x, double y, double z,double qx, double qy, double qz, double qw)
{
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points
   visualization_msgs::Marker thePoints;
   thePoints.header.frame_id = "/odom";
   thePoints.id = k; //each curve must have a unique id or you will overwrite an old ones
   thePoints.type = visualization_msgs::Marker::ARROW;
   thePoints.scale.x = 1;
   thePoints.scale.y = 0.1;
   thePoints.scale.z = 0.0;
   thePoints.color.r = 1.0f ;
   thePoints.color.a = 1.0; 
   thePoints.pose.position.x = x;
   thePoints.pose.position.y = y;
   thePoints.pose.position.z = 0;
   thePoints.pose.orientation.x = qx;
   thePoints.pose.orientation.y = qy;
   thePoints.pose.orientation.z = qz;
   thePoints.pose.orientation.w = qw;
   //generate curve points
   //geometry_msgs::Point p;
   //p.x = x;
   //p.y = y;
   //p.z = 0; //not used
   //thePoints.points.push_back(p);
   //publish new curve
   marker_pub.publish(thePoints);
}

int main(int argc, char **argv)
{
    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ROS_INFO("Starting Main Node");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    //pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    pose_sub = n.subscribe("indoor_pos", 1, pose_callback);
    map_sub = n.subscribe("/map", 1, map_callback);
    odom_sub = n.subscribe("/odom",1,odometry_calback);
    //ros::subscriber pose_sub = n.subscribe("/indoor_pos", 1, )
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    //ros::Publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    path_pub = n.advertise<nav_msgs::Path>("/path", 1, true);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/estimatedpose",1,true);

    geometry_msgs::Point marker_point;
    geometry_msgs::PoseStamped final_pos;
    nav_msgs::Path path;

    //Velocity control variable if we want to output a velocity command 
    geometry_msgs::Twist vel;

    //Mesaurements and Input
    std::vector<float> Y_matrix(3); //Measurements
    std::vector<float> X_input(3); //Inputs 
    std::vector<float> R_matrix(3); //Measurement noise
    std::vector<float> Q_matrix(3); //Model Noise assume DIAG 
    std::vector<float> Weights(3);

    Q_matrix [0] = 0.01;
    Q_matrix [1] = 0.01;
    Q_matrix [2] = 0.01;

    R_matrix [0] = 0.01;
    R_matrix [1] = 0.01;
    R_matrix [2] = 0.01;
    
    float Q = 0.1; //for quick and dirty testing 
    float R = 0.1;

    double mu = 0;
    double cumsum_w = 0; //Cumulative sum for w
    double seed = 0; //Seed for thresholding

    std::vector<float> CumSum (NUM_PARTICLES);
    //Particle matrix
    //Col 1: X Col 2: Y Col 3: Yaw Col 4: Weight of Particle 


    //Max and min locations for initial particle spread 
    int randfactor = 0.1;
    int randMin = 0;

    ROS_INFO("Populating particles uniformally");
    //populate particles uniformally 
    /*
    for (int i = 0; i < NUM_PARTICLES; i++){
        particleMatrix (0,i) = (double) randfactor*rand()/RAND_MAX;
        particleMatrix (1,i) = (double) randfactor*rand()/RAND_MAX;
        particleMatrix (2,i) = (double) rand()/RAND_MAX * 2 * M_PI - M_PI;  
    }
    */
    //Set the loop rate
    ros::Rate loop_rate(ROSLOOPRATE);    //10Hz update rate
    //setup RVIZ Viz stuff
    visualization_msgs::Marker points;
    points.header.frame_id = "/odom";
    points.ns              = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.r = 1.0f;
    points.color.a = 1;
    path.header.frame_id = "/odom";
    final_pos.header.frame_id = "/odom";

    ROS_INFO("Marker Object Setup");

    int loopcounter = 0; //to allow for multirate (odom + IPS)

    while (ros::ok())
    {
        //ROS_INFO("Starting loop");

        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new meanssages

        //Input variables from odom currently (TODO should we also get this from IPS?)
        X_input[0]= lin_v;
        X_input[1]= lin_v;
        X_input[2] = ang_v;

        //ROS_INFO("Registering Inputs");
        //if (motion == true)
        if (true) //currently waitng for motion is not working well 
        {
            motion = false;
            loopcounter ++;
            
            //get measurements from IPS or Odom 
            if (loopcounter % 10 == 0) //IPS a t H
            {
                //only use IPS at 1 Hz 
                Y_matrix[0] = ips_x + sampleNormdist(0,0.1);
                Y_matrix[1] = ips_y + sampleNormdist(0,0.1);
                Y_matrix[2] = ips_yaw + sampleNormdist(0,0.01);
            }
            else
            {
                Y_matrix[0] = Y_matrix[0] + dt*cos(Y_matrix[2])*X_input[0];
                Y_matrix[1] = Y_matrix[1] + dt*sin(Y_matrix[2])*X_input[1];
                Y_matrix[2] = Y_matrix[2] + dt*X_input[2];
                Y_matrix[2] = atan2(sin(Y_matrix[2]), cos(Y_matrix[2]));
            }
            
            //Right now take only ODOM, IPS from Gazebo updates slow? 
                Y_matrix[0] = ips_x + sampleNormdist(0,0);
                Y_matrix[1] = ips_y + sampleNormdist(0,0);
                Y_matrix[2] = ips_yaw + sampleNormdist(0,0);
                
                //ROS_INFO("IPS X: %f Y: %f Yaw: %f", ips_x, ips_y,ips_yaw);
                ROS_INFO("Angular Velocity %f", X_input[2]);

                double sum_x=0; //used for averaging particles
                double sum_y=0; //used for averaging particles 
                double sum_yaw=0;


            //Update particles from motion model and assign weighs to each particle based on measurements 
            for (int i = 0; i < NUM_PARTICLES; i++){
                //Simple motion model --> xk+1 = xk + dt*vx + w 
                particleMatrix(0,i) = particleMatrix(0,i) + dt*cos(particleMatrix(2,i))*X_input[0] + sampleNormdist(0,Q_matrix[0]); //x update
                particleMatrix(1,i) = particleMatrix(1,i) + dt*sin(particleMatrix(2,i))*X_input[1] + sampleNormdist(0,Q_matrix[1]); //y update 
                particleMatrix(2,i) = particleMatrix(2,i) + dt*X_input[2] + sampleNormdist(0,Q_matrix[2]);; //TODO add noise to yaw update 
                particleMatrix(2,i) = atan2(sin(particleMatrix(2,i)),cos(particleMatrix(2,i))); 
                //ROS_INFO("Angular vel: %f", X_input[2]);
                //Calculate individual weights
                
                Weights[1]= normal_pdf(Y_matrix[0], particleMatrix(0,i),0.01);
                Weights[2]= normal_pdf(Y_matrix[1], particleMatrix(1,i),0.01);
                Weights[3]= normal_pdf(Y_matrix[2], particleMatrix(2,i),0.01);
                

                //Sum x and y for pose output 
                
                sum_x = sum_x + particleMatrix(0,i);
                sum_y = sum_y + particleMatrix(1,i);
                sum_yaw = sum_yaw + particleMatrix(2,i);
                
                //ROS_INFO("X PDF Params: %f --- %f --- %f",Y_matrix[0], particleMatrix(0,i),0.01);
                //ROS_INFO("Y PDF Params: %f --- %f --- %f",Y_matrix[1], particleMatrix(1,i),0.01);
                //ROS_INFO("Yaw PDF Params: %f --- %f --- %f",Y_matrix[2], particleMatrix(2,i),0.1);
               
                //Combine and store as 3rd element in particleMatrix
                //particleMatrix(3,i) = Weights[1]*Weights[2]*Weights[3];
                //not doing yaw weights right now cuz it was giving me fuckery  
                //particleMatrix(3,i) = 1/(Q*Q*2*M_PI)*exp(-(pow(Y_matrix[0]-particleMatrix(0,i),2)/(2*Q*Q)+pow(Y_matrix[1]-particleMatrix(1,i),2)/(2*Q*Q)));
                particleMatrix(3,i) = Weights[1]*Weights[2]*Weights[3];
                // ROS_INFO("Weights: %f --- %f --- %f ---- %f",Weights[1],Weights[2],Weights[3], particleMatrix(3,i));
                //Compute a cumulative weight for resampling the particles later on 
                if (i==0) 
                    CumSum[i] = particleMatrix(3,i);
                else{
                    CumSum[i] = CumSum[i-1] + particleMatrix(3,i);
                   // ROS_INFO("CUMSUM[i] : %f",CumSum[i]);
                }

                //Draw the particle 
                marker_point.x = particleMatrix(0,i);
                marker_point.y = particleMatrix(1,i);
                //points.points.push_back(marker_point);
                drawPoint(i, marker_point.x, marker_point.y, 0);
            }
            //ROS_INFO("Weights: %f ",CumSum[NUM_PARTICLES-2]);

            //Create our path based on averaging our particles
            final_pos.pose.position.x = sum_x / NUM_PARTICLES;
            final_pos.pose.position.y = sum_y / NUM_PARTICLES;
            final_pos.pose.position.z = 0;
            ROS_INFO("SUM X: %f SUM Y: %f SUM YAW: %f",sum_x / NUM_PARTICLES,sum_y / NUM_PARTICLES,sum_yaw/NUM_PARTICLES);

            geometry_msgs::Quaternion est_quat = tf::createQuaternionMsgFromYaw(sum_yaw/NUM_PARTICLES);

            final_pos.pose.orientation = est_quat;
            drawArrow(1,final_pos.pose.position.x,final_pos.pose.position.y,final_pos.pose.position.z,est_quat.x,est_quat.y,est_quat.z,est_quat.w);

            path.poses.push_back(final_pos);            
            double seed;
             if (ipsUpdate == true){
                sum_x = 0;
                sum_y = 0;
                sum_yaw = 0;

                ipsUpdate = false;
            // For resampiling particles based on weight seeding
                int tempindex = 0;
                for (int j = 0; j < NUM_PARTICLES; j++)
                {
                    seed = CumSum[NUM_PARTICLES-1]*rand()/RAND_MAX;
                    //ROS_INFO("SEED: %f CUMSUM[NUM_PARTICLES-1]: %f",seed,CumSum[NUM_PARTICLES-1]);

                    // Determine Particles Greater than the Seed
                    for (int i = 0; i< NUM_PARTICLES; i++){
                        //ROS_INFO("CUMSUM[i]: %f",CumSum[i]);
                        if (CumSum[i] >= seed)
                        {
                            tempindex = i;
                            i=NUM_PARTICLES+1;
                        }
                    }

                    bestPDFParticles (0,j) = particleMatrix(0,tempindex);
                    bestPDFParticles (1,j) = particleMatrix(1,tempindex);
                    bestPDFParticles (2,j) = particleMatrix(2,tempindex);
                    sum_x = sum_x + bestPDFParticles(0,j);
                    sum_y = sum_y + bestPDFParticles(1,j);
                    sum_yaw = sum_yaw + bestPDFParticles(2,j);
                }
                particleMatrix = bestPDFParticles; //store updated particles

                final_pos.pose.position.x = sum_x / NUM_PARTICLES;
                final_pos.pose.position.y = sum_y / NUM_PARTICLES;
                final_pos.pose.position.z = 0;
                geometry_msgs::Quaternion est_quat;
                est_quat.x=0;
                est_quat.y = 0;
                est_quat.z=sin(sum_yaw/NUM_PARTICLES/2);
                est_quat.w=cos(sum_yaw/NUM_PARTICLES/2);//tf::createQuaternionMsgFromYaw(sum_yaw/NUM_PARTICLES);
                final_pos.pose.orientation = est_quat;
                pose_pub.publish(final_pos);
                } 
            }
        marker_pub.publish(points);
        path_pub.publish(path);
        //uncomment this if you want to send a fixed velocity command to test the particle filter X = 0.1 Z = 0.1 works well
        //vel.linear.x = 0.1; // set linear speed
        //vel.angular.z = 0.1; // set angular speed
        //velocity_publisher.publish(vel); // Publish the command velocity   
    }
    return 0;
}
