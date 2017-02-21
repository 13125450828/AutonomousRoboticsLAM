#include <ros/ros.h>
#include <string.h>
#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <icp/ICPConfig.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

struct trans
{
  float tx;
  float ty;
  float theta;
};

// Parameters to be changed
float test_scan_increment = 0.001;
float test_tx = 0.5;
float test_ty = 0.5;
float test_theta = 0.1;
int outer_loop_count = 3;

bool new_params = false;
void callback(icp::ICPConfig& config, uint32_t level) {
  
  // Load into actual variables
  test_scan_increment = config.test_angle_increment;
  test_tx = config.test_tx;
  test_ty = config.test_ty;
  test_theta = config.test_theta;
  outer_loop_count = config.outer_loop_count;
  new_params = true;
}


sensor_msgs::LaserScan CreateFakeLaserScan()
{
  sensor_msgs::LaserScan scan;
  scan.header.frame_id = "map";
  scan.angle_min = -0.5;
  scan.angle_max = 0.5;
  scan.angle_increment = test_scan_increment;
  scan.range_min = 0;
  scan.range_max = 10;

  int num_points = (scan.angle_max - scan.angle_min)/scan.angle_increment;
  scan.ranges = std::vector<float>();

  for (int i = 0; i < num_points; i++)
  {
    scan.ranges.push_back(5.0);
  }
}


sensor_msgs::PointCloud TransformScan(sensor_msgs::LaserScan scan_in)
{
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud cloud;
  projector_.projectLaser(scan_in, cloud);

  return cloud;
}

void TransformCloud(sensor_msgs::PointCloud* cloud, trans t)
{
  float x;
  float y;
  float z;

  float c = cos(t.theta);
  float s = sin(t.theta);

  geometry_msgs::Point32 point;

  for(std::vector<geometry_msgs::Point32>::iterator it = (*cloud).points.begin(); it != (*cloud).points.end(); ++it)
  {
    x = (*it).x;
    y = (*it).y;  
    z = (*it).z;

    (*it).x = c*x - s*y + t.tx;
    (*it).y = s*x + c*y + t.ty;
    z = (*it).z;
  }
}



trans GetPointGradient(float x1, float y1, float x2, float y2, trans curr)
{
  float c = cos(curr.theta);
  float s = sin(curr.theta);

  float alpha = 1 - c;

  float a = 2*(c*x1 - s*y1 + curr.tx - x2);
  float b = 2*(c*y1 + s*x1 + curr.ty - y2);

  trans grad;

  grad.tx = 1*a;
  grad.ty = 1*b;
  grad.theta = a*(-s*x1 -c*y2) + b*(c*x2 - s*y2);
}

trans GetFullGradient(std::vector<geometry_msgs::Point32> cloud1, std::vector<geometry_msgs::Point32> cloud2, std::vector<unsigned int> corr)
{

}

std::vector<unsigned int> GetCorrespondances(std::vector<geometry_msgs::Point32> cloud1, std::vector<geometry_msgs::Point32> cloud2)
{
  // point2(i) will be closest to point1(j)
  //return[j] holds # i
  std::vector<unsigned int> indices;
  float min_dist;
  float min_index;
  float index;
  float dist;

  float dx;
  float dy;
  float dz;

  for(std::vector<geometry_msgs::Point32>::iterator it1 = cloud1.begin(); it1 != cloud1.end(); ++it1)
  {
    min_dist = 999;
    min_index = -1;
    index = 0;

    for(std::vector<geometry_msgs::Point32>::iterator it2 = cloud2.begin(); it2 != cloud2.end(); ++it2)
    {

      dx = (*it1).x - (*it2).x;
      dy = (*it1).y - (*it2).y;
      dz = (*it1).z - (*it2).z;

      dist = dx*dx + dy*dy + dz*dz;

      if (dist < min_dist)
      {
        min_dist = dist;
        min_index = index;
      }

      index++;
    }

    indices.push_back(min_index);
  }

  return indices;
}

trans ICP(sensor_msgs::PointCloud* c1, sensor_msgs::PointCloud* c2)
{
  // c2 is a transform from c1
  // find the transform c1 -> c2
  trans t_init;
  t_init.tx = 0;
  t_init.ty = 0;
  t_init.theta = 0;

  for(int i = 0; i < outer_loop_count; i++)
  {
    std::vector<unsigned int> corr = GetCorrespondances((*c1).points, (*c2).points);

    trans grad = GetFullGradient((*c1).points, (*c2).points, corr);
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "icp_node");
  ros::NodeHandle n;

  // set up the reconfigure stuff
  dynamic_reconfigure::Server<icp::ICPConfig> server;
  dynamic_reconfigure::Server<icp::ICPConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::Publisher test_pub1 = n.advertise<sensor_msgs::PointCloud>("/cloud_1", 5);
  ros::Publisher test_pub2 = n.advertise<sensor_msgs::PointCloud>("/cloud_2", 5);
  ros::Publisher test_pubres = n.advertise<sensor_msgs::PointCloud>("/cloud_res", 5);

  while(ros::ok())
  {
    
    sensor_msgs::LaserScan scan1 = CreateFakeLaserScan();
    sensor_msgs::LaserScan scan2 = CreateFakeLaserScan();

    sensor_msgs::PointCloud cloud1 = TransformScan(scan1);
    sensor_msgs::PointCloud cloud2 = TransformScan(scan2);

    trans t;
    t.tx = test_tx;
    t.ty = test_ty;
    t.theta = test_theta;

    TransformCloud(&cloud2, t);

    //trans new_t = ICP(&cloud1, &cloud2)

    test_pub1.publish(cloud1);
    test_pub2.publish(cloud2);

    //TransformCloud(&cloud1, new_t);
    //test_pubres.publish(cloud1);


    ros::spinOnce();
  }
  
  return 0;
}