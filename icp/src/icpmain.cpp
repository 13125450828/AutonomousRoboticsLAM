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
float test_scan_num = 10;
float test_tx = 0.5;
float test_ty = 0.5;
float test_theta = 0.1;
int outer_loop_count = 3;
float grad_alpha = 0.001;
float grad_beta = 0.001;

bool new_params = false;
void callback(icp::ICPConfig& config, uint32_t level) {
  
  // Load into actual variables
  test_scan_num = config.test_angle_num;
  test_tx = config.test_tx;
  test_ty = config.test_ty;
  test_theta = config.test_theta;
  outer_loop_count = config.outer_loop_count;
  grad_alpha = config.grad_alpha;
  grad_beta = config.grad_beta;
  new_params = true;
}


void CreateFakeLaserScan(sensor_msgs::LaserScan* scan)
{
  //sensor_msgs::LaserScan scan;
  scan->header.frame_id = "map";
  scan->angle_min = -0.5;
  scan->angle_max = 0.5;
  scan->angle_increment = (scan->angle_max - scan->angle_min)/test_scan_num;
  scan->range_min = 0;
  scan->range_max = 10;

  scan->ranges = std::vector<float>();

  //ROS_INFO_STREAM("Adding points: " << test_scan_num << " of " << test_scan_num);
  for (int i = 0; i < test_scan_num; i++)
  {
    scan->ranges.push_back(3.0);
  }
}


void TransformScan(sensor_msgs::LaserScan* scan, sensor_msgs::PointCloud* cloud)
{
  float x;
  float y;
  float z = 0.0;

  cloud->header.frame_id = scan->header.frame_id;

  //projector_.projectLaser(scan_in, cloud);
  //ROS_INFO_STREAM("Starting scan iter");
  int size = test_scan_num;
  for(int i =0; i < size; i++)
  {
    float th = scan->angle_min + i* scan->angle_increment;
    float c = cos(th);
    float s = sin(th);
    float r = scan->ranges.at(i);

    geometry_msgs::Point32 p;
    p.x = c*r;
    p.y = s*s*r;
    p.z = z;

    //ROS_INFO_STREAM("Adding point " << i);
    //ROS_INFO_STREAM("Adding point: " << i << " of " << size << " r=" << r << ", th=" << th);
    cloud->points.push_back(p); 
  }

  //return cloud;
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

  //ROS_INFO_STREAM("a: " << a << ", b: " << b);
  //ROS_INFO_STREAM("c: " << c << ", s: " << s);
  //ROS_INFO_STREAM("x1: " << x1 << ", y1: " << y1);

  trans grad;

  grad.tx = (1*a)*grad_beta;
  grad.ty = (1*b)*grad_beta;
  grad.theta = (a*(-s*x1 -c*y1) + b*(c*x1 - s*y1))*grad_beta;

  return grad;
}

trans GetFullGradient(std::vector<geometry_msgs::Point32> cloud1, std::vector<geometry_msgs::Point32> cloud2, std::vector<unsigned int> corr, trans t)
{

  trans grad;
  grad.tx = 0;
  grad.ty = 0;
  grad.theta = 0;

  for(int i = 0; i < cloud1.size(); i++)
  {
    int index = corr.at(i);
    geometry_msgs::Point32 p1 = cloud1.at(i);
    geometry_msgs::Point32 p2 = cloud2.at(i);

    trans t_new = GetPointGradient(p1.x, p1.y, p2.x, p2.y, t);

    //ROS_INFO_STREAM("Part Grad: " << t_new.tx << " " << t_new.ty << " " << t_new.theta);

    grad.tx = grad.tx + t_new.tx*grad_alpha;
    grad.ty = grad.ty + t_new.ty*grad_alpha;
    grad.theta = grad.theta + t_new.theta*grad_alpha;
  }

  grad.tx = grad.tx / test_scan_num;
  grad.ty = grad.ty / test_scan_num;
  grad.theta = grad.theta / test_scan_num;

  return grad;
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

trans ICP(sensor_msgs::PointCloud* c1, sensor_msgs::PointCloud* c2, sensor_msgs::PointCloud* temp, ros::Publisher* pub)
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

    trans grad = GetFullGradient((*c1).points, (*c2).points, corr, t_init);

    ROS_INFO_STREAM("Full Grad: " << grad.tx << " " << grad.ty << " " << grad.theta);

    trans vis_grad;
    vis_grad.tx = -grad.tx;
    vis_grad.ty = -grad.ty;
    vis_grad.theta = -grad.theta;
    TransformCloud(temp, vis_grad);
    pub->publish(*temp);

    t_init.tx = t_init.tx - grad.tx;
    t_init.ty = t_init.ty - grad.ty; 
    t_init.theta = t_init.theta - grad.theta;

    sleep(0.5);
  }

  sleep(0.5);
  return t_init;
}


int main(int argc, char **argv) {
  ROS_INFO_STREAM("Starting");
  ros::init(argc, argv, "icp_node");
  ros::NodeHandle n;

  // set up the reconfigure stuff
  dynamic_reconfigure::Server<icp::ICPConfig> server;
  dynamic_reconfigure::Server<icp::ICPConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  ROS_INFO_STREAM("Dyn Rec Created");

  ros::Publisher test_pub1 = n.advertise<sensor_msgs::PointCloud>("/cloud_1", 5);
  ros::Publisher test_pub2 = n.advertise<sensor_msgs::PointCloud>("/cloud_2", 5);
  ros::Publisher test_pubres = n.advertise<sensor_msgs::PointCloud>("/cloud_res", 5);
  ROS_INFO_STREAM("Publishers Created");

  sleep(2);

  while(ros::ok())
  {
    //ROS_INFO_STREAM("Creating Scans");
    sensor_msgs::LaserScan scan1;
    CreateFakeLaserScan(&scan1);
    sensor_msgs::LaserScan scan2;
    CreateFakeLaserScan(&scan2);

    //ROS_INFO_STREAM("Creating Clouds");
    laser_geometry::LaserProjection projector_;
    //sensor_msgs::PointCloud cloud;
    
    sensor_msgs::PointCloud cloud1;
    TransformScan(&scan1, &cloud1);
    sensor_msgs::PointCloud cloud2;
    TransformScan(&scan2, &cloud2);

    sensor_msgs::PointCloud cloud_disp;
    TransformScan(&scan1, &cloud_disp);
    //ROS_INFO_STREAM("Transfering data to clouds");

    if(!scan1.ranges.empty() && !scan2.ranges.empty())
    {

      trans t;
      t.tx = test_tx;
      t.ty = test_ty;
      t.theta = test_theta;

      //ROS_INFO_STREAM("Transforming C2");
      TransformCloud(&cloud2, t);

      ROS_INFO_STREAM("Starting ICP");

      test_pub1.publish(cloud1);
      test_pub2.publish(cloud2);

      ros::Time t1 = ros::Time::now();
      trans new_t = ICP(&cloud1, &cloud2, &cloud_disp, &test_pubres);
      ros::Time t2 = ros::Time::now();

      ROS_INFO_STREAM("ICP time (s): " << (t2 - t1).toSec());
      ROS_INFO_STREAM("Final: " << new_t.tx << ", " << new_t.ty << ", " << new_t.theta);

      

      TransformCloud(&cloud1, new_t);
      test_pubres.publish(cloud1);

      sleep(5);
    }
    else
    {
      ROS_INFO_STREAM("Empty Scans");
    }


    ros::spinOnce();
  }
  
  return 0;
}