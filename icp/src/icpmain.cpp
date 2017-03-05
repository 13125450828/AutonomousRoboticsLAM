#include <ros/ros.h>
#include <string.h>
#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <icp/ICPConfig.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <lab2_msgs/slam.h>
#include <geometry_msgs/PoseStamped.h>

sensor_msgs::LaserScan old_scan;
sensor_msgs::LaserScan new_scan;

geometry_msgs::PoseStamped old_pose;
geometry_msgs::PoseStamped new_pose;

sensor_msgs::PointCloud old_cloud;
sensor_msgs::PointCloud new_cloud;

bool got_new_scan = false;
bool currently_processing = false;

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
float corr_test_skip = 1;
float decay_const = 0.99;
float timeout = 1.0;

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
  corr_test_skip = config.corr_test_skip;
  decay_const = config.decay_const;
  timeout = config.timeout;

  //old_scan = sensor_msgs::LaserScan();
  //new_scan = sensor_msgs::LaserScan();
  //old_cloud = sensor_msgs::PointCloud();
  //new_cloud = sensor_msgs::PointCloud();
  //got_new_scan = false;

  new_params = true;
}


void TransformScan(sensor_msgs::LaserScan* scan, sensor_msgs::PointCloud* cloud)
{
  float x;
  float y;
  float z = 0.0;

  cloud->header.frame_id = scan->header.frame_id;
  cloud->points.clear();

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
    p.y = s*r;
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

  // hard-coded
  cloud->header.frame_id = "odom";
}

void TransformCloud(sensor_msgs::PointCloud* cloud, geometry_msgs::PoseStamped pose)
{
  
  trans t;

  t.tx = pose.pose.position.x;
  t.ty = pose.pose.position.y;

  t.theta = -2.0*asin(pose.pose.orientation.z);

  TransformCloud(cloud, t);

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

trans GetFullGradient(std::vector<geometry_msgs::Point32> cloud1, std::vector<geometry_msgs::Point32> cloud2, std::vector<unsigned int> corr, trans t, double alpha)
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

    grad.tx = grad.tx + t_new.tx*alpha;
    grad.ty = grad.ty + t_new.ty*alpha;
    grad.theta = grad.theta + t_new.theta*alpha;
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

    for(std::vector<geometry_msgs::Point32>::iterator it2 = cloud2.begin(); it2 != cloud2.end(); std::advance(it2, 1))
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

  ros::Time start = ros::Time::now();

  double use_alpha = grad_alpha;

  for(int i = 0; i < outer_loop_count; i++)
  {
    //ROS_INFO_STREAM("Getting Corresponsances");
    std::vector<unsigned int> corr = GetCorrespondances((*c1).points, (*c2).points);

    trans grad = GetFullGradient((*c1).points, (*c2).points, corr, t_init, use_alpha);

    //ROS_INFO_STREAM("Full Grad: " << grad.tx << " " << grad.ty << " " << grad.theta);

    trans vis_grad;
    vis_grad.tx = -grad.tx;
    vis_grad.ty = -grad.ty;
    vis_grad.theta = -grad.theta;
    //TransformCloud(temp, vis_grad);
    //pub->publish(*temp);

    t_init.tx = t_init.tx - grad.tx;
    t_init.ty = t_init.ty - grad.ty; 
    t_init.theta = t_init.theta - grad.theta;

    use_alpha = use_alpha * decay_const; // Slowly decay alpha

    ros::Time end = ros::Time::now();
    double time = (end - start).toSec();
    if(time > timeout && timeout > 0.001)
    {
      break; // Break if timed out
    }

  }
  return t_init;
}


void SLAMCallback(lab2_msgs::slam msg)
{
  ROS_INFO_STREAM("SLAM Callback");
  if (currently_processing)
  {
    return;
  }
  new_scan = msg.scan;
  new_pose = msg.pose;
  got_new_scan = true;
  ROS_INFO_STREAM("SLAM Callback Successful");
}

geometry_msgs::PoseStamped AppendTransform(trans t)
{
  /*
  float new_p_th = 2.0*asin(new_pose.pose.orientation.z);
  float new_p_c = cos(new_p_th);
  float new_p_s = sin(new_p_th);//ros copy one pointcloud into another
  */

  geometry_msgs::PoseStamped pose_out;

  /*
  pose_out.pose.position.x = new_pose.pose.position.x + t.tx*new_p_c - t.ty*new_p_s;
  pose_out.pose.position.y = new_pose.pose.position.y + t.tx*new_p_s + t.ty*new_p_c;
  pose_out.pose.position.z = 0;

  pose_out.pose.orientation.w = cos((new_p_th + t.theta)/2.0);
  pose_out.pose.orientation.x = 0;
  pose_out.pose.orientation.y = 0;
  pose_out.pose.orientation.z = sin((new_p_th + t.theta)/2.0);
  */
}


void CopyPointCloud(sensor_msgs::PointCloud* dest, sensor_msgs::PointCloud* src)
{
  dest->points.clear();

  for(std::vector<geometry_msgs::Point32>::iterator it = src->points.begin(); it != src->points.end(); std::advance(it, 1))
  {
    dest->points.push_back((*it));
  }
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

  ros::Publisher vis_new = n.advertise<sensor_msgs::PointCloud>("/cloud_new", 5);
  ros::Publisher vis_old= n.advertise<sensor_msgs::PointCloud>("/cloud_old", 5);
  ros::Publisher vis_result = n.advertise<sensor_msgs::PointCloud>("/cloud_res", 5);
  ros::Publisher mot_pub = n.advertise<geometry_msgs::PoseStamped>("/icp_pose", 5);
  ROS_INFO_STREAM("Publishers Created");

  ros::Subscriber las_sub = n.subscribe("/slam_inputs", 10, SLAMCallback);

  
  
  sensor_msgs::PointCloud cloud_disp;

  while(ros::ok())
  {
    
    //ROS_INFO_STREAM("Transfering data to clouds");

    currently_processing = true;
    if(got_new_scan && (!old_cloud.points.empty() && !new_scan.ranges.empty()))
    {

      //TransformScan(&old_scan, &cloud_disp);
      ROS_INFO_STREAM("Changing scan to cloud");
      TransformScan(&new_scan, &new_cloud);

      ROS_INFO_STREAM("Transforming Cloud");
      TransformCloud(&new_cloud, new_pose);

      
      ROS_INFO_STREAM("Publishing original clouds");
      //vis_new.publish(new_cloud);
      //vis_old.publish(old_cloud);

      ros::Time t1 = ros::Time::now();
      ROS_INFO_STREAM("Starting ICP");
      trans new_t = ICP(&old_cloud, &new_cloud, &cloud_disp, &vis_result);
      ros::Time t2 = ros::Time::now();

      ROS_INFO_STREAM("ICP time (s): " << (t2 - t1).toSec());
      ROS_INFO_STREAM("Final: " << new_t.tx << ", " << new_t.ty << ", " << new_t.theta);


      //TransformCloud(&old_cloud, new_t);
      //vis_result.publish(new_cloud);


      
      //old_scan = new_scan;
      //old_cloud = new_cloud;
      //old_cloud.points = new_cloud.points;
      //ROS_INFO_STREAM("Copying old datax");
      //CopyPointCloud(&old_cloud, &new_cloud);

      ROS_INFO_STREAM("Outputing");

      float new_p_th = 2.0*asin(new_pose.pose.orientation.z);
      float new_p_c = cos(new_p_th);
      float new_p_s = sin(new_p_th);//ros copy one pointcloud into another
      
      geometry_msgs::PoseStamped pose_out;

      pose_out.pose.position.x = new_pose.pose.position.x + new_t.tx*new_p_c - new_t.ty*new_p_s;
      pose_out.pose.position.y = new_pose.pose.position.y + new_t.tx*new_p_s + new_t.ty*new_p_c;
      pose_out.pose.position.z = 0;

      pose_out.pose.orientation.w = cos((new_p_th + new_t.theta)/2.0);
      pose_out.pose.orientation.x = 0;
      pose_out.pose.orientation.y = 0;
      pose_out.pose.orientation.z = sin((new_p_th + new_t.theta)/2.0);

      old_pose = pose_out;
      mot_pub.publish(pose_out);

      ROS_INFO_STREAM("Resetting");
      got_new_scan = false;
      ROS_INFO_STREAM(" ");
    }
    else if(got_new_scan && old_cloud.points.empty() && !new_scan.ranges.empty())
    {
      ROS_INFO_STREAM("Filling from new scan");
      old_scan = new_scan;
      TransformScan(&new_scan, &new_cloud);
      old_pose = new_pose;
      old_cloud.header.frame_id="odom";
      CopyPointCloud(&old_cloud, &new_cloud);
      //TransformScan(&old_scan, &old_cloud);
      //TransformCloud(&old_cloud, old_pose);
      got_new_scan = false;
    }
    else if (got_new_scan)
    {
      ROS_INFO_STREAM("Some other state");
      ROS_INFO_STREAM(" " << old_cloud.points.empty() << " " << new_scan.ranges.empty() << " " << new_cloud.points.empty());
    }
    currently_processing = false;


    //ROS_INFO_STREAM("*");
    ros::spinOnce();
  }
  
  return 0;
}