#include <ros/ros.h>
#include <string.h>
#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <icp/FakeLaserConfig.h>

#include <sensor_msgs/LaserScan.h>


bool got_new_scan = false;

struct trans
{
  float tx;
  float ty;
  float theta;
};

// Parameters to be changed
float test_scan_num = 10;
float vx = 0.0;
float vy = 0.0;
float omega = 0.0;

float T = 1.0;

bool new_params = false;
void callback(icp::FakeLaserConfig& config, uint32_t level) {
  
  // Load into actual variables
  test_scan_num = config.test_angle_num;
  vx = config.vx;
  vy = config.vy;
  omega = config.omega;
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

  ros::Publisher mot_pub = n.advertize<geometry_msgs::Pose>("/icp_change", 5);
  ROS_INFO_STREAM("Publishers Created");

  sleep(2);

  

  while(ros::ok())
  {
    sensor_msgs::LaserScan new_scan;
    
    //Fake(test_pub1, test_pub2, test_pubres);

    if (got_new_scan)
    {
      Real();
      got_new_scan = false;
    }

    if (fake)
    {
      CreateFakeLaserScan(&old_scan);
      CreateFakeLaserScan(&new_scan);

    }

    sensor_msgs::PointCloud old_cloud;
    sensor_msgs::PointCloud new_cloud;

    TransformScan(new_scan, &new_cloud);

    sensor_msgs::PointCloud cloud_disp;
    TransformScan(&scan1, &cloud_disp);
    //ROS_INFO_STREAM("Transfering data to clouds");

    if(!old_scan.ranges.empty() && !new_scan.ranges.empty())
    {

      trans t;
      t.tx = test_tx;
      t.ty = test_ty;
      t.theta = test_theta;

      if (fake)
      {
      //ROS_INFO_STREAM("Transforming C2");
      TransformCloud(&cloud2, t);
      }

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


      geometry_msgs::Pose pose_out;

      pose_out.position.x = new_t.tx;
      pose_out.position.y = new_t.ty;
      pose_out.position.z = 0;

      pose_out.orientation.w = cos(t_new.theta/2.0);
      pose_out.orientation.x = 0;
      pose_out.orientation.y = 0;
      pose_out.orientation.z = sin(t_new.theta/2.0);

      res_pub.publish(pose_out);

      if(fake)
      {
        sleep(5);
      }

      got_new_scan = false;
    }

    if(!new_scan.ranges.empty())
    {
      old_scan = new_scan;
    }


    ros::spinOnce();
  }
  
  return 0;
}