#include <ros/ros.h>
#include <string.h>

#include <dynamic_reconfigure/server.h>

// This name defined at bottom of cgf file, with Config added to end
#include <dr_example_node/DRExampleConfig.h>

int a;
double b;
std::string message;
bool q;

// Probably best to do something like this
bool new_params = false;


// This callback is called when new parameters are set
void callback(dr_example_node::DRExampleConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);

  // Load into actual variables
  a = config.int_param;
  b = config.double_param;
  message = config.str_param.c_str();
  q = config.bool_param;

  new_params = true;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "node_name");

  // set up the reconfigure stuff
  dynamic_reconfigure::Server<dr_example_node::DRExampleConfig> server;
  dynamic_reconfigure::Server<dr_example_node::DRExampleConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  while(ros::ok())
  {
    if (new_params)
    {
      ROS_INFO_STREAM("New Message: " << message);
      new_params = false;
    }
    ros::spinOnce();
  }
  
  return 0;
}