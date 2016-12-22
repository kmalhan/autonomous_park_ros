/*
 * This node implements dynamic reconfigure to manually control the Audibot for testing (Debug only)
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <dynamic_reconfigure/server.h>
#include <spot_detection/vehicle_controlConfig.h>

#define LENGTH 3.02367
#define RATIO 16.0

ros::Publisher pub_speed;
ros::Publisher pub_steer;
vehicle_control::vehicle_controlConfig cfg;

void reconfig(vehicle_control::vehicle_controlConfig& config, uint32_t level)
{
  cfg = config;
}

void timerCallback(const ros::TimerEvent& event)
{
  std_msgs::Float64 speed;
  std_msgs::Float64 steer;

  speed.data = cfg.speed;
  steer.data = cfg.steering;

  pub_speed.publish(speed);
  pub_steer.publish(steer);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicle_control");
  ros::NodeHandle n;

  pub_speed = n.advertise<std_msgs::Float64>("/audibot/audi_speed_controller/speed_cmd", 1);
  pub_steer = n.advertise<std_msgs::Float64>("/audibot/audi_steering_controller/steering_cmd", 1);
  ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);

  dynamic_reconfigure::Server<vehicle_control::vehicle_controlConfig> srv;
  srv.setCallback(boost::bind(reconfig, _1, _2));

  ros::spin();
}
