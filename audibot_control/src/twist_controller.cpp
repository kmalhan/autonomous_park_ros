#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

ros::Publisher pub_speed;
ros::Publisher pub_steering;

double wheelbase;
double steering_ratio;

void recvTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  std_msgs::Float64 speed_msg;
  speed_msg.data = msg->linear.x;

  std_msgs::Float64 steer_msg;
  if (fabs(msg->linear.x) < 0.05){
    steer_msg.data = 0.0;
  }else{
    steer_msg.data = steering_ratio * atan(wheelbase * msg->angular.z / msg->linear.x);
  }

  pub_speed.publish(speed_msg);
  pub_steering.publish(steer_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_controller");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pn.param("wheelbase", wheelbase, 3.0);
  pn.param("steering_ratio", steering_ratio, 16.0);

  ros::Subscriber sub_twist = n.subscribe("cmd_vel", 1, recvTwist);
  pub_speed = n.advertise<std_msgs::Float64>("/audibot/audi_speed_controller/speed_cmd", 1);
  pub_steering = n.advertise<std_msgs::Float64>("/audibot/audi_steering_controller/steering_cmd", 1);

  ros::spin();
}


