/*
 * This node control which node get access to control the Audibot based on detection input (production)
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>


ros::Publisher pub_mode;
bool detect_parking = false;
int detect_count = 0;
bool detect_updated = false;
std_msgs::Int16 control;
bool parking_mode = false;
geometry_msgs::PoseStamped goal;
bool start_detect = false;

void recvParking(const std_msgs::Bool::ConstPtr& msg)
{
  detect_parking = msg->data;
  detect_updated = true;
}

void recvGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Wait till goal changes to second one (due to Gazebo issue)
  if (msg->pose.position.x >= 45)       // match 46.376
    start_detect = true;
}

void timerCallback(const ros::TimerEvent& event)
{

  if (!parking_mode && start_detect)
  {
    if (detect_updated)
    {
      detect_updated = false;

      if (detect_parking)
        detect_count += 1;
      else
        detect_count = 0;
    }

    if (detect_count >= 5){
      parking_mode = true;
      control.data = 1;
      ROS_INFO("Parking Spot is Detected!");
    }
    else {
      control.data = 0;
    }
  }
//  ROS_INFO("Detect Count %d",detect_count);
  pub_mode.publish(control);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "master_node");
  ros::NodeHandle n;

  ros::Subscriber sub_park_spot = n.subscribe("/parking_detected", 1, recvParking);
  ros::Subscriber sub_goal = n.subscribe("/local_goal", 1, recvGoal);
  pub_mode = n.advertise<std_msgs::Int16>("/control_mode", 1);

  ros::Timer timer = n.createTimer(ros::Duration(0.02), timerCallback);

  control.data = 0;

  ros::spin();
}
