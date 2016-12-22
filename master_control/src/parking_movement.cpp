#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>


ros::Publisher pub_speed;
ros::Publisher pub_steer;

int control = 0;
int phase = 0;
int back_status = 0;

int stable_time = 100;
int forward_time = 400;
int curve_back_time = 550;
int back_fix_time = 150;
bool done = false;

//int redo_time = 50;


void recvControl(const std_msgs::Int16::ConstPtr& msg)
{
  control = msg->data;
}

void recvBack(const std_msgs::Int16::ConstPtr& msg)
{
  back_status = msg->data;
}

void timerCallback(const ros::TimerEvent& event)
{
  std_msgs::Float64 speed;
  std_msgs::Float64 steer;

  if (control == 1)
  {
    if (back_status == 2) // Redo-left
    {
      speed.data = 1.0;
      steer.data = 5.0;
//      redo_time--;
//      if (phase == 2)
//        curve_back_time++;
      if (phase == 3)
        back_fix_time++;
    } else if (back_status == 3) // redo-right
    {
      speed.data = 1.0;
      steer.data = -5.0;
//      redo_time--;
      if (phase == 2)
        curve_back_time++;
      if (phase == 3)
        back_fix_time++;
    } else if (back_status == 0) // Normal
    {

      if (stable_time == 99)
        ROS_INFO("Stable Time");

      switch (phase)
      {
        case 0: // stable time
          speed.data = 0.0;
          steer.data = 0.0;
          stable_time--;
          if (stable_time == 0){
            stable_time = 100;
            phase = 1;
            ROS_INFO("Forward Time");
          }
          break;
        case 1: // forward
          speed.data = 1.0;
          steer.data = 0.0;
          forward_time--;
          if (forward_time == 0){
            forward_time = 400;
            phase = 2;
            speed.data = 0.0;
            steer.data = -15.0;
            ROS_INFO("Back Curve Time");
          }
          break;
        case 2:   // back curve
          speed.data = -1.0;
          steer.data = -15.0;
          curve_back_time--;
          if (curve_back_time == 0){
            forward_time = 550;
            phase = 3;
            speed.data = 0.0;
            steer.data = 0.0;
            ROS_INFO("Fix Time");
          }
          break;
        case 3:   // fix time
          speed.data = -1.0;
          steer.data = 0.0;
//          back_fix_time--;
//          if (back_fix_time == 0){
//            back_fix_time = 150;
//            phase = 4;
//            speed.data = 0.0;
//            steer.data = 0.0;
//            ROS_INFO("Done Time");
//          }
          break;
        case 4:   // Done
          speed.data = 0.0;
          steer.data = 0.0;
          break;
        default:
          break;
      }
    }  else if (back_status == 1) // Done
    {
       speed.data = 0.0;
       steer.data = 0.0;
       if (done == false)
         ROS_INFO("Parking is Completed!!");
       done = true;
    }
    // Publish Speed and Steer
    pub_speed.publish(speed);
    pub_steer.publish(steer);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "parking_movement");
  ros::NodeHandle n;

  ros::Subscriber sub_control = n.subscribe("/control_mode", 1, recvControl);
  ros::Subscriber sub_back = n.subscribe("/back_camera/status", 1, recvBack);

  pub_speed = n.advertise<std_msgs::Float64>("/audibot/audi_speed_controller/speed_cmd", 1);
  pub_steer = n.advertise<std_msgs::Float64>("/audibot/audi_steering_controller/steering_cmd", 1);

  ros::Timer timer = n.createTimer(ros::Duration(0.02), timerCallback);

  ros::spin();
}
