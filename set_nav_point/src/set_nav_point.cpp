/*
 * This node publishes the goal for global planner. Switches to next when get closer (Production)
 */
#include <ros/ros.h>
#include <ugv_course_libs/gps_conv.h>
#include <geometry_msgs/PoseStamped.h>

std::vector<tf::Vector3> way_points;
geometry_msgs::PoseStamped robot_pose;
ros::Publisher pub_local_goal;

void recvPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  robot_pose = *msg;
}

void timerCallback(const ros::TimerEvent& event)
{
  static int index = 0;
  geometry_msgs::PoseStamped current_goal;
  double y_diff, x_diff, distance;
  // calculate distance till goal
  x_diff = robot_pose.pose.position.x - way_points[index].getX();
  y_diff = robot_pose.pose.position.y - way_points[index].getY();
  distance = hypot(y_diff, x_diff);
  // increment goal if distance is less than 5m
  if (distance <= 5.0)
  {
    if (index == (way_points.size()-1))
      index = 0;
    else
      index++;
  }
  // keep updating goal
  current_goal.header.frame_id = "/map";
  current_goal.header.stamp = event.current_real;
  current_goal.pose.orientation.w = 1.0;
  current_goal.pose.position.x = (double)way_points[index].getX();
  current_goal.pose.position.y = (double)way_points[index].getY();

  pub_local_goal.publish(current_goal);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_nav_point");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Waypoint used as goal during navigation
  way_points.resize(4);
  way_points[0].setX(44.550);
  way_points[0].setY(0.075);
  way_points[1].setX(45.372);
  way_points[1].setY(-54.315);
  way_points[2].setX(23.727);
  way_points[2].setY(-55.902);
  way_points[3].setX(24.930);
  way_points[3].setY(-2.900);

  pub_local_goal = n.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
  ros::Subscriber sub_fix = n.subscribe("/audibot/current_pose", 1, recvPose);
  ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);

  ros::spin();
}
