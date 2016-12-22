/*
 * This node implements global, local costmap, global planner, and teb local planner (Production)
 */

#include <ros/ros.h>
#include <navfn/navfn_ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>

#include <teb_local_planner/teb_local_planner_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>

#define LENGTH 3.02367
#define RATIO 16.0

tf::TransformListener* listener;
costmap_2d::Costmap2DROS* local_costmap;
costmap_2d::Costmap2DROS* global_costmap;
navfn::NavfnROS* global_planner;
teb_local_planner::TebLocalPlannerROS* teb_planner;

//ros::Publisher pub_twist;
ros::Publisher pub_goal;
ros::Publisher pub_speed;
ros::Publisher pub_steer;
ros::Publisher pub_pose;

geometry_msgs::PoseStamped current_goal;
tf::Stamped<tf::Pose> pose_stamped;
bool valid_goal = false;
double x_offset, y_offset;
int control = 0;

void recvControl(const std_msgs::Int16::ConstPtr& msg)
{
  control = msg->data;
}

/* Callback when Goal is received from Rviz */
void recvGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  valid_goal = true;
  current_goal = *msg;
}

/* Callback when Odom is received */
void recvOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_stamped.frame_id_ = msg->header.frame_id;
  pose_stamped.stamp_ = msg->header.stamp;

  tf::Vector3 vehicle_position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  pose_stamped.setOrigin(vehicle_position);

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  pose_stamped.setRotation(q);
}

/* Publish Speed and Steering Control */
void timerCallback(const ros::TimerEvent& event)
{
  if (valid_goal && (control == 0))
  {
    std_msgs::Float64 speed;
    std_msgs::Float64 steer;

    geometry_msgs::Twist twist_msg;
    teb_planner->computeVelocityCommands(twist_msg);
  //  ROS_INFO("Speed: %f, YawRate: %f", twist_msg.linear.x, twist_msg.angular.z);

    speed.data = twist_msg.linear.x;
    if (twist_msg.linear.x == 0.0){
      steer.data = 0.0;
    } else {
      steer.data = RATIO * atan(LENGTH * twist_msg.angular.z / twist_msg.linear.x);
    }

    pub_speed.publish(speed);
    pub_steer.publish(steer);
  }
//  pub_twist.publish(twist_msg);
}

/* Calculate Global Plan and Local Plan */
void goalCallback(const ros::TimerEvent& event)
{
  listener->transformPose("/map", ros::Time(0), pose_stamped, pose_stamped.frame_id_, pose_stamped);

  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.frame_id = "/map";
  robot_pose.header.stamp = pose_stamped.stamp_;
  tf::quaternionTFToMsg(pose_stamped.getRotation(), robot_pose.pose.orientation);
  robot_pose.pose.position.x = pose_stamped.getOrigin().x();
  robot_pose.pose.position.y = pose_stamped.getOrigin().y();
  robot_pose.pose.position.z = pose_stamped.getOrigin().z();

//  ROS_INFO("Transform Position: x-> %f, y-> %f", robot_pose.pose.position.x ,robot_pose.pose.position.y);
  pub_pose.publish(robot_pose);

  if (valid_goal && (control == 0))
  {
    std::vector<geometry_msgs::PoseStamped> plan;
    global_planner->makePlan(robot_pose, current_goal, plan);
    teb_planner->setPlan(plan);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_nav_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  listener = new tf::TransformListener;
  local_costmap = new costmap_2d::Costmap2DROS("local_costmap", *listener);
  global_costmap = new costmap_2d::Costmap2DROS("global_costmap", *listener);
  teb_planner = new teb_local_planner::TebLocalPlannerROS();
  teb_planner->initialize("TebLocalPlannerROS", listener, local_costmap);
  global_planner = new navfn::NavfnROS("global_planner", global_costmap);

  ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 1, recvGoal);
  ros::Subscriber sub_odom = n.subscribe("odom", 1, recvOdom);
  ros::Subscriber sub_control = n.subscribe("/control_mode", 1, recvControl);
//  pub_goal = n.advertise<geometry_msgs::PoseStamped>("/global_planner/goal", 1);
//  pub_twist = n.advertise<geometry_msgs::Twist>("/audibot/cmd_vel", 1);
  pub_speed = n.advertise<std_msgs::Float64>("/audibot/audi_speed_controller/speed_cmd", 1);
  pub_steer = n.advertise<std_msgs::Float64>("/audibot/audi_steering_controller/steering_cmd", 1);
  pub_pose = n.advertise<geometry_msgs::PoseStamped>("/audibot/current_pose", 1);

  double freq;
  pn.param("freq", freq, 20.0);
  ros::Timer control_timer = n.createTimer(ros::Duration(1.0/freq), timerCallback);

  ros::Timer goal_timer = n.createTimer(ros::Duration(1.0), goalCallback);

  ros::spin();
}
