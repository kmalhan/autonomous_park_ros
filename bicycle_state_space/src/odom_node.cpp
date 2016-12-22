/*
 *  This node publishes Odometer message and transform from base_footprint to map (production)
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <ugv_course_libs/gps_conv.h>

/* Audibot information */
#define WHEEL_LENGTH  3.02367
#define STEER_RATIO   16.0

ros::Publisher pub_odom;
geometry_msgs::Twist current_twist;
geometry_msgs::Pose current_estimate;

double sample_time;
std::string parent_frame;
std::string child_frame;
double wheel_length;
double steer_ratio;
double current_steer_angle;

UTMCoords ref_utm;
bool updated = false;
double gps_x, gps_y;

/* Received steering angle from audibot */
void recvSteer(const std_msgs::Float64::ConstPtr& msg)
{
  current_steer_angle = msg->data;
}

/* Received twist message from audibot */
void recvTwist(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  current_twist = msg->twist;
}

/* Received GPS fix from audibot */
void recvFix(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static UTMCoords last_utm;
  static bool first_sample = true;

  if (first_sample)
  {
    first_sample = false;
    last_utm = UTMCoords(*msg);
    ref_utm = last_utm;
    return;
  }

  UTMCoords current_utm(*msg);
  tf::Vector3 rel_position = current_utm - ref_utm;
  gps_x = rel_position.getX();
  gps_y = rel_position.getY();
  updated = true;

//  ROS_INFO("GPS-> x: %f, y: %f", gps_x, gps_y);
}

/* State Space equation with GPS update */
void timerCallback(const ros::TimerEvent& event)
{
  static tf::TransformBroadcaster broadcaster;

  // State space equation for bicycle model
  double v = current_twist.linear.x;
  double pdot_c = current_twist.angular.z;

  double cos_psi = current_estimate.orientation.w * current_estimate.orientation.w
                  - current_estimate.orientation.z * current_estimate.orientation.z;
  double sin_psi = 2 * current_estimate.orientation.w * current_estimate.orientation.z;
  double psi = atan2(sin_psi, cos_psi);

  current_estimate.position.x += sample_time * v * cos_psi;
  current_estimate.position.y += sample_time * v * sin_psi;

  if (updated){
  current_estimate.position.x = gps_x;
  current_estimate.position.y = gps_y;
  updated = false;
  }

//  ROS_INFO("SS->  x: %f, y: %f", current_estimate.position.x, current_estimate.position.y);

  double new_psi = psi + sample_time * ((v / wheel_length) * tan(current_steer_angle / steer_ratio));
  current_estimate.orientation = tf::createQuaternionMsgFromYaw(new_psi);

  // Populate TF transform
  tf::StampedTransform transform;
  transform.frame_id_ = parent_frame;
  transform.child_frame_id_ = child_frame;
  transform.stamp_ = event.current_real;
  transform.setOrigin(tf::Vector3(current_estimate.position.x, current_estimate.position.y, 0));
  transform.setRotation(tf::Quaternion(0, 0, current_estimate.orientation.z, current_estimate.orientation.w));
  broadcaster.sendTransform(transform);

  // Populate and publish odometry
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = event.current_real;
  odom_msg.header.frame_id = parent_frame;
  odom_msg.child_frame_id = child_frame;
  odom_msg.pose.pose = current_estimate;
  odom_msg.twist.twist = current_twist;
  pub_odom.publish(odom_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  ros::Subscriber sub_twist = n.subscribe("/audibot/twist", 1, recvTwist);
  ros::Subscriber sub_steer_angle = n.subscribe("/audibot/steering_wheel_angle", 1, recvSteer);
  ros::Subscriber sub_gps = n.subscribe("/audibot/gps/fix", 1, recvFix);
  pub_odom = n.advertise<nav_msgs::Odometry>("odom", 1);

  pn.param("sample_time", sample_time, 0.02);
  pn.param("parent_frame", parent_frame, std::string("map"));    // Self Test: map,     (odom)
  pn.param("child_frame", child_frame, std::string("vehicle"));  // Self Test: vehicle, (base_footprint)
  pn.param("wheel_length", wheel_length, WHEEL_LENGTH);
  pn.param("steer_ratio", steer_ratio, STEER_RATIO);

  ros::Timer state_space_timer = n.createTimer(ros::Duration(sample_time), timerCallback);

//  current_estimate.position.x = -40.0;
//  current_estimate.position.y = 40.0;
//  current_estimate.orientation = tf::createQuaternionMsgFromYaw(0);

  ros::spin();
}
