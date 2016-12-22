
#ifndef INCLUDE_UGV_COURSE_GAZEBO_PLUGINS_POSEPLUGIN_H_
#define INCLUDE_UGV_COURSE_GAZEBO_PLUGINS_POSEPLUGIN_H_

#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

namespace gazebo
{

class PosePlugin : public ModelPlugin
{

public:
  PosePlugin();
  virtual ~PosePlugin();

protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void Reset();

private:
  void recvModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg);
  void recvJointStates(const sensor_msgs::JointState::ConstPtr& msg);
  void twistUpdate(const ros::TimerEvent& event);

  ros::NodeHandle* n_;
  ros::Timer tf_timer_;
  ros::Timer twist_timer_;
  ros::Publisher pub_actual_twist_;
  ros::Publisher pub_actual_odom_;
  ros::Publisher pub_steering_wheel_angle_;
  ros::Subscriber sub_model_states_;
  ros::Subscriber sub_joint_states_;

  tf::TransformBroadcaster* broadcaster_;

  std::string model_name_;
  std::string base_link_name_;
  double twist_publish_rate_;
  bool pub_tf_;
  bool pub_odom_;
  geometry_msgs::TwistStamped twist_;
  sensor_msgs::JointState joints_;
  std_msgs::Float64 steering_angle_msg_;

};

GZ_REGISTER_MODEL_PLUGIN(PosePlugin)

}



#endif /* INCLUDE_UGV_COURSE_GAZEBO_PLUGINS_POSEPLUGIN_H_ */
