
#ifndef AUDISPEEDCONTROLLER_H_
#define AUDISPEEDCONTROLLER_H_

#include <audibot_control/ActuatorSim.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

namespace audibot_control{

class AudiSpeedController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);
private:
  void loadParams(ros::NodeHandle& n);
  void recvCommand(const std_msgs::Float64::ConstPtr& msg);

  ros::Subscriber sub_cmd_;

  hardware_interface::JointHandle left_wheel_joint_;
  hardware_interface::JointHandle right_wheel_joint_;

  ActuatorSim* actuators_;
  ros::Time command_stamp_;
  double target_left_speed_;
  double target_right_speed_;

  // Parameters
  std::string left_wheel_name_;
  std::string right_wheel_name_;
  double wheel_radius_;
  double timeout_;
  double wheelbase_;
  double track_;
};

PLUGINLIB_EXPORT_CLASS(audibot_control::AudiSpeedController, controller_interface::ControllerBase);

}

#endif /* AUDISPEEDCONTROLLER_H_ */
