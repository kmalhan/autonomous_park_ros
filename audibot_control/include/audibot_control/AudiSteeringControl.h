
#ifndef INCLUDE_AUDIBOT_CONTROL_AUDISTEERINGCONTROL_H_
#define INCLUDE_AUDIBOT_CONTROL_AUDISTEERINGCONTROL_H_

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

namespace audibot_control{

class AudiSteeringController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);
private:
  void loadParams(ros::NodeHandle& n);
  void recvCommand(const std_msgs::Float64::ConstPtr& msg);

  hardware_interface::JointHandle left_steer_joint_;
  hardware_interface::JointHandle right_steer_joint_;

  ros::Time command_stamp_;
  ros::Subscriber sub_cmd_;
  double target_steering_wheel_angle_;

  // Parameters
  std::string left_steer_joint_name_;
  std::string right_steer_joint_name_;
  double steering_ratio_;
  double max_steering_wheel_angle_;
  double timeout_;
  double wheelbase_;
  double track_;

};

PLUGINLIB_EXPORT_CLASS(audibot_control::AudiSteeringController, controller_interface::ControllerBase);

}

#endif /* INCLUDE_AUDIBOT_CONTROL_AUDISTEERINGCONTROL_H_ */
