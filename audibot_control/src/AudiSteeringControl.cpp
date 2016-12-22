#include <audibot_control/AudiSteeringControl.h>

namespace audibot_control{

bool AudiSteeringController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n)
{
  sub_cmd_ = n.subscribe("steering_cmd", 1, &AudiSteeringController::recvCommand, this);

  target_steering_wheel_angle_ = 0.0;

  loadParams(n);

  left_steer_joint_ = hw->getHandle(left_steer_joint_name_);
  right_steer_joint_ = hw->getHandle(right_steer_joint_name_);
  return true;
}

void AudiSteeringController::update(const ros::Time& time, const ros::Duration& period)
{
  double t_alph = tan(target_steering_wheel_angle_ / steering_ratio_);

  double left_steer = atan(wheelbase_ * t_alph / (wheelbase_ - 0.5 * track_ * t_alph));
  double right_steer = atan(wheelbase_ * t_alph / (wheelbase_ + 0.5 * track_ * t_alph));

  left_steer_joint_.setCommand(left_steer);
  right_steer_joint_.setCommand(right_steer);
}

void AudiSteeringController::starting(const ros::Time& time)
{
}

void AudiSteeringController::stopping(const ros::Time& time)
{
}

void AudiSteeringController::loadParams(ros::NodeHandle& n)
{
  n.param("control_timeout", timeout_, 0.25);
  n.param("left_steer_joint", left_steer_joint_name_, std::string("wheel_fl_steer"));
  n.param("right_steer_joint", right_steer_joint_name_, std::string("wheel_fr_steer"));
  n.param("wheelbase", wheelbase_, 3.023);
  n.param("track", track_, 0.5);
  n.param("steering_ratio", steering_ratio_, 16.0);

  double max_tire_angle;
  n.param("max_tire_angle", max_tire_angle, 0.5);

  max_steering_wheel_angle_ = steering_ratio_ * wheelbase_ / (wheelbase_ / tan(max_tire_angle) + 0.5 * track_);
}

void AudiSteeringController::recvCommand(const std_msgs::Float64::ConstPtr& msg)
{
  command_stamp_ = ros::Time::now();
  target_steering_wheel_angle_ = msg->data;
  if (fabs(target_steering_wheel_angle_) > max_steering_wheel_angle_){
    target_steering_wheel_angle_ = (target_steering_wheel_angle_ > 0 ? max_steering_wheel_angle_ : -max_steering_wheel_angle_);
  }
}

}
