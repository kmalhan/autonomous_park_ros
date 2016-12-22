#include <audibot_control/AudiSpeedControl.h>

namespace audibot_control{

void AudiSpeedController::loadParams(ros::NodeHandle& n)
{
  n.param("control_timeout", timeout_, 0.25);
  n.param("left_wheel_joint", left_wheel_name_, std::string("wheel_rl"));
  n.param("right_wheel_joint", right_wheel_name_, std::string("wheel_rr"));
  n.param("wheel_radius", wheel_radius_, 0.5);
  n.param("wheelbase", wheelbase_, 1.0);
  n.param("track", track_, 0.5);

  std::vector<double> actuator_params(NUM_PARAMS);
  n.param("max_drive_speed", actuator_params[MAX_DRIVE_SPEED], 60.0);
  n.param("max_drive_accel", actuator_params[MAX_DRIVE_ACCEL], 4.0);
  n.param("max_drive_decel", actuator_params[MAX_DRIVE_DECEL], 8.0);
  actuators_ = new ActuatorSim(actuator_params);
}

void AudiSpeedController::recvCommand(const std_msgs::Float64::ConstPtr& msg)
{
  command_stamp_ = ros::Time::now();
  target_left_speed_ = msg->data / wheel_radius_;
  target_right_speed_ = msg->data / wheel_radius_;
}

bool AudiSpeedController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& n)
{
  sub_cmd_ = n.subscribe("speed_cmd", 1, &AudiSpeedController::recvCommand, this);

  target_left_speed_ = 0;
  target_right_speed_ = 0;

  loadParams(n);

  left_wheel_joint_ = hw->getHandle(left_wheel_name_);
  right_wheel_joint_ = hw->getHandle(right_wheel_name_);
  return true;
}

void AudiSpeedController::update(const ros::Time& time, const ros::Duration& period)
{
  double left_actual_speed;
  double right_actual_speed;

  if ((time - command_stamp_).toSec() >= timeout_){
    left_actual_speed = actuators_->iterateLeftDrive(0, period.toSec());
    right_actual_speed = actuators_->iterateRightDrive(0, period.toSec());
  }else{
    left_actual_speed = actuators_->iterateLeftDrive(target_left_speed_, period.toSec());
    right_actual_speed = actuators_->iterateRightDrive(target_right_speed_, period.toSec());
  }

  left_wheel_joint_.setCommand(left_actual_speed);
  right_wheel_joint_.setCommand(right_actual_speed);
}

void AudiSpeedController::starting(const ros::Time& time)
{
}

void AudiSpeedController::stopping(const ros::Time& time)
{
}

}


