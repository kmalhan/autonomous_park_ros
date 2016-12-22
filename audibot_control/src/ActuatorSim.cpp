#include <audibot_control/ActuatorSim.h>

namespace audibot_control{

ActuatorSim::ActuatorSim(const std::vector<double>& params)
{
  params_ = params;
  current_left_speed_ = 0.0;
  current_right_speed_ = 0.0;
  current_left_steering_vel_ = 0.0;
  current_right_steering_vel_ = 0.0;
}

double ActuatorSim::iterateLeftDrive(double target_speed, double dt)
{
  double target_accel = (target_speed - current_left_speed_) / dt;

  if (current_left_speed_ >= -1.0) {
    if (target_accel > params_[MAX_DRIVE_ACCEL]) {
      target_accel = params_[MAX_DRIVE_ACCEL];
    } else if (target_accel < -params_[MAX_DRIVE_DECEL]) {
      target_accel = -params_[MAX_DRIVE_DECEL];
    }
  } else{
    if (target_accel > params_[MAX_DRIVE_DECEL]) {
      target_accel = params_[MAX_DRIVE_DECEL];
    } else if (target_accel < -params_[MAX_DRIVE_ACCEL]) {
      target_accel = -params_[MAX_DRIVE_ACCEL];
    }
  }

  current_left_speed_ += (dt * target_accel);
  if (current_left_speed_ > params_[MAX_DRIVE_SPEED]){
    current_left_speed_ = params_[MAX_DRIVE_SPEED];
  }else if (current_left_speed_ < -params_[MAX_DRIVE_SPEED]){
    current_left_speed_ = -params_[MAX_DRIVE_SPEED];
  }

  return current_left_speed_;
}

double ActuatorSim::iterateRightDrive(double target_speed, double dt)
{
  double target_accel = (target_speed - current_right_speed_) / dt;

  if (current_right_speed_ >= -1.0) {
    if (target_accel > params_[MAX_DRIVE_ACCEL]) {
      target_accel = params_[MAX_DRIVE_ACCEL];
    } else if (target_accel < -params_[MAX_DRIVE_DECEL]) {
      target_accel = -params_[MAX_DRIVE_DECEL];
    }
  } else{
    if (target_accel > params_[MAX_DRIVE_DECEL]) {
      target_accel = params_[MAX_DRIVE_DECEL];
    } else if (target_accel < -params_[MAX_DRIVE_ACCEL]) {
      target_accel = -params_[MAX_DRIVE_ACCEL];
    }
  }

  current_right_speed_ += (dt * target_accel);
  if (current_right_speed_ > params_[MAX_DRIVE_SPEED]){
    current_right_speed_ = params_[MAX_DRIVE_SPEED];
  }else if (current_right_speed_ < -params_[MAX_DRIVE_SPEED]){
    current_right_speed_ = -params_[MAX_DRIVE_SPEED];
  }

  return current_right_speed_;
}

double ActuatorSim::iterateLeftSteering(double angle_error)
{

  current_left_steering_vel_ = params_[STEERING_GAIN] * angle_error;

  if (current_left_steering_vel_ > params_[MAX_STEERING_VEL]){
    current_left_steering_vel_ = params_[MAX_STEERING_VEL];
  }else if (current_left_steering_vel_ < -params_[MAX_STEERING_VEL]){
    current_left_steering_vel_ = -params_[MAX_STEERING_VEL];
  }

  return current_left_steering_vel_;
}

double ActuatorSim::iterateRightSteering(double angle_error)
{

  current_right_steering_vel_ = params_[STEERING_GAIN] * angle_error;

  if (current_right_steering_vel_ > params_[MAX_STEERING_VEL]){
    current_right_steering_vel_ = params_[MAX_STEERING_VEL];
  }else if (current_right_steering_vel_ < -params_[MAX_STEERING_VEL]){
    current_right_steering_vel_ = -params_[MAX_STEERING_VEL];
  }

  return current_right_steering_vel_;
}

}


