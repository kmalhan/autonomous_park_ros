#ifndef ACTUATORSIM_H_
#define ACTUATORSIM_H_

#include <vector>
#include <math.h>

namespace audibot_control
{

enum
{
  MAX_DRIVE_SPEED, MAX_DRIVE_ACCEL, MAX_DRIVE_DECEL, STEERING_GAIN, MAX_STEERING_VEL, NUM_PARAMS
};

class ActuatorSim
{
public:
  ActuatorSim(const std::vector<double>& params);
  double iterateLeftDrive(double target_speed, double dt);
  double iterateRightDrive(double target_speed, double dt);
  double iterateLeftSteering(double angle_error);
  double iterateRightSteering(double angle_error);
private:

  std::vector<double> params_;
  double current_left_speed_;
  double current_right_speed_;
  double current_left_steering_vel_;
  double current_right_steering_vel_;
};

}

#endif /* ACTUATORSIM_H_ */
