#include <ugv_course_gazebo_plugins/PosePlugin.h>

namespace gazebo{

PosePlugin::PosePlugin()
{
}

PosePlugin::~PosePlugin()
{
  twist_timer_.stop();
  n_->shutdown();
  delete n_;
}

void PosePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Gazebo
  model_name_ = model->GetName();
  base_link_name_ = model->GetLink()->GetName();
  pub_tf_ = false;

  if (!sdf->HasElement("twist_publish_rate")) {
    ROS_WARN("[twist_publish_rate] SDF element not defined! Defaulting to 50 Hz");
    twist_publish_rate_ = 50.0;
  } else {
    sdf->GetElement("twist_publish_rate")->GetValue()->Get(twist_publish_rate_);
  }

  if (sdf->HasElement("pub_tf")){
    sdf->GetElement("pub_tf")->GetValue()->Get(pub_tf_);
  }

  if (sdf->HasElement("pub_odom")){
    sdf->GetElement("pub_odom")->GetValue()->Get(pub_odom_);
  }

  // ROS
  n_ = new ros::NodeHandle(model_name_);
  broadcaster_ = new tf::TransformBroadcaster;

  pub_actual_twist_ = n_->advertise<geometry_msgs::TwistStamped>("twist", 1);
  sub_model_states_ = n_->subscribe("/gazebo/model_states", 1, &PosePlugin::recvModelStates, this);

  if (!model_name_.compare("audibot")){
    pub_steering_wheel_angle_ = n_->advertise<std_msgs::Float64>("steering_wheel_angle", 1);
    sub_joint_states_ = n_->subscribe("joint_states", 1, &PosePlugin::recvJointStates, this);
  }

  if (pub_odom_){
    pub_actual_odom_ = n_->advertise<nav_msgs::Odometry>("odom", 1);
  }

  twist_timer_ = n_->createTimer(ros::Duration(1.0 / twist_publish_rate_), &PosePlugin::twistUpdate, this);

  twist_.header.frame_id = base_link_name_;
}

void PosePlugin::recvJointStates(const sensor_msgs::JointState::ConstPtr& msg)
{
  double wheelbase = 3.02367;
  double track = 1.9;
  double steering_gain = 16.0;

  int idx = -1;
  for (size_t i=0; i<msg->name.size(); i++){
    if (!msg->name[i].compare("wheel_fl_steer")){
      idx = (int)i;
    }
  }

  if (idx < 0){
    ROS_WARN("Steering angle not found");
    return;
  }

  double tal = tan(msg->position[idx]);
  steering_angle_msg_.data = steering_gain * atan(wheelbase * tal / (wheelbase + 0.5 * track * tal));
}

void PosePlugin::recvModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  ros::Time current_time = ros::Time::now();

  int idx = 0;
  for (size_t i=0; i<msg->name.size(); i++){
    if (!msg->name[i].compare(model_name_)){
      idx = i;
      break;
    }
  }

  double cpsi = msg->pose[idx].orientation.w * msg->pose[idx].orientation.w - msg->pose[idx].orientation.z * msg->pose[idx].orientation.z;
  double spsi = 2 * msg->pose[idx].orientation.w * msg->pose[idx].orientation.z;

  twist_.twist.linear.x = msg->twist[idx].linear.x * cpsi + msg->twist[idx].linear.y * spsi;
  twist_.twist.linear.y = -msg->twist[idx].linear.x * spsi + msg->twist[idx].linear.y * cpsi;
  twist_.twist.angular.z = msg->twist[idx].angular.z;

  if (pub_tf_){
    tf::StampedTransform pose;
    pose.frame_id_ = "/world";
    pose.child_frame_id_ = base_link_name_;
    pose.stamp_ = current_time;
    pose.setOrigin(tf::Vector3(msg->pose[idx].position.x, msg->pose[idx].position.y, msg->pose[idx].position.z));
    pose.setRotation(tf::Quaternion(msg->pose[idx].orientation.x, msg->pose[idx].orientation.y, msg->pose[idx].orientation.z, msg->pose[idx].orientation.w));
    broadcaster_->sendTransform(pose);
  }

  if (pub_odom_){
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "/world";
    odom_msg.header.stamp = current_time;
    odom_msg.child_frame_id = base_link_name_;

    odom_msg.pose.pose.position.x = msg->pose[idx].position.x;
    odom_msg.pose.pose.position.y = msg->pose[idx].position.y;
    odom_msg.pose.pose.position.z = msg->pose[idx].position.z;
    odom_msg.pose.pose.orientation.w = msg->pose[idx].orientation.w;
    odom_msg.pose.pose.orientation.x = msg->pose[idx].orientation.x;
    odom_msg.pose.pose.orientation.y = msg->pose[idx].orientation.y;
    odom_msg.pose.pose.orientation.z = msg->pose[idx].orientation.z;
    odom_msg.twist.twist = twist_.twist;

    pub_actual_odom_.publish(odom_msg);
  }
}

void PosePlugin::Reset()
{
}

void PosePlugin::twistUpdate(const ros::TimerEvent& event)
{
  twist_.header.stamp = event.current_real;
  pub_actual_twist_.publish(twist_);

  if (!model_name_.compare("audibot")){
    pub_steering_wheel_angle_.publish(steering_angle_msg_);
  }
}

}
