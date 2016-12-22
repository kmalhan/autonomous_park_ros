#include <ugv_course_gazebo_plugins/GpsPlugin.h>

namespace gazebo{

GpsPlugin::GpsPlugin()
{
}

GpsPlugin::~GpsPlugin()
{
  update_timer_.stop();
  srv_.reset();
  n_->shutdown();
  delete n_;
}

void GpsPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Gazebo
  link_ = model->GetLink("base_footprint");

  if (!link_){
    ROS_WARN("base_footprint doesn't exist!");
  }

  if (!sdf->HasElement("update_rate")){
    ROS_WARN("[update_rate] SDF element not defined! Defaulting to 1.0 sec");
    update_rate_ = 1.0;
  }else{
    sdf->GetElement("update_rate")->GetValue()->Get(update_rate_);
  }

  if (!sdf->HasElement("ant_offset_x")){
    ROS_WARN("[ant_offset_x] SDF element not defined! Defaulting to 0.0 meters");
    ant_offset_x_ = 0.0;
  }else{
    sdf->GetElement("ant_offset_x")->GetValue()->Get(ant_offset_x_);
  }

  if (!sdf->HasElement("ant_offset_y")){
    ROS_WARN("[ant_offset_y] SDF element not defined! Defaulting to 0.0 meters");
    ant_offset_y_ = 0.0;
  }else{
    sdf->GetElement("ant_offset_y")->GetValue()->Get(ant_offset_y_);
  }

  if (!sdf->HasElement("ant_offset_z")){
    ROS_WARN("[ant_offset_z] SDF element not defined! Defaulting to 0.0 meters");
    ant_offset_z_ = 0.0;
  }else{
    sdf->GetElement("ant_offset_z")->GetValue()->Get(ant_offset_z_);
  }

  if (!sdf->HasElement("pub_heading")){
    ROS_WARN("[pub_heading] SDF element not defined! Defaulting to false");
    publish_heading_ = false;
  }else{
    sdf->GetElement("pub_heading")->GetValue()->Get(publish_heading_);
  }

  std::string link_name;
  if (!sdf->HasElement("link_name")){
    ROS_WARN("[link_name] SDF element not defined! Defaulting to 'gps'");
    link_name = "gps";
  }else{
    link_name = sdf->GetElement("link_name")->GetValue()->GetAsString();
  }

  // ROS
  n_ = new ros::NodeHandle(sdf->GetParent()->Get<std::string>("name"), link_name);

  srv_.reset(new dynamic_reconfigure::Server<ugv_course_gazebo_plugins::GpsPluginConfig>(*n_));
  srv_->setCallback(boost::bind(&GpsPlugin::reconfig, this, _1, _2));

  pub_fix_ = n_->advertise<sensor_msgs::NavSatFix>("fix", 1);
  if (publish_heading_){
    pub_heading_ = n_->advertise<std_msgs::Float64>("heading", 1);
  }

  update_timer_ = n_->createTimer(ros::Duration(1.0 / update_rate_), &GpsPlugin::update, this);
}

void GpsPlugin::Reset()
{
}

void GpsPlugin::update(const ros::TimerEvent& event)
{
  if (!link_) return;

  math::Pose pose = link_->GetWorldPose();
  double heading = atan2(2 * (pose.rot.w * pose.rot.z + pose.rot.x * pose.rot.y),
                         1 - 2 * (pose.rot.y * pose.rot.y + pose.rot.z * pose.rot.z));

  tf::Vector3 ant_offset_vect;
  ant_offset_vect.setX(ant_offset_x_ * cos(heading) - ant_offset_y_ * sin(heading));
  ant_offset_vect.setY(ant_offset_x_ * sin(heading) + ant_offset_y_ * cos(heading));
  ant_offset_vect.setZ(ant_offset_z_);

  LatLon geodetic_position = ref_point_ + (tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z) + ant_offset_vect);
  sensor_msgs::NavSatFix fix_msg;
  fix_msg.header.stamp = event.current_real;
  fix_msg.latitude = geodetic_position.getLat();
  fix_msg.longitude = geodetic_position.getLon();
  fix_msg.altitude = geodetic_position.getAlt();
  pub_fix_.publish(fix_msg);

  if (publish_heading_){
    std_msgs::Float64 heading_msg;
    heading_msg.data = 180.0 / M_PI * (M_PI / 2 - heading);
    if (heading_msg.data < 0){
      heading_msg.data += 360;
    }
    pub_heading_.publish(heading_msg);
  }
}

void GpsPlugin::reconfig(ugv_course_gazebo_plugins::GpsPluginConfig& config, uint32_t level)
{
  ref_point_ = UTMCoords(LatLon(config.ref_lat, config.ref_lon, 0.0));
}

}
