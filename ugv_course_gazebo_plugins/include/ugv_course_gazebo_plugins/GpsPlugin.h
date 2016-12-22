
#ifndef INCLUDE_UGV_COURSE_GAZEBO_PLUGINS_GPSPLUGIN_H_
#define INCLUDE_UGV_COURSE_GAZEBO_PLUGINS_GPSPLUGIN_H_

#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ugv_course_libs/gps_conv.h>
#include <ugv_course_gazebo_plugins/GpsPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

namespace gazebo{

  class GpsPlugin : public ModelPlugin
  {
  public:
    GpsPlugin();
    virtual ~GpsPlugin();

  protected:
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    virtual void Reset();

  private:

    void reconfig(ugv_course_gazebo_plugins::GpsPluginConfig& config, uint32_t level);
    void update(const ros::TimerEvent& event);

    ros::NodeHandle* n_;
    ros::Timer update_timer_;
    ros::Publisher pub_fix_;
    ros::Publisher pub_heading_;

    boost::shared_ptr<dynamic_reconfigure::Server<ugv_course_gazebo_plugins::GpsPluginConfig> > srv_;

    UTMCoords ref_point_;
    physics::LinkPtr link_;
    double update_rate_;
    bool publish_heading_;
    double ant_offset_x_;
    double ant_offset_y_;
    double ant_offset_z_;
  };

  GZ_REGISTER_MODEL_PLUGIN(GpsPlugin)
}



#endif /* INCLUDE_UGV_COURSE_GAZEBO_PLUGINS_GPSPLUGIN_H_ */
