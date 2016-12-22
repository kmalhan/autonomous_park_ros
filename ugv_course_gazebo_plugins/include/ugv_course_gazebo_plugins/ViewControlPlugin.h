
#ifndef INCLUDE_UGV_COURSE_GAZEBO_PLUGINS_VIEWCONTROLPLUGIN_H_
#define INCLUDE_UGV_COURSE_GAZEBO_PLUGINS_VIEWCONTROLPLUGIN_H_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>

namespace gazebo
{

class ViewControlPlugin : public SystemPlugin
{
public:
  ViewControlPlugin();
  virtual ~ViewControlPlugin();
protected:
  virtual void Load(int argc, char** argv);
  virtual void Init();
  virtual void Reset();
private:
  void Update();

  rendering::UserCameraPtr user_cam_;
  std::vector<event::ConnectionPtr> connections_;
  ros::AsyncSpinner* async_;

  ros::NodeHandle* n_;

};

GZ_REGISTER_SYSTEM_PLUGIN(ViewControlPlugin)
}



#endif /* INCLUDE_UGV_COURSE_GAZEBO_PLUGINS_VIEWCONTROLPLUGIN_H_ */
