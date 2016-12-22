#include <ugv_course_gazebo_plugins/ViewControlPlugin.h>

namespace gazebo{

ViewControlPlugin::ViewControlPlugin()
{
}

ViewControlPlugin::~ViewControlPlugin()
{
}

void ViewControlPlugin::Load(int argc, char** argv)
{
  ros::init(argc, argv, "view_control");
  n_ = new ros::NodeHandle("view_control");

  async_ = new ros::AsyncSpinner(3);
  async_->start();
}

void ViewControlPlugin::Init()
{
  ros::NodeHandle cfg_nh(*n_, "move_cam");

  connections_.push_back(
            event::Events::ConnectPreRender(
              boost::bind(&ViewControlPlugin::Update, this)));
}

void ViewControlPlugin::Reset()
{
}

void ViewControlPlugin::Update()
{
  if (!user_cam_)
  {
    // Get a pointer to the active user camera
    user_cam_ = gui::get_active_camera();
  }

  user_cam_->SetWorldPosition(math::Vector3(0,0,10));
}

}
