/*
 * Implements manual image tranform (Under development)
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#define Y_PIX   800
#define Z_PIX   800
#define H_FOV   1.3962634
#define H       1.3
#define PITCH   0.4

using namespace Eigen;
using namespace visualization_msgs;

geometry_msgs::PoseArray points;
tf::TransformListener* listener;
ros::Publisher pub_mark;
MarkerArray vis_point;


void recvPoint(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  points = *msg;
}

void timerCallback(const ros::TimerEvent& event)
{
  if (points.poses.size() > 0)
  {
    int y_pix = Y_PIX;
    int z_pix = Z_PIX;
    double alpha = (H_FOV / y_pix);
    double beta = (H_FOV / z_pix);

    double alpha_y = -((y_pix / 2.0) * alpha);
    double beta_z = -((z_pix / 2.0) * beta);

    Matrix3f p_matrix;
    p_matrix << alpha, 0, alpha_y,
                0, beta, beta_z,
                0, 0, 1;

    vis_point.markers.resize(points.poses.size());

    for(int i=0; i<(points.poses.size()-1); i++)
    {
      //-- Step1: Pixels to Field of View
      Matrix<float, 3, 1> pixel_angle;
      Matrix<float, 3, 1> pixel_pos;

      pixel_pos << points.poses[i].position.y, points.poses[i].position.z, 1;
      pixel_angle = p_matrix * pixel_pos;

      //-- Step2: Field of View to Camera Frame
      double rj;
      geometry_msgs::Vector3Stamped camera_f;
      camera_f.header.frame_id = "/front_camera"; //points.header.frame_id;
      camera_f.header.stamp = points.header.stamp;

      rj = (H / std::sin(PITCH + (double)pixel_angle(1, 0)));

      camera_f.vector.x = rj * std::cos((double)pixel_angle(1, 0));
      camera_f.vector.y = rj * std::tan((double)pixel_angle(0, 0));
      camera_f.vector.z = -(rj * std::sin((double)pixel_angle(1, 0)));

      //-- Step3: Camera Frame to BaseFootPrint
      geometry_msgs::Vector3Stamped base_f;
      base_f.header.frame_id = "/base_footprint";
      base_f.header.stamp = camera_f.header.stamp;

      listener->transformVector("/base_footprint", ros::Time(0), camera_f, "/front_camera", base_f);

      //-- Step4: BaseFootPrint to Map
      geometry_msgs::Vector3Stamped map_f;
      map_f.header.frame_id = "/map";
      map_f.header.stamp = camera_f.header.stamp;

      listener->transformVector("/map", ros::Time(0), base_f, "/base_footprint", map_f);

      //-- Step5: Visualize on Rviz(?)
      Marker new_marker;
      new_marker.header.stamp = event.current_real;
      new_marker.header.frame_id = "/map";
      new_marker.action = Marker::ADD;
      new_marker.color.a = 1.0;
      new_marker.id = 0;
      new_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      new_marker.color.r = 0.0;
      new_marker.color.g = 1.0;
      new_marker.color.b = 0.0;
      new_marker.type = Marker::CYLINDER;
      new_marker.scale.x = 0.2;
      new_marker.scale.y = 0.2;
      new_marker.scale.z = 0.2;
      new_marker.pose.position.z = 0.0;

      new_marker.pose.position.x = map_f.vector.x;
      new_marker.pose.position.y = map_f.vector.y;
      vis_point.markers.push_back(new_marker);
      new_marker.id = i;
    }
    pub_mark.publish(vis_point);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_transform");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  listener = new tf::TransformListener;
  ros::Subscriber sub_point = n.subscribe("/front_camera/point", 1, recvPoint);
  ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);

  pub_mark = n.advertise<visualization_msgs::MarkerArray>("/lane", 1);



  ros::spin();
}
