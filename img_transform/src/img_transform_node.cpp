/*
 * Try to implement image transform using image geometry package (under development)
 */
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

tf::TransformListener* listener;
sensor_msgs::CameraInfo info;
ros::Publisher pub_cloud;

void recvInfo(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  info = *msg;
}

void recvImage(const sensor_msgs::Image::ConstPtr& msg)
{
  //-- Get the image as Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  cv::Mat canny_img = cv_ptr->image;

  //-- Lookup Transform
  tf::StampedTransform camera_to_base, base_to_map;
  bool good_transform = false;

  try {
  listener->lookupTransform("/base_footprint", "/front_camera", ros::Time(0), camera_to_base);
  listener->lookupTransform("/map", "/base_footprint", ros::Time(0), base_to_map);
  good_transform = true;
  } catch (tf::TransformException &ex){
    ROS_WARN("%s", ex.what());
  }

  //-- Image to PointCloud
  image_geometry::PinholeCameraModel model;
  info.header.frame_id = "/front_camera";
  model.fromCameraInfo(info);

  //-- Camera location in audibot frame
  tf::Vector3 v0 = camera_to_base(tf::Vector3(0, 0, 0));
//  ROS_INFO("cam loc in bot frame %f, %f, %f", v0.getX(), v0.getY(), v0.getZ());
  geometry_msgs::Point32 point;
  int width = canny_img.cols;
  int height = canny_img.rows;
  sensor_msgs::PointCloud cloud;
  int count = 0;
  int z = 0;

  for(int i=0; i<width; i++)
  {
    for(int j=0; j<height; j++)
    {
      if(canny_img.at<unsigned char>(j, i) == 255)
      {
        count += 1;

        cv::Point2d p2d(i, j);
        cv::Point3d p3d = model.projectPixelTo3dRay(p2d);
        //-- pixel in the global coordinate frame
        tf::Vector3 v1 = camera_to_base(tf::Vector3(p3d.x, p3d.y, p3d.z));
        //-- compute common part of the transformation
        // d=(z-z_0)/(z_1-z_0)
        double d = (z - v0.z()) / (v1.z() - v0.z());
        //compute x and y transformation
        double x = d * (v1.x() - v0.x()) + v0.x();
        double y = d * (v1.y() - v0.y()) + v0.y();

        point.x = x;
        point.y = y;
        point.z = z;

        cloud.points.push_back(point);
      }
    }
  }
//  ROS_INFO("Count: %d", count);
  cloud.header = cv_ptr->header;
  cloud.header.frame_id = "/front_camera";

  pub_cloud.publish(cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "img_transform_node");
  ros::NodeHandle n;

  listener = new tf::TransformListener;
  ros::Subscriber sub_image = n.subscribe("/front_camera/canny", 1, recvImage);
  ros::Subscriber sub_cam_info = n.subscribe("/audibot/front_camera/camera_info", 1, recvInfo);
  pub_cloud = n.advertise<sensor_msgs::PointCloud>("/front_camera/cloud", 1);

  ros::spin();
}
