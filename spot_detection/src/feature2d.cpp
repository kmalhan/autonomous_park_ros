/*
 * This node implemented feature2d. Not used in the project (Not Used)
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <dynamic_reconfigure/server.h>
#include <spot_detection/lane_extractConfig.h>

lane_extract::lane_extractConfig cfg;

int hough_threshold, hough_min_line, hough_max_line;
cv::Mat templ;

void recvImage(const sensor_msgs::Image::ConstPtr msg)
{
  // Get the image as Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat raw_img = cv_ptr->image;
  cv::imshow("Raw Image", raw_img);

  // Convert to HSV
  cv::Mat raw_hsv_img;
  cv::cvtColor(raw_img, raw_hsv_img, CV_BGR2HSV);

  // Split image into HSV channels
  std::vector<cv::Mat> split_img;
  cv::split(raw_hsv_img, split_img);

  // Threshold Hue
  cv::Mat t1, t2;
  cv::threshold(split_img[0], t1, cfg.h_center - cfg.h_width, 255, CV_THRESH_BINARY);
  cv::threshold(split_img[0], t2, cfg.h_center + cfg.h_width, 255, CV_THRESH_BINARY_INV);

  cv::Mat h_thres;
  cv::bitwise_and(t1, t2, h_thres);

  // Threshold Saturation
  cv::Mat s_thres;
  cv::threshold(split_img[1], s_thres, cfg.v_thres, 255, CV_THRESH_BINARY);

  // Threshold Value
  cv::Mat v_thres;
  cv::threshold(split_img[2], v_thres, cfg.v_thres, 255, CV_THRESH_BINARY);

  // Erode Line and Dilate
  cv::Mat erode;
  cv::Mat dilate;
  cv::erode(h_thres, erode, cv::Mat::ones(3, 3, CV_8U));
  cv::dilate(erode, dilate, cv::Mat::ones(3, 3, CV_8U));

  // Canny Edge Detection
  cv::Mat canny_img_orig, canny_img;
  cv::Canny(dilate, canny_img_orig, 50, 200, 3);
  cv::dilate(canny_img_orig, canny_img, cv::Mat::ones(5, 5, CV_8U));
  cv::imshow("Canny", canny_img);

  /*------------------------- Feature2d -------------------------------------*/

  //-- Step1: Detect the keypoints using SURF Detector
  int minHessian = 400;
  cv::SurfFeatureDetector detector(minHessian);
  std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

  detector.detect(raw_img, keypoints_scene);
  detector.detect(templ, keypoints_object);

  //-- Step2: Calculate descriptors (feature vectors)
  cv::SurfDescriptorExtractor extractor;
  cv::Mat descriptors_object, descriptors_scene, descriptors_scene_t;

  extractor.compute(raw_img, keypoints_scene, descriptors_scene);
  extractor.compute(templ, keypoints_object, descriptors_object);

  //-- FlannBasedMatcher needs CV_32F as format

  descriptors_scene.convertTo(descriptors_scene_t, CV_32F);

  if(descriptors_scene.type()!=CV_32F) {
    descriptors_scene.convertTo(descriptors_scene, CV_32F);
  }

  if(descriptors_object.type()!=CV_32F) {
    descriptors_object.convertTo(descriptors_object, CV_32F);
  }
  ROS_INFO("type: %d", descriptors_scene.type());
  ROS_INFO("type: %d", descriptors_object.type());
  //-- Step3: Matching descriptor vectors using FLANN matcher
  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> matches;
  matcher.match(descriptors_object, descriptors_scene_t, matches, cv::Mat());

  //-- Quick calculation of max and min distances between keypoints
  double max_dist = 0;
  double min_dist = 100;

  for(int i=0; i<descriptors_object.rows; i++)
  {
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }
//  ROS_INFO("Min dist: %f, Max dist: %f", min_dist, max_dist);

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist)
  std::vector<cv::DMatch> good_matches;

  for(int i=0; i<descriptors_object.rows; i++)
  {
    if(matches[i].distance < 3*min_dist)
    {
      good_matches.push_back(matches[i]);
    }
  }

  cv::Mat img_matches;
  cv::drawMatches(templ, keypoints_object, raw_img, keypoints_scene,
              good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
              cv::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Localize the object
  std::vector<cv::Point2f> obj;
  std::vector<cv::Point2f> scene;

  for(int i=0; i<good_matches.size(); i++)
  {
    //-- Get the keypoints from the good matches
    obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
    scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
  }

  cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC);

  //-- Get the corners from the image_1 (the object to be "detected")
  std::vector<cv::Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0);
  obj_corners[1] = cvPoint(raw_img.cols, 0);
  obj_corners[2] = cvPoint(raw_img.cols, raw_img.rows);
  obj_corners[3] = cvPoint(0, raw_img.rows);

  std::vector<cv::Point2f> scene_corners(4);
  cv::perspectiveTransform(obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  cv::line( img_matches, scene_corners[0] + cv::Point2f( raw_img.cols, 0), scene_corners[1] + cv::Point2f( raw_img.cols, 0), cv::Scalar( 0, 255, 0), 4 );
  cv::line( img_matches, scene_corners[1] + cv::Point2f( raw_img.cols, 0), scene_corners[2] + cv::Point2f( raw_img.cols, 0), cv::Scalar( 0, 255, 0), 4 );
  cv::line( img_matches, scene_corners[2] + cv::Point2f( raw_img.cols, 0), scene_corners[3] + cv::Point2f( raw_img.cols, 0), cv::Scalar( 0, 255, 0), 4 );
  cv::line( img_matches, scene_corners[3] + cv::Point2f( raw_img.cols, 0), scene_corners[0] + cv::Point2f( raw_img.cols, 0), cv::Scalar( 0, 255, 0), 4 );

  //-- Show
  cv::imshow("Detection", img_matches);

}

void reconfig(lane_extract::lane_extractConfig& config, uint32_t level)
{
  cfg = config;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature2d");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pn.param("hough_threshold", hough_threshold, 25);
  pn.param("hough_min_line", hough_min_line, 150);
  pn.param("hough_max_line", hough_max_line, 30);

  ros::Subscriber sub_image = n.subscribe("/audibot/front_camera/image_raw", 1, recvImage);

  std::string filename;
  pn.getParam("filename", filename);
  templ = cv::imread(filename, 0);
//  cv::imshow("Template", templ);
//  cv::namedWindow("Template", CV_WINDOW_NORMAL);

  cv::namedWindow("Raw Image", CV_WINDOW_NORMAL);
  cv::namedWindow("Canny", CV_WINDOW_NORMAL);
  cv::namedWindow("Detection", CV_WINDOW_NORMAL);

  cv::startWindowThread();

  dynamic_reconfigure::Server<lane_extract::lane_extractConfig> srv;
  srv.setCallback(boost::bind(reconfig, _1, _2));

  ros::spin();
}
