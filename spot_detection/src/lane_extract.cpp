/*
 * This node processes front camera image and provides detects the parking spot (production)
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <spot_detection/lane_extractConfig.h>

lane_extract::lane_extractConfig cfg;
int hough_threshold, hough_min_line, hough_max_line;
cv::Mat templ;
ros::Publisher pub_point;
ros::Publisher pub_img;
ros::Publisher pub_park;

void recvImage(const sensor_msgs::Image::ConstPtr msg)
{
  //-- Get the image as Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat raw_img = cv_ptr->image;
  cv::imshow("Raw Image", raw_img);

  //-- Convert to HSV
  cv::Mat raw_hsv_img;
  cv::cvtColor(raw_img, raw_hsv_img, CV_BGR2HSV);

  //-- Split image into HSV channels
  std::vector<cv::Mat> split_img;
  cv::split(raw_hsv_img, split_img);

//  cv::imshow("H", split_img[0]);
//  cv::imshow("S", split_img[1]);
//  cv::imshow("V", split_img[2]);

  //-- Threshold Hue
  cv::Mat t1, t2;
  cv::threshold(split_img[0], t1, cfg.h_center - cfg.h_width, 255, CV_THRESH_BINARY);
  cv::threshold(split_img[0], t2, cfg.h_center + cfg.h_width, 255, CV_THRESH_BINARY_INV);

  cv::Mat h_thres;
  cv::bitwise_and(t1, t2, h_thres);

  //-- Threshold Saturation
//  cv::Mat s_thres;
//  cv::threshold(split_img[1], s_thres, cfg.v_thres, 255, CV_THRESH_BINARY);

  //-- Threshold Value
//  cv::Mat v_thres;
//  cv::threshold(split_img[2], v_thres, cfg.v_thres, 255, CV_THRESH_BINARY);

  //-- Display threshold images
//  cv::imshow("H_thres", h_thres);
//  cv::imshow("S_thres", s_thres);
//  cv::imshow("V_thres", v_thres);

  //-- Erode Line and Dilate
  cv::Mat erode;
  cv::Mat dilate;
  cv::erode(h_thres, erode, cv::Mat::ones(3, 3, CV_8U));
  cv::dilate(erode, dilate, cv::Mat::ones(3, 3, CV_8U));

  //-- Canny Edge Detection
  cv::Mat canny_img_orig, canny_img, canny_img_c;
  cv::Canny(dilate, canny_img_orig, 50, 200, 3);
  cv::dilate(canny_img_orig, canny_img, cv::Mat::ones(5, 5, CV_8U));
//  cv::imshow("Canny", canny_img);
  cv::Mat canny_pub;
  canny_pub = canny_img.clone();


  /*------------------------- Template Matching -------------------------------------*/
  cv::Mat result;
  cv::Mat img_display;
  canny_img.copyTo(img_display);

  //-- Create the result matrix
  int result_cols = canny_img.cols - templ.cols + 1;
  int result_rows = canny_img.rows - templ.rows + 1;
  result.create(result_rows, result_cols, CV_32FC1);

  //-- Do the matching and normalize
  cv::matchTemplate(canny_img, templ, result, cfg.method);
  cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

  //-- Locating Best match
  double minVal, maxVal;
  cv::Point minLoc, maxLoc, matchLoc;
  cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

  // -------------
  if( cfg.method  == CV_TM_SQDIFF || cfg.method == CV_TM_SQDIFF_NORMED )
      { matchLoc = minLoc; }
    else
      { matchLoc = maxLoc; }

  //-- Show image
  cv::rectangle( img_display, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar(255,255,255), 2, 8, 0 );
  cv::rectangle( result, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar(0,0,255), 2, 8, 0 );

  //-- Display Point of reference
  int x_1 = (int)(matchLoc.x + (templ.cols / 2));
  int y_1 = (int)(matchLoc.y + 2 * (templ.rows / 3));
  int y_2 = (int)(matchLoc.y + (templ.rows / 6));

  cv::circle( img_display, cv::Point(x_1, y_1), 1, cv::Scalar(255,255,255), 5, 8, 0);
  cv::circle( img_display, cv::Point(x_1, y_2), 1, cv::Scalar(255,255,255), 5, 8, 0);

  cv::imshow( "image_window", img_display );
//  cv::imshow( "result_window", result );

  //-- Extract image inside rectangle
  cv::Mat inside_img, inside_orig_img;
  inside_img = canny_img(cv::Rect(matchLoc.x, matchLoc.y, templ.cols, templ.rows));
  inside_orig_img = raw_img(cv::Rect(matchLoc.x, matchLoc.y, templ.cols, templ.rows));
//  imshow("Inside", inside_img);
//  ROS_INFO("x:%d, y:%d, w:%d, h:%d", matchLoc.x, matchLoc.y, templ.cols, templ.rows);

  /*---------------------- Corner Detection ---------------------*/
//  cv::vector<cv::Point2f> corners;
//  int maxCorner = 2;
//  double qualityLevel = 0.01;
//  double minDistance = 10;
//  int blockSize = 3;
//  bool useHarrisDetector = false;
//  double k = 0.04;
//  // Copy canny output img
//  cv::Mat copy;
//  copy = raw_img.clone();
//  // Apply corner detection
//  cv::goodFeaturesToTrack(canny_img, corners, maxCorner, qualityLevel, minDistance, cv::Mat(), blockSize, useHarrisDetector, k);
//
//  /// Draw corners detected
//   int r = 4;
//   for( int i = 0; i < corners.size(); i++ )
//      { cv::circle( copy, corners[i], r, cv::Scalar(0,0,0), -1, 8, 0 ); }
//
//   /// Show what you got
////   cv::imshow( "Corners", copy );

  /*---------------------------- Hough Line P Detection ----------------------------------*/
  std::vector<cv::Vec4i> line_segments;
  cv::HoughLinesP(inside_img, line_segments, 1, 0.05, cfg.vote, cfg.min_length, cfg.max_gap);
                                         // 1, 0.05, 250,      250,             10
//  for (int i=0; i<line_segments.size(); i++){
//    cv::line(inside_orig_img, cv::Point(line_segments[i][0], line_segments[i][1]),
//             cv::Point(line_segments[i][2], line_segments[i][3]), cv::Scalar(0, 0, 255));
//  }
  int x = line_segments.size();
//  ROS_INFO("Original Lines Detected: %d", x);
//  cv::imshow("Hough Output", inside_orig_img);

  //-- Delete similar angled line
  for(int i=0; i<line_segments.size();i++)
  {
    double i_angle = atan2((line_segments[i][4]-line_segments[i][2]),(line_segments[i][1]-line_segments[i][0]));

    for(int j=i; j<line_segments.size();j++)
    {
      double j_angle = atan2((line_segments[j][4]-line_segments[j][2]),(line_segments[j][1]-line_segments[j][0]));
      if (fabs(i_angle - j_angle) < 0.7){
        line_segments.erase(line_segments.begin()+j);
      }
    }
  }
  x = line_segments.size();
//  ROS_INFO("Filter Lines Detected: %d", x);

  for (int i=0; i<line_segments.size(); i++){
    cv::line(inside_orig_img, cv::Point(line_segments[i][0], line_segments[i][1]),
             cv::Point(line_segments[i][2], line_segments[i][3]), cv::Scalar(0, 0, 255));
  }
  cv::imshow("Hough Output", inside_orig_img);

  //-- Publish Parking Detection Status
  std_msgs::Bool parking_detected;
  if (line_segments.size() >= 3)
  {
    parking_detected.data = true;
//    ROS_INFO("Parking Spot DETECTED!!");
  } else {
    parking_detected.data = false;
//    ROS_INFO("Searching for Parking...");
  }
  pub_park.publish(parking_detected);

  //-- Send two points out
  geometry_msgs::PoseArray points;

  points.header.frame_id = "/front_camera";
  points.header.stamp = msg->header.stamp;
  points.poses.resize(2);

  points.poses[0].position.x = x_1;
  points.poses[0].position.y = y_1;
  points.poses[0].position.z = 0;
  points.poses[0].orientation.w = 1;

  points.poses[1].position.x = x_1;
  points.poses[1].position.y = y_2;
  points.poses[1].position.z = 0;
  points.poses[1].orientation.w = 1;

  pub_point.publish(points);

  cv::circle( canny_pub, cv::Point(x_1, y_1), 1, cv::Scalar(255,255,255), 5, 8, 0);
  cv::circle( canny_pub, cv::Point(x_1, y_2), 1, cv::Scalar(255,255,255), 5, 8, 0);

  //-- Publish Canny Image
  cv_bridge::CvImage cv_image(cv_ptr->header, "mono8", canny_pub);
  pub_img.publish(cv_image.toImageMsg());

}

void reconfig(lane_extract::lane_extractConfig& config, uint32_t level)
{
  cfg = config;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_extract");
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

  cv::namedWindow( "image_window", CV_WINDOW_NORMAL );
//  cv::namedWindow( "result_window", CV_WINDOW_NORMAL );
//  cv::namedWindow( "Corners", CV_WINDOW_NORMAL );
  cv::namedWindow("Raw Image", CV_WINDOW_NORMAL);

//  cv::namedWindow("H", CV_WINDOW_NORMAL);
//  cv::namedWindow("S", CV_WINDOW_NORMAL);
//  cv::namedWindow("V", CV_WINDOW_NORMAL);

//  cv::namedWindow("H_thres", CV_WINDOW_NORMAL);
//  cv::namedWindow("S_thres", CV_WINDOW_NORMAL);
//  cv::namedWindow("V_thres", CV_WINDOW_NORMAL);

//  cv::namedWindow("Canny", CV_WINDOW_NORMAL);
//  cv::namedWindow("Inside", CV_WINDOW_NORMAL);
  cv::namedWindow("Hough Output", CV_WINDOW_NORMAL);

  cv::startWindowThread();

  pub_point = n.advertise<geometry_msgs::PoseArray>("/front_camera/point", 1);

  pub_img = n.advertise<sensor_msgs::Image>("/front_camera/canny", 1);
  pub_park = n.advertise<std_msgs::Bool>("/parking_detected", 1);

  dynamic_reconfigure::Server<lane_extract::lane_extractConfig> srv;
  srv.setCallback(boost::bind(reconfig, _1, _2));

  ros::spin();
}
