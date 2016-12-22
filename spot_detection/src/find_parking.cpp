/*
 * This node processes back camera image and provides feedback to parking control (Production)
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int hough_threshold, hough_min_line, hough_max_line;
int h_center, h_width;
ros::Publisher pub_back;

void recvBackImage(const sensor_msgs::Image::ConstPtr msg)
{
  // Get the image as Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat raw_back_img = cv_ptr->image;
//  cv::imshow("Back Raw Image", raw_back_img);

  // Convert to HSV
  cv::Mat raw_hsv_back_img;
  cv::cvtColor(raw_back_img, raw_hsv_back_img, CV_BGR2HSV);

  // Split image into HSV channels
  std::vector<cv::Mat> split_back_img;
  cv::split(raw_hsv_back_img, split_back_img);

  // Threshold Hue
  cv::Mat t1, t2;
  cv::threshold(split_back_img[0], t1, h_center - h_width, 255, CV_THRESH_BINARY);
  cv::threshold(split_back_img[0], t2, h_center + h_width, 255, CV_THRESH_BINARY_INV);

  cv::Mat h_back_thres;
  cv::bitwise_and(t1, t2, h_back_thres);

  // Erode Line and Dilate
  cv::Mat erode;
  cv::Mat dilate;
  cv::erode(h_back_thres, erode, cv::Mat::ones(3, 3, CV_8U));
  cv::dilate(erode, dilate, cv::Mat::ones(3, 3, CV_8U));

  // Canny Edge Detection
  cv::Mat canny_img_orig, canny_img, canny_img_c;
  cv::Canny(dilate, canny_img_orig, 50, 200, 3);
  cv::dilate(canny_img_orig, canny_img, cv::Mat::ones(5, 5, CV_8U));
  cv::imshow("Canny_back", canny_img);

  //-- Hough Check
  cv::Mat back_img;
  back_img = raw_back_img.clone();
  std::vector<cv::Vec2f> lines;
  cv::HoughLines(canny_img, lines, 1, CV_PI/180, 250, 0, 0 );
  for( int i = 0; i < lines.size(); i++ )
  {
      float rho = lines[i][0], theta = lines[i][1];
      cv::Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      cv::line( back_img, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
  }
  cv::imshow("Back Output", back_img);

  // Delete Similar angled line
  for (int i=0; i<lines.size(); i++)
  {
    for (int j=0; j<lines.size(); j++)
    {
      if (fabs(lines[i][1] - lines[j][1]) <= 0.2)
        lines.erase(lines.begin()+j);
    }
  }

  std_msgs::Int16 back_status;
  back_status.data = 0; // None
  // Status of Back Line
  for (int i=0; i<lines.size(); i++){
    if (fabs(lines[i][1]) > 1.5 && fabs(lines[i][1] < 1.6)){
      if (lines[i][0] > 200.0){
        back_status.data = 1; // Done
        break;
      }else {
        back_status.data = 0;
        break;
      }
    }else{
       if (lines[i][0] >= 0)
         back_status.data = 3; // right
       else
         back_status.data = 2; // left
    }


//    ROS_INFO("Angles %f, status %d", (double)lines[i][1], back_status.data);
//      ROS_INFO("Angles %f, distance %f", (double)lines[i][1], (double)lines[i][0]);
  }
  pub_back.publish(back_status);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_parking");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pn.param("hough_threshold", hough_threshold, 25);
  pn.param("hough_min_line", hough_min_line, 150);
  pn.param("hough_max_line", hough_max_line, 30);
  pn.param("h_center", h_center, 30);
  pn.param("h_width", h_width, 10);

  ros::Subscriber sub_back_img = n.subscribe("/audibot/back_camera/image_raw", 1, recvBackImage);
  pub_back = n.advertise<std_msgs::Int16>("/back_camera/status", 1);

//  cv::namedWindow("Back Raw Image", CV_WINDOW_NORMAL);
  cv::namedWindow("Back Output", CV_WINDOW_NORMAL);
  cv::namedWindow("Canny_back", CV_WINDOW_NORMAL);

  cv::startWindowThread();

  ros::spin();
}
