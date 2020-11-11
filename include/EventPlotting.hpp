#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <dvs_msgs/EventArray.h>
#include <cmath>
#include "std_msgs/Float32.h"

class EventPlotting
{
  public:
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  static ros::Publisher pub;
  static cv::Point2f c1;
  static cv::Point2f c2;
  static cv::Point2f center;
  static std_msgs::Float32 msg1;

  const std::string OPENCV_WINDOW = "Image window";
  static const double vertical_slope_threshold;

  EventPlotting(ros::NodeHandle&);
  ~EventPlotting();

  static void Events(dvs_msgs::EventArray msg);
};