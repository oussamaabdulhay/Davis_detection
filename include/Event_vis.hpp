#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <dvs_msgs/EventArray.h>
#include <cmath>
#include "std_msgs/Float32.h"
#include <ros/ros.h>

class Event_vis
{
  public:
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub;


  const std::string OPENCV_WINDOW = "Image window";

  Event_vis(ros::NodeHandle &);
  ~Event_vis();

  void Events(dvs_msgs::EventArray msg);
};