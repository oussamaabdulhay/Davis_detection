#include "Circle_detector.hpp"



Circle_detector::Circle_detector(ros::NodeHandle &main_nodehandle)
    : it_(nh_)
{
    nh_=main_nodehandle;
    image_sub_ = it_.subscribe("/EventsImage", 1000,&Circle_detector::imageCb, this);
    puby = nh_.advertise<std_msgs::Float32>("camera_provider_y", 1000);
    pubx = nh_.advertise<std_msgs::Float32>("camera_provider_x", 1000);
    //cv::namedWindow(OPENCV_WINDOW);

    params.filterByArea = true;
    params.minArea = 2500;
    params.maxArea = 3500;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.1;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.2;

    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.2;

    threshold = 10;
}

Circle_detector::~Circle_detector()
{
    //cv::destroyWindow(OPENCV_WINDOW);
}

void Circle_detector::imageCb(const sensor_msgs::ImageConstPtr &msg)

{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception &e)
    {

        ROS_ERROR("cv_bridge exception: %s", e.what());

        return;
    }
    cv::Mat imgOriginal = cv_ptr->image;

    cv::Mat blurred,im_with_keypoints;
    cv::GaussianBlur(imgOriginal, blurred, cv::Size(19, 19), 0, 0);
    cv::bitwise_not(blurred, blurred);
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect(blurred, keypoints);
    cv::drawKeypoints(blurred, keypoints, im_with_keypoints, cv::Scalar(128, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::cout<<keypoints.size()<<std::endl;

    std_msgs::Float32 msg_y;
    std_msgs::Float32 msg_x;

  if (keypoints.size() == 0)
  {
    std::cout << "EMPTY KEYPOINTS\n";
  }

  else
  {
    float std_dev;
    temp.push_back(keypoints[0].pt);

    if (temp.size() == 3)
    {
      std_dev = filter->getStdDev(temp);
      //std::cout << std_dev << std::endl;
      if (std_dev < threshold)
      {
         _c_.x = temp.back().x;
         _c_.y = temp.back().y;
         msg_y.data = _c_.y-130;
         msg_x.data = _c_.x-173;
         obj_pos.y = _c_.y-130;
         obj_pos.x = _c_.x-173;
         puby.publish(msg_y);
         pubx.publish(msg_x);
         pixel_location.setVector2DMsg(obj_pos);
        this->emitMsgUnicastDefault((DataMessage *)&pixel_location);
         

      }

       else
       {
         std::cout << "standard dev too high\n";
         _c_ = filter->getMedian(temp, _c_);
         msg_y.data = _c_.y-130;
         msg_x.data = _c_.x-173;
          obj_pos.y = _c_.y-130;
         obj_pos.x = _c_.x-173;
         puby.publish(msg_y);
         pubx.publish(msg_x);
         point_of_interest = sqrt((pow(_c_.x, 2)) + (_c_.y, 2));
         pixel_location.setVector2DMsg(obj_pos);
        this->emitMsgUnicastDefault((DataMessage *)&pixel_location);
       }
    temp.erase(temp.begin());
    }
  }

    // cv::imshow("Original", imgOriginal);             //show the original image
    // cv::imshow("im_with_keypoints", im_with_keypoints); 
    // cv::waitKey(1);
}
