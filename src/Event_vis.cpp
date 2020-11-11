#include "Event_vis.hpp"

Event_vis::Event_vis(ros::NodeHandle &t_nh):it_(nh_)
{

    nh_ = t_nh;
    sub = nh_.subscribe("/dvs/events", 1, &Event_vis::Events,this);
    pub=it_.advertise("/EventsImage",1);
    //cv::namedWindow(OPENCV_WINDOW);
}

Event_vis::~Event_vis()
{
    //cv::destroyWindow(OPENCV_WINDOW);
}

void Event_vis::Events(dvs_msgs::EventArray msg)
{
    cv::Mat EventsWindow = cv::Mat::zeros(260, 346, CV_8UC1);
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;

    for (int i = 0; i < msg.events.size(); i++)
    {
        cv::circle(EventsWindow, cv::Point(msg.events[i].x, msg.events[i].y), 2.0, cv::Scalar(255, 0, 0), -1, 8);
    }

    std_msgs::Header header;
    header.seq = 1;
    header.stamp= ros::Time::now();
    img_bridge=cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, EventsWindow);
    img_bridge.toImageMsg(img_msg);
    pub.publish(img_msg);
    
    // cv::imshow("EventsWindow", EventsWindow);
    // cv::waitKey(1);
}
