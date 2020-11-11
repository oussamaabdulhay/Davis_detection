#include "EventPlotting.hpp"

const double EventPlotting::vertical_slope_threshold = 60. * (M_PI / 180);
ros::Publisher EventPlotting::pub;
cv::Point2f EventPlotting::c1;
cv::Point2f EventPlotting::c2; 
cv::Point2f EventPlotting::center;
std_msgs::Float32 EventPlotting::msg1;

EventPlotting::EventPlotting(ros::NodeHandle &t_nh)
{

    nh_ = t_nh;
    sub = nh_.subscribe("/dvs/events", 1, &EventPlotting::Events);
    pub = nh_.advertise<std_msgs::Float32>("chatter", 1000);
    cv::namedWindow(OPENCV_WINDOW);
}

EventPlotting::~EventPlotting()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void EventPlotting::Events(dvs_msgs::EventArray msg)
{
    cv::Mat EventsWindow = cv::Mat::zeros(260, 346, CV_8UC1);
    cv::Mat cdstP = cv::Mat::zeros(260, 346, CV_8UC1);

    for (int i = 0; i < msg.events.size(); i++)
    {
        cv::circle(EventsWindow, cv::Point(msg.events[i].x, msg.events[i].y), 0.5, cv::Scalar(255, 0, 0), -1, 8);
    }

    std::vector<cv::Vec4i> linesP;
    HoughLinesP(EventsWindow, linesP, 1, CV_PI / 180, 50, 50, 10);
    if (linesP.size() > 1)
    {
        for (size_t i = 0; i < linesP.size(); i++)
        {
            cv::Vec4i l = linesP[i];

            if (fabs(l[0] - l[2]) > 0)
            {
                double angle = fabs(atan((double)(l[3] - l[1]) / (double)(l[2] - l[0])));

                if (angle > vertical_slope_threshold)
                {
                    cv::line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
                }
            }

            else
            {
                cv::line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
            }
        }
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        findContours(cdstP, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        std::vector<cv::Moments> mu(contours.size());
        if (contours.size() > 1)
        {
            mu[0] = moments(contours[0], false);
            c1.x = mu[0].m10 / mu[0].m00;
            c1.y = mu[0].m01 / mu[0].m00;
            mu[1] = moments(contours[1], false);
            c2.x = mu[1].m10 / mu[1].m00;
            c2.y = mu[1].m01 / mu[1].m00;
            center.x = (c2.x + c1.x) / 2;
            center.y = (c1.y + c2.y) / 2;
        }
        msg1.data = center.x;
        pub.publish(msg1);
        cv::circle(cdstP, center, 10, cv::Scalar(255, 0, 0), -1, 8);
    }
    else
    {
        msg1.data = center.x;
        pub.publish(msg1);
        cv::circle(cdstP, center, 10, cv::Scalar(255, 0, 0), -1, 8);
    }

    cv::imshow("EventsWindow", EventsWindow);
    imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);

    cv::waitKey(1);
}
