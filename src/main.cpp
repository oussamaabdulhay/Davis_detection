#include "Event_vis.hpp"
#include "EventPlotting.hpp"
#include "Circle_detector.hpp"
#include <ros/ros.h>
#include "rayrotation_events.hpp"
#include <opencv2/core/types.hpp>
#include "common_srv/ROSUnit_Factory.hpp"
#include "ROSUnit_Optitrack.hpp"
#include <iostream>

int main(int argc, char** argv)
{


ros::init(argc, argv, "ball_detector_node");
ros::NodeHandle main_nodehandle;
ROSUnit_Factory ROSUnit_Factory_main{main_nodehandle};
ROSUnit_Optitrack* position_in_z=new ROSUnit_Optitrack(main_nodehandle);
//EventPlotting* detect=new EventPlotting(main_nodehandle);
Event_vis* visualisation=new Event_vis(main_nodehandle);
Circle_detector* detection=new Circle_detector(main_nodehandle);

  ROSUnit* rosunit_x_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/x");
  ROSUnit* rosunit_y_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/y");
  ROSUnit* rosunit_roll_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/roll");
  ROSUnit* rosunit_pitch_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/pitch");
  ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                  ROSUnit_msg_type::ROSUnit_Point,
                                                                  "/providers/yaw");
  ROSUnit* rosunit_camera = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
                                                                  ROSUnit_msg_type::ROSUnit_PointUint64,
                                                                  "/camera_provider");
  // ROSUnit* rosunit_angles = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher,
  //                                                                 ROSUnit_msg_type::ROSUnit_Point,
  //                                                                 "/camera_angles");
  // ROSUnit* rosunit_camera_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber,
  //                                                                 ROSUnit_msg_type::ROSUnit_Float,
  //                                                                 "/camera_provider");
  // ROSUnit* rosunit_camera_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber,
  //                                                                 ROSUnit_msg_type::ROSUnit_Float,
  //                                                                 "/camera_provider_x");
                                                                

  rayrotation_events* rotate = new rayrotation_events();

  rosunit_x_provider->setEmittingChannel((int)rayrotation_events::receiving_channels::ch_x);
  rosunit_y_provider->setEmittingChannel((int)rayrotation_events::receiving_channels::ch_y);
  position_in_z->setEmittingChannel((int)rayrotation_events::receiving_channels::ch_z);
  rosunit_roll_provider->setEmittingChannel((int)rayrotation_events::receiving_channels::ch_roll);
  rosunit_pitch_provider->setEmittingChannel((int)rayrotation_events::receiving_channels::ch_pitch);
  rosunit_yaw_provider->setEmittingChannel((int)rayrotation_events::receiving_channels::ch_yaw);
  detection->setEmittingChannel((int)rayrotation_events::receiving_channels::camera);
  // rosunit_camera_y->setEmittingChannel((int)rayrotation::receiving_channels::ch_camera_y);
  // rosunit_camera_x->setEmittingChannel((int)rayrotation::receiving_channels::ch_camera_x);


  rosunit_x_provider->addCallbackMsgReceiver((MsgReceiver*)rotate);
  rosunit_y_provider->addCallbackMsgReceiver((MsgReceiver*)rotate);
  position_in_z->addCallbackMsgReceiver((MsgReceiver*)rotate);
  rosunit_roll_provider->addCallbackMsgReceiver((MsgReceiver*)rotate);
  rosunit_pitch_provider->addCallbackMsgReceiver((MsgReceiver*)rotate);
  rosunit_yaw_provider->addCallbackMsgReceiver((MsgReceiver*)rotate);
  detection->addCallbackMsgReceiver((MsgReceiver*)rotate);
  rotate->addCallbackMsgReceiver((MsgReceiver*)rosunit_camera);
  //rotate->addCallbackMsgReceiver((MsgReceiver*)rosunit_angles);
  // rosunit_camera_x->addCallbackMsgReceiver((MsgReceiver*)rotate);
  // rosunit_camera_y->addCallbackMsgReceiver((MsgReceiver*)rotate);


  ros::Rate r(200);
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
