#pragma once

// #define CVUI_IMPLEMENTATION
#include "nav_msgs/Odometry.h"
#include "robot_gui/cvui.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class CVUIROSOdomSubscriber {
public:
  CVUIROSOdomSubscriber(ros::NodeHandle& nh);
  nav_msgs::Odometry data;
  nav_msgs::Odometry run();

private:
  ros::Subscriber sub_;
  std::string topic_name;
  void msgCallback(const nav_msgs::Odometry::ConstPtr &msg);
//   const std::string WINDOW_NAME = "CVUI ROS SIMPLE SUBSCRIBER";
};