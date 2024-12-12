/*
This is a ROS node that subscribes to a message of type nav_msgs/Odometry
and uses the incoming data to calculate the distance traveled which is
stored on a member variable. The node also has a service server which sends
a response with the distance traveled each time a request of type
std_srvs/Trigger.h is received.
*/
#pragma once
#include "nav_msgs/Odometry.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "std_srvs/Trigger.h"
#include <iomanip>
#include <sstream>

class DistanceTracker {
public:
//   static 
  DistanceTracker(ros::NodeHandle nh):nh_(nh) {
    distance_ = 0.0;
    odom_sub_ = nh_.subscribe("odom", 1, &DistanceTracker::odomCallback, this);
    get_distance_service_ = nh_.advertiseService(
        "get_distance", &DistanceTracker::getDistanceServiceCallback, this);
    ROS_INFO("Distance tracker node initialized. Use: rosservice call "
             "/get_distance \"{}\" ");
    reset_distance_service_ = nh_.advertiseService(
        "reset_distance", &DistanceTracker::resetDistanceServiceCallback, this);
    ROS_INFO("Distance tracker node initialized. Use: rosservice call "
             "/reset_distance \"{}\" ");
  }
  
private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::ServiceServer get_distance_service_;
  ros::ServiceServer reset_distance_service_;
  double distance_;
  std::string formatFloatToString(float f) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << f;
    return out.str();
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // calculate distance traveled using Euclidean distance formula
    double dx = msg->pose.pose.position.x - prev_pose_.position.x;
    double dy = msg->pose.pose.position.y - prev_pose_.position.y;
    double dz = msg->pose.pose.position.z - prev_pose_.position.z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);

    // add distance traveled to total distance
    distance_ += distance;

    // update previous pose
    prev_pose_ = msg->pose.pose;
  }

  bool getDistanceServiceCallback(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res) {
    ROS_INFO("service was called");                              
    res.success = true;
    // Respond with distance traveled in meters
    res.message = formatFloatToString(distance_);
    return true;
  }

  bool resetDistanceServiceCallback(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res) {
    ROS_INFO("reset service was called"); 
    distance_ = 0;                             
    res.success = true;
    // Respond with distance traveled in meters
    res.message = formatFloatToString(distance_);
    return true;
  }  
  geometry_msgs::Pose prev_pose_;
};


