#pragma once
#include "robot_gui/odometry_msgs.h"
#include "robot_gui/service_tracker.h"
#include "robot_gui/trigger_msg_srv.h"
#include <mutex> 
class GUI{
public:
GUI(ros::NodeHandle& nh, CVUIROSOdomSubscriber* odom_sub);
void run();
void info_callback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
std::mutex mutex;
protected:
std::string info1;
std::string info2;
std::string info3;
std::string info4;
std::string info5;
std::string info6;
std::string info7;
std::string info8;

private:
CVUIROSOdomSubscriber* o_sub;
ros::Publisher pub_cmd;
ros::Subscriber sub_info;
geometry_msgs::Twist twist;
std::string topic_name;
float linear_change = 0.1;
float angular_change = 0.1;
float x;
float z;
const std::string WINDOW_NAME = "GUI ROS TELEOP";
ros::NodeHandle nh_;
std::string last_msg_;

};

