/*
This is an extremely simple demo application to showcase the
basic structure, features and use of cvui.

Code licensed under the MIT license, check LICENSE file.
*/
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <string>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include "robot_gui/odometry_msgs.h"
#include "robot_gui/service_tracker.h"
#include "robot_gui/trigger_msg_srv.h"
#include "robot_gui/gui.h"
#include <thread>

// One (and only one) of your C++ files must define CVUI_IMPLEMENTATION
// before the inclusion of cvui.h to ensure its implementaiton is compiled.
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"



CVUIROSOdomSubscriber::CVUIROSOdomSubscriber(ros::NodeHandle& nh){
topic_name = "odom";
sub_ = nh.subscribe(topic_name, 10, &CVUIROSOdomSubscriber::msgCallback, this);
}
void CVUIROSOdomSubscriber::msgCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  data = *msg;
//   ROS_INFO("Position x,y,z: [%0.2f, %0.2f, %0.2f]", msg->pose.pose.position.x,
//             msg->pose.pose.position.y, msg->pose.pose.position.z);
}

nav_msgs::Odometry CVUIROSOdomSubscriber::run(){
  
  return data;
}


GUI::GUI(ros::NodeHandle& nh, CVUIROSOdomSubscriber* odom_sub):nh_(nh),o_sub(odom_sub){
topic_name = "cmd_vel";
pub_cmd = nh_.advertise <geometry_msgs::Twist>(topic_name,10);
sub_info = nh_.subscribe("robot_info", 1, &GUI::info_callback, this);

}
void GUI::info_callback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg){
    info1 = msg->data_field_01;
    info2 = msg->data_field_02;
    info3 = msg->data_field_03;
    info4 = msg->data_field_04;
    info5 = msg->data_field_05;
    info6 = msg->data_field_06;
    info7 = msg->data_field_07;
    info8 = msg->data_field_08; 
    
    // ROS_INFO("info test");
    // ROS_INFO("%s",info1.c_str());
    // ROS_INFO(info2);
    // ROS_INFO(info3);
    // ROS_INFO(info4);

}
void GUI::run(){
	cv::Mat frame = cv::Mat(800, 600, CV_8UC3);
	// int count = 0;

	// Init cvui and tell it to create a OpenCV window, i.e. cv::namedWindow(WINDOW_NAME).
	cv::namedWindow(WINDOW_NAME);
    cvui::init(WINDOW_NAME);
    DistanceTracker ds(nh_); 
    CVUIROSTriggerMsgServiceClient sc_get(nh_ ,"get_distance");
    CVUIROSTriggerMsgServiceClient sc_reset(nh_ ,"reset_distance");

    while (ros::ok()) {
        nav_msgs::Odometry odom = o_sub->run();
        float pos_x = odom.pose.pose.position.x;
        float pos_y = odom.pose.pose.position.y;
        float pos_z = odom.pose.pose.position.z;
		// Fill the frame with a nice color
		frame = cv::Scalar(49, 52, 49);

        // top window
        cvui::window(frame, 50, 10, 300, 145, "Info:");
        cvui::printf(frame, 50, 35, 0.4, 0xff0000, "info1: %s ", info1.c_str());        
        cvui::printf(frame, 50, 50, 0.4, 0xff0000, "Info2: %s ", info2.c_str());
        cvui::printf(frame, 50, 65, 0.4, 0xff0000, "info3: %s ", info3.c_str());        
        cvui::printf(frame, 50, 80, 0.4, 0xff0000, "Info4: %s ", info4.c_str());
        cvui::printf(frame, 50, 95, 0.4, 0xff0000, "info5: %s ", info5.c_str());        
        cvui::printf(frame, 50, 110, 0.4, 0xff0000, "Info6: %s ", info6.c_str());
        cvui::printf(frame, 50, 125, 0.4, 0xff0000, "info7: %s ", info7.c_str());        
        cvui::printf(frame, 50, 140, 0.4, 0xff0000, "Info8: %s ", info8.c_str());    

        cvui::window(frame, 50, 400, 120, 40, "Linear velocity:");
    // Show the current velocity inside the window
        cvui::printf(frame, 50, 425, 0.4, 0xff0000,"%f", x);
        
        cvui::window(frame, 400, 400, 120, 40, "Angular velocity:");
    // Show the current velocity inside the window
        cvui::printf(frame, 400, 425, 0.4, 0xff0000,"%f", z);
        
    // robot position based on odometry
        cvui::printf(frame, 50, 450, 0.8, 0Xffffff, "robot position based on odometry");

        cvui::window(frame,20, 470, 150, 150, "x");
        cvui::printf(frame,25, 575,"%f", pos_x,1);
        cvui::window(frame,200, 470, 150, 150, "y");
        cvui::printf(frame,205, 575,"%f", pos_y,1);        
        cvui::window(frame,380, 470, 150, 150, "z");
        cvui::printf(frame,385, 575,"%f", pos_z);
		
        cvui::printf(frame, 20, 620, 0.6, 0Xffffff, "Distance traveled");
        
        cvui::window(frame, 200, 630, 300, 150, "Distance in meters");
        if (not last_msg_.empty()) {
            cvui::printf(frame, 210, 680, 0.4, 0xff0000, "%s",last_msg_.c_str());}
        // Buttons will return true if they were clicked, which makes
		// handling clicks a breeze.
		if (cvui::button(frame, 190, 170, "Forward",1)) {
			// The button was clicked, drive-forward.
            twist.linear.x = twist.linear.x + linear_change;
            x = twist.linear.x;
		}

		if (cvui::button(frame,220, 250, "Stop",1)) {
			// The button was clicked, stop.
            twist.linear.x = 0;
            twist.angular.z = 0;
            x = twist.linear.x;
            z = twist.angular.z;

		}

		if (cvui::button(frame, 190, 330, "Backward",1)) {
			// The button was clicked, drive-backward.
            twist.linear.x = twist.linear.x - linear_change;
            x = twist.linear.x;

        }

		if (cvui::button(frame,80, 250, "Left",1)) {
			// The button was clicked, turn left.
            twist.angular.z = twist.angular.z + angular_change;
            z = twist.angular.z;
        }

		if (cvui::button(frame, 370, 250, "Right",1)) {
			// The button was clicked, turn right.
            twist.angular.z = twist.angular.z - angular_change;
		   	z = twist.angular.z;
        }                

		if (cvui::button(frame, 20, 650, "call",1)) {
			// The button was clicked, measure distance.
            last_msg_ = sc_get.run();
       
		}
		if (cvui::button(frame, 20, 720, "reset",1)) {
			// The button was clicked, measure distance.
            // ds.reset_distabce();
            last_msg_ = sc_reset.run();
       
		}

		// Sometimes you want to show text that is not that simple, e.g. strings + numbers.
		// You can use cvui::printf for that. It accepts a variable number of parameter, pretty
		// much like printf does.
		// Let's show how many times the button has been clicked.
		// cvui::printf(frame, 250, 90, 0.4, 0xff0000, "Button click count: %d", count);

		// Update cvui stuff and show everything on the screen

        pub_cmd.publish(twist);		
        cvui::update();
        cvui::imshow(WINDOW_NAME, frame);
        ros::spinOnce();

		// Check if ESC key was pressed
		if (cv::waitKey(20) == 27) {
			break;
		}
	}}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"gui_node");
  ros::NodeHandle nh;
  CVUIROSOdomSubscriber odom_sub(nh);

  GUI gui(nh, &odom_sub);
  std::thread t1(&GUI::run, &gui);
  ros::spin();
  t1.join();
  
	return 0;
}