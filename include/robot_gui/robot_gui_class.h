#pragma once

#define CVUI_IMPLEMENTATION
#include <string>
#include "robot_gui/cvui.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <std_srvs/Trigger.h>

class RobotGui {
public:
  RobotGui();
  void run();

private:
  ros::Publisher twist_pub_;
  ros::Subscriber info_sub_;
  robotinfo_msgs::RobotInfo10Fields info_data_;
  std::string info_topic_name = "/robot_info";
  geometry_msgs::Twist twist_msg;
  std::string twist_topic_name = "/cmd_vel";
  float linear_velocity_step = 0.5;
  float angular_velocity_step = 0.5;
  ros::Subscriber odom_sub_;
  nav_msgs::Odometry odom_data_;
  std::string odom_topic_name = "/odom";
  ros::ServiceClient service_client;
  std_srvs::Trigger srv_req;
  std::string service_name = "/get_distance";
  std::string last_service_call_msg;
  int service_call_counter = 0;
  void odomCallback(const nav_msgs::Odometry::ConstPtr &omsg);
  void infoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &imsg);
  const std::string WINDOW_NAME = "CVUI ROBOT GUI";
};