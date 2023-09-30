#include "robot_gui/robot_gui_class.h"
#include "ros/init.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_node");
  RobotGui gui;
  gui.run();
  return 0;
}