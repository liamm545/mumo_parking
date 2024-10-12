/*
 * erp42_controller_node.cpp
 *
 *  Created on: Oct 12, 2024
 *      Author: yeong
 */

#include <ros/ros.h>
#include "car_demo/Erp42ControllerRos.hpp"

using namespace car_demo;
int main(int argc, char** argv) {
  ros::init(argc, argv, "erp42_controller_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
  Erp42ControllerRos controller(nh);
  const double frequency = 50.0;
  controller.initialize(1 / frequency);

  sleep(2.0);
  ros::Rate r(frequency);
  while (ros::ok()) {
    controller.advance();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
