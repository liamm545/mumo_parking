/*
 * PriusControllerRos.hpp
 *
 *  Created on: Apr 6, 2020
 *      Author: jelavice
 */

#pragma once
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "erp42_msgs/DriveCmd.h"
#include "erp42_msgs/ModeCmd.h"
#include "car_demo/PIDController.hpp"
#include "prius_msgs/PriusControl.hpp"
#include "se2_navigation_msgs/Path.hpp"
#include "se2_navigation_msgs/RequestCurrentStateSrv.h"
#include "se2_navigation_msgs/SendControllerCommandSrv.h"

namespace pure_pursuit {
class PathTracker;
class Path;
}  // namespace pure_pursuit

namespace car_demo {

class Erp42ControllerRos {
  using CurrentStateService = se2_navigation_msgs::RequestCurrentStateSrv;
  using ControllerCommandService = se2_navigation_msgs::SendControllerCommandSrv;

 public:
  Erp42ControllerRos(ros::NodeHandlePtr nh);
  virtual ~Erp42ControllerRos(); /* = default(), defined in cpp file*/
  void initialize(double dt);
  void advance();

 private:
  void update();
  void translateCommands(double longitudinalSpeed, double steeringAngle, prius_msgs::PriusControl* ctrl);
  void translateGear(double longitudinalSpeed, prius_msgs::PriusControl* ctrl) const;
  void translateVelocity(double desiredVelocityMagnitude, prius_msgs::PriusControl* ctrl);
  void createPathTrackerAndLoadParameters();
  void loadPIDParameters();
  void publishControl(const prius_msgs::PriusControl& ctrl) const;
  void initRos();
  void erpStateCallback(const nav_msgs::Odometry& odometry);
  void pathCallback(const se2_navigation_msgs::PathMsg& pathMsg);
  void stopTracking();
  bool currentStateRequestService(CurrentStateService::Request& req, CurrentStateService::Response& res);
  bool controllerCommandService(ControllerCommandService::Request& req, ControllerCommandService::Response& res);

  void processStartTrackingCommand();
  void processAbortTrackingCommand();

  // void publishWheelTorque(double torque);
  // void publishSteeringAngle(double angle);

  ros::NodeHandlePtr nh_;
  double dt_ = 0.01;
  ros::Publisher drive_pub_;
  ros::Publisher mode_pub_;
  ros::Publisher erpControlPub_;
  ros::Publisher right_front_wheel_pub_;
  ros::Publisher right_back_wheel_pub_;  
  ros::Publisher left_front_wheel_pub_;   
  ros::Publisher left_back_wheel_pub_;    
  ros::Publisher right_steer_wheel_pub_;  
  ros::Publisher left_steer_wheel_pub_;   
  ros::Subscriber erpStateSub_;
  ros::Subscriber pathSub_;
  ros::ServiceServer erpCurrentStateService_;
  ros::ServiceServer controllerCommandService_;
  nav_msgs::Odometry erpState_;

  /*state machine variables*/
  bool planReceived_ = false;
  bool currentlyExecutingPlan_ = false;
  bool receivedStartTrackingCommand_ = false;
  bool doneFollowingPrev_ = false;
  bool publishTrackingStatus_ = false;

  prius_msgs::PriusControl priusControl_;
  erp42_msgs::DriveCmd drive_cmd_;
  erp42_msgs::ModeCmd mode_cmd_;
  std::unique_ptr<pure_pursuit::PathTracker> pathTracker_;
  PIDController pidController_;
  PIDController pidControllerLeftFront_;
  PIDController pidControllerLeftBack_;
  PIDController pidControllerRightFront_;
  PIDController pidControllerRightBack_;
  PIDController pidControllerRightSteer_;
  PIDController pidControllerLeftSteer_;
  se2_navigation_msgs::Path currentPath_;
};

double longitudinalVelocity(const nav_msgs::Odometry& odom);
void convert(se2_navigation_msgs::Path& path, pure_pursuit::Path* out);
} /* namespace car_demo*/
