/*
 * se2_planner_node.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include <ros/ros.h>

#include "se2_planning_ros/loaders.hpp"

#include "se2_planning/GridMapStateValidator.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ompl/base/spaces/SE2StateSpace.h>

grid_map::GridMap gridMap;
bool isMapReceived = false;

std::shared_ptr<se2_planning::OmplReedsSheppPlanner> planner;

void gridMapCallback(const grid_map_msgs::GridMap& msg) {
  grid_map::GridMapRosConverter::fromMessage(msg, gridMap);
  isMapReceived = true;
  ROS_INFO("GridMap received successfully");

  if (!gridMap.exists("obstacle")) {
    ROS_WARN("Obstacle layer not found in the received grid map.");
    return;
  }

  const auto& obstacleLayer = gridMap["obstacle"];
  std::vector<grid_map::Position> obstaclePositions;
  // const auto& data = gridMap.get("obstacle");
  // std::cout << data << std::endl;
  for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    const double value = obstacleLayer(index(0), index(1));

    if (value >= 0.1) {
      grid_map::Position position;
      gridMap.getPosition(index, position);
      obstaclePositions.push_back(position);
    }
  }

  for (const auto& pos : obstaclePositions) {
    ROS_INFO_STREAM("Obstacle at: (" << pos.x() << ", " << pos.y() << ")");
  }

  se2_planning::RobotFootprint footprint = se2_planning::computeFootprint(1.0, 1.0, 0.5, 0.5);

  std::string obstacleLayerStr = "obstacle";

  const auto& data = gridMap.get("obstacle");
  std::cout << data << std::endl;

  auto stateValidator = se2_planning::createGridMapStateValidator(gridMap, footprint, obstacleLayerStr);
  planner->setStateValidator(std::move(stateValidator));
}

int main(int argc, char** argv) {
  using namespace se2_planning;

  ros::init(argc, argv, "se2_planner_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  std::string filename = nh->param<std::string>("/ompl_planner_ros/parameter_path", "ompl_rs_planner_ros/nav_msgs_path");
  const auto plannerParameters = loadOmplReedsSheppPlannerParameters(filename);
  const auto plannerRosParameters = loadOmplReedsSheppPlannerRosParameters(filename);

  ros::Subscriber gridMapSub = nh->subscribe("/se2_grid_map_generator_node/grid_map", 1, gridMapCallback);

  planner = std::make_shared<OmplReedsSheppPlanner>();
  planner->setParameters(plannerParameters);

  // ros 연동 부분
  se2_planning::OmplReedsSheppPlannerRos plannerRos(nh);
  plannerRos.setPlanningStrategy(planner);
  plannerRos.setParameters(plannerRosParameters);
  plannerRos.initialize();

  // ompl 플래너 생성
  OmplPlannerParameters plannerOmplParameters;
  const std::string plannerName = plannerParameters.omplPlannerName_;
  loadOmplPlannerParameters(plannerName, filename, &plannerOmplParameters);
  auto omplPlanner = createPlanner(planner->getSimpleSetup()->getSpaceInformation(), plannerName);
  setPlannerParameters(plannerOmplParameters, plannerName, omplPlanner);
  planner->setOmplPlanner(omplPlanner);

  ros::spin();

  return 0;
}
