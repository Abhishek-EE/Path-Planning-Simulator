#include <iostream>
#include <memory>
#include "OccupancyGrid.h"
#include "PathPlannerInterface.h"

#define INTERFACE_NOT_IMPLEMENTED  // remove when interface implemented

int main() {

  OccupancyGrid grid{};
  std::unique_ptr<PathPlannerInterface> planner;
#ifdef INTERFACE_NOT_IMPLEMENTED
  std::cout << "Hello World!\n";
#else
  grid = std::make_unique<PathPlannerImplementation>();/// PathPlannerImplementation is the derived class
  auto start = Eigen::Vector3f::Identity();
  auto end = Eigen::Vector3f::Identity();
  planner->getCollisionFreePath(grid, start, end);
  // print function for vector
#endif
  return 0;
}
