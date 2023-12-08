#include <iostream>
#include <memory>
#include "OccupancyGrid.h"
#include "PathPlannerInterface.h"
#include "GenerateGrid.h"
#include "utility.h"

// #define INTERFACE_NOT_IMPLEMENTED  // remove when interface implemented

int main() {

  // std::unique_ptr<PathPlannerInterface> planner;
  Eigen::Vector3f start(0.0f, 0.0f, 0.0f); // Example start position
  Eigen::Vector3f end(15.0f, 15.0f, 0.0f); // Example end position
  const float robotDia = 0.6;
  bool isValid= true;
  float occupancyratio = 0.5;
#ifdef INTERFACE_NOT_IMPLEMENTED
  std::cout << "Hello World!\n";
#else
  // grid = std::make_unique<PathPlannerImplementation>();/// PathPlannerImplementation is the derived class
  // auto start = Eigen::Vector3f::Identity();
  // auto end = Eigen::Vector3f::Identity();
  // planner->getCollisionFreePath(grid, start, end);
  GenerateGridImplementation gridGenerator(robotDia,occupancyratio);
  gridGenerator.generateGrid(start,end,isValid);
  const OccupancyGrid& grid = gridGenerator.getGrid();
  // grid.visualizeGrid(start,end);
  // grid.visualizeGrid(start,end);
  visualizeGrid(grid,start,end);


#endif
  return 0;
}
