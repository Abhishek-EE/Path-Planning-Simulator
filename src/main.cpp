#include <iostream>
#include <memory>
#include "OccupancyGrid.h"
#include "PathPlannerInterface.h"
#include "GenerateGrid.h"
#include "utility.h"

// #define INTERFACE_NOT_IMPLEMENTED  // remove when interface implemented

int main() {

  // std::unique_ptr<PathPlannerInterface> planner;
  Eigen::Vector3f start(2.1f, 3.2f, 0.0f); // Example start position
  Eigen::Vector3f end(15.0f, 15.0f, 0.0f); // Example end position
  const float robotDia = 0.6;
  bool isValid= true;
  float occupancyratio = 0.2;
#ifdef INTERFACE_NOT_IMPLEMENTED
  std::cout << "Hello World!\n";
#else
  Trajectory path;
  // GenerateGridImplementation gridGenerator(robotDia,occupancyratio);
  PathPlannerImplementation planner(robotDia);
  // gridGenerator.generateGrid(start,end,isValid);
  // const OccupancyGrid& grid = gridGenerator.getGrid();
  OccupancyGrid grid;
  generateOccupancyGridFromImage(grid,"/home/abhishek/dev/Planning_Assingment/grid-example.png");
  visualizeGrid(grid,start,end);
  path = planner.getCollisionFreePath(grid,start,end);
  // visualizeGridAndTrajectory(grid,path);
#endif
  return 0;
}
