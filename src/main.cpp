#include <iostream>
#include <memory>
#include "OccupancyGrid.h"
#include "PathPlannerInterface.h"
#include "GenerateGrid.h"
#include "utility.h"

// #define INTERFACE_NOT_IMPLEMENTED  // remove when interface implemented


int main() {

  // std::unique_ptr<PathPlannerInterface> planner;
  Eigen::Vector3f start(3.0f, 4.0f, 0.0f); // Example start position
  Eigen::Vector3f end(13.0f, 12.0f, 0.0f); // Example end position
  const float robotDia = 0.05;
  bool isValid= true;
  float occupancyratio = 0.5;
  bool useImageGrid = true;
#ifdef INTERFACE_NOT_IMPLEMENTED
  std::cout << "Hello World!\n";
#else
  Trajectory path;
  PathPlannerImplementation planner(robotDia);
  OccupancyGrid grid;
  GenerateGridImplementation gridGenerator(robotDia,occupancyratio);
  if(useImageGrid){
    generateOccupancyGridFromImage(grid,"grid-example.png");

  }
  else{
    gridGenerator.generateGrid(start,end,isValid);
    grid = gridGenerator.getGrid();

  }
  visualizeGrid(grid,start,end);
  path = planner.getCollisionFreePath(grid,start,end);
  visualizeGridAndTrajectory(grid,path,start,end);
#endif
  return 0;
}
