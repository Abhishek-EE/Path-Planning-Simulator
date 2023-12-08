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
  Trajectory validPath;
  GenerateGridImplementation gridGenerator(robotDia,occupancyratio);
  gridGenerator.generateGrid(start,end,isValid);
  const OccupancyGrid& grid = gridGenerator.getGrid();
  visualizeGrid(grid,start,end);
  validPath = gridGenerator.getValidTrajectory();
  visualizeGridAndTrajectory(grid,validPath);
  OccupancyGrid grid2;
  generateOccupancyGridFromImage(grid2,"/home/abhishek/dev/Planning_Assingment/grid-example.png");
  visualizeGrid(grid2,start,end);




#endif
  return 0;
}
