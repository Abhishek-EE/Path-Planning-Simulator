#include <iostream>
#include <memory>
#include<libconfig.h++>
#include "OccupancyGrid.h"
#include "PathPlannerInterface.h"
#include "GenerateGrid.h"
#include "utility.h"
#include<yaml-cpp/yaml.h>

// #define INTERFACE_NOT_IMPLEMENTED  // remove when interface implemented


int main() {

  // Read YAML configuration file
  YAML::Node config = YAML::LoadFile("config/simulation.yaml");

  // Extract parameters from YAML configuration
  Eigen::Vector3f start(config["start"]["x"].as<float>(), config["start"]["y"].as<float>(), config["start"]["z"].as<float>());
  Eigen::Vector3f end(config["end"]["x"].as<float>(), config["end"]["y"].as<float>(), config["end"]["z"].as<float>());
  const float robotDia = config["robot_dia"].as<float>();
  bool isValid = config["is_valid"].as<bool>();
  float occupancyRatio = config["occupancy_ratio"].as<float>();
  bool useImageGrid = config["use_image_grid"].as<bool>();

  // // Default Values
  // Eigen::Vector3f start(3.0f, 4.0f, 0.0f); // Example start position
  // Eigen::Vector3f end(13.0f, 12.0f, 0.0f); // Example end position
  // const float robotDia = 0.6;
  // bool isValid= true;
  // float occupancyratio = 0.2;
  // bool useImageGrid = true;

  

  Trajectory path;
  PathPlannerImplementation planner(robotDia);
  OccupancyGrid grid;
  GenerateGridImplementation gridGenerator(robotDia,occupancyRatio);
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

  return 0;
}
