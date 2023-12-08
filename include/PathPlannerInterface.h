#pragma once
#include <Eigen/Dense>
#include "OccupancyGrid.h"
#include <vector>

typedef std::vector<Eigen::Vector3f> Trajectory;

class PathPlannerInterface {

public:
  virtual Trajectory getCollisionFreePath(const OccupancyGrid &grid,
                                          const Eigen::Vector3f &start,
                                          const Eigen::Vector3f &end) = 0;

};

// class PathPlannerImplementation: public PathPlannerInterface{

// public:
//   Trajectory getCollisionFreePath(const OccupancyGrid &grid,
//                                   const Eigen::Vector3f &start,
//                                   const Eigen::Vector3f &end) override{

//                                   }
// };