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

// class AStarPathPlanner : public PathPlannerInterface {
// public:
//     AStarPathPlanner(float robotDiameter);

//     Trajectory getCollisionFreePath(const OccupancyGrid &grid,
//                                     const Eigen::Vector3f &start,
//                                     const Eigen::Vector3f &end) override;

// private:
//     float robotDia;

//     struct Node {
//         Eigen::Vector3f position;
//         float gCost; // Cost from start to this node
//         float hCost; // Heuristic cost from this node to end
//         float fCost() const { return gCost + hCost; } // Total cost
//         Node* parent; // Parent node in the path

//         // Node constructor and other details
//     };

//     Trajectory reconstructPath(Node* endNode) const;
//     std::vector<Node*> getNeighbors(const Node* node, const OccupancyGrid &grid) const;
//     bool isTraversable(const Eigen::Vector3f &point, const OccupancyGrid &grid) const;
//     float heuristic(const Eigen::Vector3f &point1, const Eigen::Vector3f &point2) const;
    
//     // Additional helper functions as needed
// };