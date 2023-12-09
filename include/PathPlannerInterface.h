#pragma once
#include <Eigen/Dense>
#include "OccupancyGrid.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>

typedef std::vector<Eigen::Vector3f> Trajectory;

class PathPlannerInterface {

public:
  virtual Trajectory getCollisionFreePath(const OccupancyGrid &grid,
                                          const Eigen::Vector3f &start,
                                          const Eigen::Vector3f &end) = 0;

};

class PathPlannerImplementation : public PathPlannerInterface {
public:
    PathPlannerImplementation(float robotDiameter);
    Trajectory getCollisionFreePath(const OccupancyGrid &grid,
                                    const Eigen::Vector3f &start,
                                    const Eigen::Vector3f &end) override;

private:
    struct Node {
        Eigen::Vector3f position; // Position in the grid
        float gCost;              // Cost from start to this node
        float hCost;              // Heuristic cost from this node to end
        float fCost() const { return gCost + hCost; } // Total cost
        std::shared_ptr<Node> parent; // Parent node in the path

        // Constructor and other necessary member functions
    };

    float robotDia;

    Trajectory reconstructPath(std::shared_ptr<Node> endNode) const;
    std::vector<std::shared_ptr<Node>> getNeighbors(std::shared_ptr<Node> node,const Eigen::Vector3f &end, const OccupancyGrid &grid) const;
    bool isTraversable(const Eigen::Vector3f &point, const OccupancyGrid &grid) const;
    float heuristic(const Eigen::Vector3f &point1, const Eigen::Vector3f &point2) const;
    Eigen::Vector3f transformRobotToGridFrame(const Eigen::Vector3f& robotFramePoint, const Eigen::Vector3f& robotPosition) const;
    // Additional helper functions as needed
};