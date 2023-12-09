#include "PathPlannerInterface.h"
#include <limits>
#include <cmath>
#include <unordered_set>

struct Vector2fHash {
    size_t operator()(const Eigen::Vector2f& key) const {
        // Simple example of combining the hash of each component
        // This is a basic approach and can be improved for better hash distribution
        size_t xHash = std::hash<float>()(key.x());
        size_t yHash = std::hash<float>()(key.y()) << 1; // Bit shift to mix the hashes
        // size_t zHash = std::hash<float>()(key.z()) << 2; // Bit shift to mix the hashes

        return xHash ^ yHash ; // XOR to combine the hashes
    }
};


// Constructor
PathPlannerImplementation::PathPlannerImplementation(float robotDiameter)
    : robotDia(robotDiameter) {}

// A* Pathfinding Algorithm
Trajectory PathPlannerImplementation::getCollisionFreePath(const OccupancyGrid &grid,
                                                           const Eigen::Vector3f &start,
                                                           const Eigen::Vector3f &end) {
    // The open set for A* (nodes to be evaluated)
    auto compare = [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) { 
        return a->fCost() > b->fCost(); 
    };
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(compare)> openSet(compare);

    // Create and add the start node
    auto startNode = std::make_shared<Node>();
    startNode->position = start;
    startNode->gCost = 0;
    startNode->hCost = heuristic(start, end);
    openSet.push(startNode);

    // Set of visited nodes to avoid processing a node more than once
    std::unordered_set<Eigen::Vector2f, Vector2fHash> closedSet;

    while (!openSet.empty()) {
        auto currentNode = openSet.top();
        openSet.pop();

        // Add to closed set
        closedSet.insert(currentNode->position.head<2>());

        // Goal check
        if (heuristic(currentNode->position, end) < robotDia) {
            return reconstructPath(currentNode);
        }

        // Generate neighbors
        auto neighbors = getNeighbors(currentNode, end, grid);
        for (const auto& neighbor : neighbors) {
            // Skip if the neighbor is already in the closed set
            if (closedSet.find(neighbor->position.head<2>()) != closedSet.end()) {
                continue;
            }

            // Calculate tentative gCost
            float tentativeGCost = currentNode->gCost + (neighbor->position.head<2>() - currentNode->position.head<2>()).norm();

            if (tentativeGCost < neighbor->gCost) {
                neighbor->parent = currentNode;
                neighbor->gCost = tentativeGCost;
                neighbor->hCost = heuristic(neighbor->position, end);
            }
            openSet.push(neighbor);
        }
    }

    return Trajectory{}; // Return an empty trajectory if no path is found
}


// Reconstruct the path from end node to start node
Trajectory PathPlannerImplementation::reconstructPath(std::shared_ptr<Node> endNode) const {
    Trajectory path;
    for (auto node = endNode; node != nullptr; node = node->parent) {
        path.push_back(node->position);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Generate neighbor nodes for a given node
std::vector<std::shared_ptr<PathPlannerImplementation::Node>> PathPlannerImplementation::getNeighbors(std::shared_ptr<Node> node,const Eigen::Vector3f &end, const OccupancyGrid &grid) const {
    std::vector<std::shared_ptr<Node>> neighbors;
    for (int i = 0; i < 8; ++i) {
        float angle = 2 * M_PI * i / 8;
        Eigen::Vector3f neighborPoint = node->position + Eigen::Vector3f(robotDia * cos(angle), robotDia * sin(angle), angle);
        if(isTraversable(neighborPoint,grid)){
            auto neighborNode = std::make_shared<Node>();
            neighborNode->position = neighborPoint;
            neighborNode->gCost = node->gCost+robotDia;
            neighborNode->hCost = heuristic(neighborPoint, end);
            neighborNode->parent = node;
            neighbors.push_back(neighborNode);
        }
    }   
    return neighbors;
}

// Check if a point is traversable, considering the robot's diameter
bool PathPlannerImplementation::isTraversable(const Eigen::Vector3f &robotLocation, const OccupancyGrid &grid) const {

    for(float x=-robotDia/2; x<=robotDia/2;x+=grid.resolution_m){
        for(float y=-robotDia/2;y<=robotDia/2;y+=grid.resolution_m){
            Eigen::Vector3f robotFramePoint(x,y,0);
            Eigen::Vector3f gridFramePoint = transformRobotToGridFrame(robotFramePoint,robotLocation);
            
            int xIndex = static_cast<int>((gridFramePoint.x() / grid.resolution_m));
            int yIndex = static_cast<int>((gridFramePoint.y() / grid.resolution_m));
            int index = grid.get1DIndex(xIndex,yIndex);
            if(index == -1 || grid.data[index]){//Index out of range not traversable or index has an obstacle not traversable
                return false;
            }
        }
    }
    return true;
}

// Heuristic function (e.g., Euclidean distance)
float PathPlannerImplementation::heuristic(const Eigen::Vector3f &point1, const Eigen::Vector3f &point2) const {
    return (point1.head<2>() - point2.head<2>()).norm(); // Simple Euclidean distance
}

Eigen::Vector3f PathPlannerImplementation::transformRobotToGridFrame(const Eigen::Vector3f& robotFramePoint, const Eigen::Vector3f& robotPosition) const{
    // Extract the position (x, y) and orientation (theta) of the robot
    float robotX = robotPosition.x();
    float robotY = robotPosition.y();
    float robotTheta = robotPosition.z(); // Assuming theta is in radians

    // Create a rotation matrix
    Eigen::Matrix2f rotationMatrix;
    rotationMatrix << std::cos(robotTheta), -std::sin(robotTheta),
                      std::sin(robotTheta), std::cos(robotTheta);

    // Translate the point in the robot frame to the grid frame
    Eigen::Vector2f translatedPoint = rotationMatrix * Eigen::Vector2f(robotFramePoint.x(), robotFramePoint.y()) 
                                      + Eigen::Vector2f(robotX, robotY);

    // Return the transformed point as Eigen::Vector3f (keeping the original z-component)
    return Eigen::Vector3f(translatedPoint.x(), translatedPoint.y(), robotFramePoint.z());
}