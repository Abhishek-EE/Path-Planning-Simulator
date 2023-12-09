#include "PathPlannerInterface.h"
#include <limits>
#include <cmath>
#include <unordered_set>
#include <opencv2/opencv.hpp>
// Constructor
PathPlannerImplementation::PathPlannerImplementation(float robotDiameter)
    : robotDia(robotDiameter) {}

// A* Pathfinding Algorithm
/**
 * @brief Calculates a collision-free path from a start point to an end point within a given occupancy grid.
 *
 * This method implements the A* pathfinding algorithm to determine a collision-free path in a grid. It uses a heuristic
 * function to estimate the cost from the current node to the end point and a priority queue to efficiently select the
 * next node to process. The method handles out-of-bound errors and ensures that the path avoids occupied cells in the grid.
 *
 * @param grid The occupancy grid representing the environment, where obstacles and free spaces are marked.
 * @param start An Eigen::Vector3f representing the starting position (x, y, theta) in the grid.
 * @param end An Eigen::Vector3f representing the goal position (x, y, theta) in the grid.
 * @return Trajectory A vector of Eigen::Vector3f points representing the calculated path from start to end. 
 *         Returns an empty vector if no path is found.
 *
 * @throws std::out_of_range If an index is out of bounds, indicating an invalid position in the occupancy grid.
 *
 * @note The function uses a set of 'closed' nodes to keep track of visited positions and another set for nodes
 *       that are in the priority queue ('open' nodes) to avoid reprocessing. The robot's diameter (robotDia) is
 *       used to check proximity to the goal point.
 * @note The heuristic function and getNeighbors method are assumed to be defined and used for estimating costs
 *       and generating neighboring nodes, respectively.
 */
Trajectory PathPlannerImplementation::getCollisionFreePath(const OccupancyGrid &grid,
                                                           const Eigen::Vector3f &start,
                                                           const Eigen::Vector3f &end) {
    // The open set for A* (nodes to be evaluated)
    auto compare = [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) { 
        return a->fCost() > b->fCost(); 
    };
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(compare)> openQueue(compare);

    // Create and add the start node
    auto startNode = std::make_shared<Node>();
    startNode->position = start;
    startNode->gCost = 0;
    startNode->hCost = heuristic(start, end);
    openQueue.push(startNode);

    //Set to keep track of visited and in queue nodes
    std::unordered_set<int> closedSet;
    std::unordered_set<int> openSet;

    int iter=0; //To keep track of maximum iterations

    while (!openQueue.empty()) {
        
        ++iter;
        if (iter > 100000) {
            std::cerr << "Warning: Maximum Iterations Reached, the goal seems unreachable" << std::endl;
        }
        
        //Get the current node and add it to the closedSet (that is it is already visited)
        auto currentNode = openQueue.top();
        openQueue.pop();
        int index = getPointIndex(currentNode->position,grid);
        if (index < 0) {
            throw std::out_of_range("Index out of bounds: Something went horribly wrong");
        }
        closedSet.insert(index);

        // Goal check
        if (heuristic(currentNode->position, end) < robotDia) {
            return smoothTrajectory(reconstructPath(currentNode),3);
        }

        // Generate neighbors (only neighbors which are traversable get added)
        auto neighbors = getNeighbors(currentNode, end, grid);
        for (const auto& neighbor : neighbors) {
            // Skip if the neighbor is already in the closed set
            int index = getPointIndex(neighbor->position,grid);
            if (index < 0) {
                throw std::out_of_range("Index out of bounds: Something went horribly wrong");
            }
            if (closedSet.find(index) != closedSet.end()) {
                continue;
            }

            // Calculate tentative gCost
            float tentativeGCost = currentNode->gCost + (neighbor->position.head<2>() - currentNode->position.head<2>()).norm();
            
            //If the neighbor already visited then only add it to queue if new score less than the old
            if(openSet.find(index) != openSet.end()){
            if (tentativeGCost < neighbor->gCost) {
                neighbor->parent = currentNode;
                neighbor->gCost = tentativeGCost;
                neighbor->hCost = heuristic(neighbor->position, end);
                openQueue.push(neighbor);
                openSet.insert(index);
            }
            }
            else{
                openSet.insert(index);
                openQueue.push(neighbor);
            }

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
bool PathPlannerImplementation::isTraversable(const Eigen::Vector3f& robotLocation, const OccupancyGrid& grid) const {
    float buffer = 0.1*robotDia; //Adding extra buffer to make sure that the robot could safely traverse to this point
    for(float x=-((robotDia/2)); x<=(robotDia/2)+2*buffer;x+=grid.resolution_m){
        for(float y=-((robotDia/2)+buffer);y<=(robotDia/2)+buffer;y+=grid.resolution_m){
            Eigen::Vector3f robotFramePoint(x,y,0);
            Eigen::Vector3f gridFramePoint = transformRobotToGridFrame(robotFramePoint,robotLocation);
            int index = getPointIndex(gridFramePoint,grid);
            if(index < 0 || grid.data[index]){//Index out of range not traversable or index has an obstacle not traversable
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

//Function to smoothen the trajectory
Trajectory PathPlannerImplementation::smoothTrajectory(const Trajectory& inputTrajectory, int windowSize) {
    Trajectory smoothedTrajectory = inputTrajectory;

    if (windowSize <= 1 || inputTrajectory.size() <= 2) {
        return smoothedTrajectory; // No smoothing needed
    }

    for (size_t i = 1; i < inputTrajectory.size() - 1; ++i) {
        Eigen::Vector3f sum = inputTrajectory[i]; // start with the current point

        int count = 1; // start with the current point
        for (int j = 1; j <= windowSize / 2; ++j) {
            if (i - j > 0) {
                sum += inputTrajectory[i - j];
                ++count;
            }
            if (i + j < inputTrajectory.size()) {
                sum += inputTrajectory[i + j];
                ++count;
            }
        }

        smoothedTrajectory[i] = sum / static_cast<float>(count); // calculate the average
    }

    return smoothedTrajectory;
}
