#include "GenerateGrid.h"
#include <random>
#include <vector>
#include <cmath>
#include<iostream>

GenerateGridImplementation::GenerateGridImplementation(float robotDia, float occupancyratio) 
    : robotDia(robotDia) {
        setOccupancyRatio(occupancyratio);
}

/**
 * @brief Generates a valid path from a start point to an end point in the OccupancyGrid.
 * 
 * This function implements a random path generation algorithm. Starting from the 'start' point,
 * it iteratively generates a set of 8 candidate points around the current point, each at a distance
 * equal to the robot's diameter (robotDia). These points are evenly distributed at 45-degree intervals
 * around the current point. One of these points is then randomly selected as the next point on the path,
 * with the selection probability being inversely proportional to the point's distance from the 'end' point.
 * The function repeats this process until the selected point is the same as the 'end' point (in terms of 
 * x and y coordinates).
 * 
 * @param start The starting point of the path in 3D space (x, y, theta), where x and y are the 
 *              centroid coordinates, and theta is the orientation (yaw) of the robot.
 * @param end The end point of the path in 3D space (x, y, theta), with the same conventions as 'start'.
 *            The function considers the path complete when the x and y coordinates of the current point
 *            match those of 'end'.
 */
void GenerateGridImplementation::generateValidPath(const Eigen::Vector3f &start, const Eigen::Vector3f &end) {
    Eigen::Vector3f curr = start;
    std::random_device rd;
    std::mt19937 gen(rd());
    int iter = 0;

    while (iter<100000) {
        std::vector<Eigen::Vector3f> points;
        ++iter;
        for (int i = 0; i < 8; ++i) {
            float angle = 2 * M_PI * i / 8;
            Eigen::Vector3f point = curr + Eigen::Vector3f(robotDia * cos(angle), robotDia * sin(angle), 0);

            // Convert the point to grid coordinates and check if it's within the grid boundaries
            int xIndex = static_cast<int>((point.x() / grid.resolution_m));
            int yIndex = static_cast<int>((point.y() / grid.resolution_m));

            if (xIndex >= 0 && xIndex < grid.width && yIndex >= 0 && yIndex < grid.height) {
                points.push_back(point);
            }
        }

        if (points.empty()) {
            break; // No valid points to choose from
        }

        std::vector<double> weights;
        for (const auto& point : points) {
            double distance = (point - end).head<2>().norm();
            if (distance != 0) { // Avoid division by zero
                weights.push_back(1.0 / pow(distance,4));
            } else {
                weights.push_back(std::numeric_limits<double>::max()); // Max weight for zero distance
            }
        }

        std::discrete_distribution<> dist(weights.begin(), weights.end());
        
        Eigen::Vector3f selectedPoint = points[dist(gen)];
        int index = grid.get1DIndex(static_cast<int>(selectedPoint.x() / grid.resolution_m), 
                                    static_cast<int>(selectedPoint.y() / grid.resolution_m));
        
        if (index >= 0) {
            trajectory_index.insert(index);
        }

        Eigen::Vector3f difference = selectedPoint - end;
        if (difference.head<2>().norm() < robotDia / 2) {
            break; // Destination is close enough
        }

        curr = selectedPoint;
    }
    if (iter >= 100000) {
    std::cerr << "Warning: Exceeded maximum iterations. A valid path may not exist." << std::endl;
}
}
/**
 * @brief Adds obstacles to the grid based on the specified occupancy ratio.
 *
 * This function randomly distributes obstacles across the OccupancyGrid while ensuring
 * that the occupancy ratio of the grid is maintained as specified by the occupancyRatio
 * member variable. The function takes into account the robot's path, as represented by 
 * trajectory_index, and the robot's diameter (robotDia) to avoid placing obstacles too
 * close to the path, thereby ensuring the path remains traversable.
 *
 * The method employs a random distribution strategy to place obstacles on the grid.
 * It calculates the total number of cells to be occupied by obstacles based on the
 * occupancy ratio and iteratively places obstacles in random positions until this
 * number is reached. During each iteration, it checks to ensure that an obstacle
 * is not placed too close to the robot's path. This check considers the robot's
 * diameter and the grid's resolution (grid.resolution_m) to determine proximity
 * to the path.
 *
 * Obstacles are added by setting the corresponding grid cell in the OccupancyGrid
 * data structure to true (occupied). The function respects the grid boundaries and
 * ensures that all obstacles are placed within the valid area of the grid.
 *
 * @note The function uses random number generation for obstacle placement, which can 
 *       result in different grid configurations across different runs. For consistent 
 *       results, a fixed seed may be used for the random number generator.
 */
void GenerateGridImplementation::addObstacles() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> disX(0, grid.width - 1);
    std::uniform_int_distribution<> disY(0, grid.height - 1);

    int totalCells = grid.width * grid.height;
    int obstacleCells = static_cast<int>(occupancyRatio * totalCells);
    int addedObstacles = 0;
    int iter = 0;
    while (addedObstacles < obstacleCells && iter<100000) {
        ++iter;
        int x = disX(gen);
        int y = disY(gen);
        int index = grid.get1DIndex(x, y);

        // Check if the point is too close to the robot's path
        bool tooCloseToPath = false;
        for (int dx = -robotDia/grid.resolution_m; dx <= robotDia/grid.resolution_m; ++dx) {
            for (int dy = -robotDia/grid.resolution_m; dy <= robotDia/grid.resolution_m; ++dy) {
                int pathIndex = grid.get1DIndex(x + dx, y + dy);
                if (trajectory_index.find(pathIndex) != trajectory_index.end()) {
                    tooCloseToPath = true;
                    break;
                }
            }
            if (tooCloseToPath) {
                break;
            }
        }

        // Add obstacle if it's not too close to the path
        if (!tooCloseToPath && index != -1 && !grid.data[index]) {

            for (int dx = -robotDia/grid.resolution_m; dx <= robotDia/grid.resolution_m; ++dx) {
                for (int dy = -robotDia/grid.resolution_m; dy <= robotDia/grid.resolution_m; ++dy) {
                                grid.setAsObstacle((x+dx) * grid.resolution_m, (y+dy) * grid.resolution_m);
                                ++addedObstacles;
                }
            }      

        }

    }
    std::cout<<"something to check";
}

Trajectory GenerateGridImplementation::getValidTrajectory() const{
    
    Trajectory path;
        for (int index : trajectory_index) {
            // Convert 1D index to 2D grid coordinates
            int xIndex = index % grid.width;
            int yIndex = index / grid.width;

            // Convert grid coordinates to world coordinates
            Eigen::Vector3f worldPoint(xIndex * grid.resolution_m, yIndex * grid.resolution_m, 0.0f);
            
            path.push_back(worldPoint);
        }
        return path;
}

void GenerateGridImplementation::generateGrid(const Eigen::Vector3f &start, const Eigen::Vector3f &end, const bool isValid) {

    trajectory_index.clear();
    std::fill(grid.data.begin(),grid.data.end(),0);
    if (isValid) {
        generateValidPath(start, end);
    }
    addObstacles();

}