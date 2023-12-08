#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include "OccupancyGrid.h" 
#include "PathPlannerInterface.h"

/**
 * @brief Visualizes the occupancy grid and saves it as an image.
 *
 * This function creates a visual representation of an occupancy grid where free 
 * and occupied cells are displayed with different colors. It uses OpenCV to create 
 * and save the image. Occupied cells in the grid are marked, and the grid is saved
 * as an image file. The function assumes a 2D grid where the z-coordinate is ignored.
 *
 * @param grid The OccupancyGrid object representing the occupancy grid.
 * @param trajectory A vector of Eigen::Vector3f representing the trajectory. Each
 *                   Vector3f contains the x, y, and z coordinates of a point on the
 *                   trajectory. Only the x and y coordinates are used for visualization.
 * @param cellSize The size of each cell in the visualization in pixels. Default is 10.
 *                 This determines the resolution of the output image.
 *
 * @note The output image ('trajectory_grid.png') is saved in the current working directory.
 *       The function uses OpenCV, so ensure OpenCV is correctly linked in your project.
 */
void visualizeGrid(const OccupancyGrid& grid, const Eigen::Vector3f& start, const Eigen::Vector3f& end) {
    int cellSize = 1; // Size of each grid cell in the image (pixels)
    cv::Mat image(grid.height * cellSize, grid.width * cellSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // Draw the grid
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            int index = grid.get1DIndex(x, y);
            if (grid.data[index]) {
                cv::rectangle(image, 
                              cv::Point(x * cellSize, y * cellSize), 
                              cv::Point((x + 1) * cellSize, (y + 1) * cellSize), 
                              cv::Scalar(0, 0, 255), // Red for occupied cells
                              -1);
            }
        }
    }

    // Calculate the radius for start and end points
    int pointRadius = 5;

    // Draw start and end points with increased radius
    cv::circle(image, 
               cv::Point(static_cast<int>(start.x() / grid.resolution_m * cellSize),
                         static_cast<int>(start.y() / grid.resolution_m * cellSize)), 
               pointRadius, cv::Scalar(0, 255, 0), -1); // Green for start
    cv::circle(image, 
               cv::Point(static_cast<int>(end.x() / grid.resolution_m * cellSize),
                         static_cast<int>(end.y() / grid.resolution_m * cellSize)), 
               pointRadius, cv::Scalar(255, 0, 0), -1); // Blue for end

    //Save the image
    cv::imwrite("occupancy_grid.png", image);
    // Display the image
    cv::imshow("Occupancy Grid", image);
    cv::waitKey(0); // Wait for a key press

}

/**
 * @brief Visualizes an occupancy grid and a trajectory on it, and saves the result as an image.
 *
 * This function creates a visual representation of an occupancy grid, along with a trajectory path,
 * using OpenCV. The occupancy grid is represented with cells marked as occupied or free, and the
 * trajectory is visualized as a continuous line or series of points on the grid. The image is then
 * saved to a file.
 *
 * @param grid The OccupancyGrid object, representing the 2D occupancy grid. The grid contains
 *             information about the size, resolution, and occupancy status of each cell.
 * @param trajectory A Trajectory object (std::vector<Eigen::Vector3f>), representing the sequence
 *                   of points that constitute the trajectory. Each Eigen::Vector3f contains x, y,
 *                   and z coordinates, with z being ignored for 2D visualization.
 * @param cellSize (Optional) The size of each cell in the visualization, in pixels. This size
 *                 determines the resolution of the output image. The default value is 10 pixels.
 *
 * @note The function generates an image file named 'trajectory_grid.png' in the current working
 *       directory. This function requires OpenCV to be properly installed and linked in your project.
 *       Ensure that the grid and trajectory data are correctly populated before calling this function.
 */
void visualizeGridAndTrajectory(const OccupancyGrid& grid, const Trajectory& trajectory) {
    int cellSize = 1; // Size of each grid cell in the image (pixels)
    cv::Mat image(grid.height * cellSize, grid.width * cellSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // Draw the grid
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            int index = grid.get1DIndex(x, y);
            if (grid.data[index]) {
                cv::rectangle(image, 
                              cv::Point(x * cellSize, y * cellSize), 
                              cv::Point((x + 1) * cellSize, (y + 1) * cellSize), 
                              cv::Scalar(0, 0, 255), // Red for occupied cells
                              -1); // Filled
            }
        }
    }


    // Draw the trajectory
    for (size_t i = 1; i < trajectory.size(); ++i) {
        cv::Point p1(static_cast<int>(trajectory[i - 1].x() / grid.resolution_m * cellSize),
                    static_cast<int>(trajectory[i - 1].y() / grid.resolution_m * cellSize));
        cv::Point p2(static_cast<int>(trajectory[i].x() / grid.resolution_m * cellSize),
                    static_cast<int>(trajectory[i].y() / grid.resolution_m * cellSize));
        cv::line(image, p1, p2, cv::Scalar(0, 255, 0), 2); // Green line for trajectory
    }

    // Save the image
    cv::imwrite("trajectory_grid.png", image);
    // Display the image
    cv::imshow("Trajectory", image);
    cv::waitKey(0); // Wait for a key press
}


/**
 * @brief Generates an occupancy grid from a grayscale image.
 *
 * This function reads a grayscale image from the given file path and generates
 * an occupancy grid based on the image's pixel intensities. The function assumes
 * that darker pixels represent obstacles and lighter pixels represent free space.
 * Each pixel's intensity is compared against a threshold to determine if it
 * should be considered an obstacle.
 *
 * The generated occupancy grid will have the same dimensions as the provided
 * image, resized if necessary, and each cell in the grid will correspond to a
 * pixel in the image.
 *
 * @param grid A reference to an OccupancyGrid object where the occupancy data
 *             will be stored. The grid's width, height, and resolution should be
 *             set appropriately before calling this function.
 * @param imagePath The file path of the grayscale image to be processed.
 *
 * @throw std::runtime_error If the image cannot be read or is empty.
 *
 * @note The function uses a threshold value (typically 128) to distinguish
 *       obstacles from free space. This threshold can be adjusted as needed.
 * @note The OccupancyGrid's `setAsObstacle` method is used to mark cells as
 *       occupied based on the image data.
 */
void generateOccupancyGridFromImage(OccupancyGrid &grid, const std::string &imagePath) {
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        throw std::runtime_error("Could not read the image: " + imagePath);
    }

    // Resize image to match grid dimensions if necessary
    if (image.cols != grid.width || image.rows != grid.height) {
        cv::resize(image, image, cv::Size(grid.width, grid.height));
    }

    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            if (image.at<uchar>(y, x) < 255) {  // Assuming darker pixels are obstacles
                float x_coordinate = x*grid.resolution_m;
                float y_coordinate = y*grid.resolution_m;
                grid.setAsObstacle(x_coordinate, y_coordinate );
            }
        }
    }
}