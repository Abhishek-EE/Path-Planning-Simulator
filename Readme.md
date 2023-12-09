# Planning and Controls Assessment

## Problem Statement

A circular roomba robot is present in an already mapped environment. The map is in the format of a binary occupancy
grid, e.g. [grid.png](grid.png). Each cell in the grid contains a boolean value indicating if an obstacle is present at
a given location, i.e `true -> obstacle` and `false -> free`. The `OccupancyGrid` struct contains a 1d array of boolean
that denotes a 2d grid in a row major format. The intent is to find a safe **collision-free** path from a given location
to the vicinity of the goal location.

Create a class that inherits the `PathPlannerInterface` class, and implement the `getCollisionFreePath()` pure virtual
function to provide a trajectory which can be followed by the robot. Note that the start position and goal position may
be any point within the occupancy grid. Take any assumptions based on the code provided already in the repository, but
state the assumptions clearly in the Assumptions section below. We will analyze the design of the system as well as the
algorithm. Try to stick to Eigen and STL libs for implementation.

## System Description

### Grid

The origin of the occupancy grid is the bottom left of the image. The positive x-axis is to the right and positive
y-axis is upwards. The grid is of size `320 x 320` with a resolution of `0.05 m`. The `OccupancyGrid` class has methods
to set occupancy for a given coordinate, which could be useful for testing solutions. Feel free to edit this part of the
code as well as use any kind of visualization assist (Not needed for this assessment).

### Robot

The robot is a spherical robot of diameter `0.6m` and it operates with a 2d coordinate system with $(x.y. \theta)$ where
$\theta$ is the yaw(orientation) of the robot.

## Dependencies

### Eigen

Eigen is a linear algebra library. Documentation to use Eigen is present
[here](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html). A cheat sheet with quick usages for Eigen Data
structures and functions can be found [here](https://gist.github.com/gocarlos/c91237b02c120c6319612e42fa196d77)

To install Eigen, run the following command:

    sudo apt install libeigen3-dev

### OpenCV

OpenCV is an open-source computer vision and machine learning software library. Documentation for OpenCV can be found
[here](https://docs.opencv.org/master/). Tutorials and guides for various OpenCV functions are available
[here](https://docs.opencv.org/master/d9/df8/tutorial_root.html).

To install OpenCV on Ubuntu, you can use the following command:

    sudo apt install libopencv-dev

For other operating systems or advanced installation options, refer to the official OpenCV installation guides
[here](https://opencv.org/releases/).

## Build

To build , run

    make -j8 && bin/CheckPath

Feel free to edit the [main.cpp](src/main.cpp) to run the Path planner with any start and stop locations in any random
grid.

## Assumptions
1. The robot could rotate 360 degrees in place, given the robot is a Roomba seemed like a good enough assumption.


## Approach
This problem has mainly three subproblems that need addressing
1. Path Planning in a grid
2. How to deal with the mismatch in grid resolution and robot diameter
3. Smoothning the generated path

### Path Planning in a Grid
For this part, I decided to assume that the current Occupancy Grid is a 2D graph,
and any Single Source Shortest Path algorithm will technically work. I decided to go 
with A*, as A* combines features of Dijkstra's Algorithm (which is efficient but can 
be slow because it explores all directions equally) and the Greedy Best-First-Search 
(which is fast but can be inaccurate). 

### Tackling Grid Resolution and robot Diameter
There were two approaches available to me for solving this problem,
The first one was inflating the obstacles in the grid; after
iterating through the entire grid and inflating the obstacles by marking the cells which
are a diameter radius away from them as occupied. This approach although promising will be 
very compute expensive as it will scale directly with grid size, sometimes we only have partial 
information and we only know some parts of the grid.
The second approach that I finally decided to go with is projecting a robot size square in front of the robot
and then rotating that square to 360 degrees to calculate 8 neighbors and make sure they are traversable,
if even a single point in the project square is not traversable it is considered as not traversable.
This approach has a constant time addition cost and does not add an O(N*M) overhead to A*.

### Smoothing the Path
I am using a simple line smoother technique to make sure that the robot  can traverse the path without needing
to take any unreasonable sharp turns.

### Grid Generator
In addition to this work, I also added a grid generator to test my code, which I think will be useful in your case too.
There is also generateOccupancyGridFromImage Function -> Please add the path to the image there and it will generate the grid as per request.

### Visualization
I added the visualization to help me debug and test my code, you can find the implementation in the utility.h
You will need Open CV for that as dependencies





