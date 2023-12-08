cmake_minimum_required(VERSION 3.0.2)
project(interview-planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(/usr/local/include/)
include_directories(/usr/include/eigen3/)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_executable(CheckPath src/main.cpp src/GenerateGrid.cpp)
target_link_libraries(CheckPath ${LIBS} ${OpenCV_LIBS})
