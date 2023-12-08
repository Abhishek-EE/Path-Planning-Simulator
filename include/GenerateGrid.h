#pragma once
#include<unordered_set>
#include<OccupancyGrid.h>
#include<Eigen/Dense>
#include<memory>
#include <algorithm>
#include <PathPlannerInterface.h>

/**
* @class Generate Grid
*/
class GenerateGridInterface {

    public:
        virtual void generateGrid(const Eigen::Vector3f &start,
                                           const Eigen::Vector3f &end,
                                           const bool isValid=false) = 0;

};
/*
This is important and I should finish this
How much money are you spending
FLight - 309 + 90 + 137 for cabin for time being that's it
generateGrid Function
grid, start, end, isvalid
Trajectory = generaterandomtrajectory()
addObstacles()
*/

/**
 * @brief 
 * 
 */
class GenerateGridImplementation: public GenerateGridInterface{
    private:
        OccupancyGrid grid;
        std::unordered_set<int> trajectory_index;
        float robotDia = 0.5;
        float occupancyRatio = 0.3;
        
        void generateValidPath(const Eigen::Vector3f &start,
                                const Eigen::Vector3f &end);
        void addObstacles();
        
        void setOccupancyRatio(float ratio) {
            occupancyRatio = std::clamp(ratio, 0.0f, 1.0f);
        }

    public:

        GenerateGridImplementation() = default;
        
        GenerateGridImplementation(float robotdia, float occupancyratio);
        
        void generateGrid(const Eigen::Vector3f &start,
                                   const Eigen::Vector3f &end,
                                   const bool isValid=false) override;
        
        const OccupancyGrid& getGrid() const{
            return grid;
        }
        Trajectory getValidTrajectory()const;
};