#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <lcmtypes/robot_path_t.hpp>
#include <cmath>


MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    setParams(params);
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, 
                                     const pose_xyt_t& goal, 
                                     const SearchParams& searchParams) const
{
    // If the goal isn't valid, then no path can actually exist
    //std::cout << "goal = " << goal.x << " " << goal.y << std::endl;
    
    if(!isValidGoal(goal))
    {   
        robot_path_t failedPath;
        failedPath.utime = utime_now();
        failedPath.path_length = 1;
        failedPath.path.push_back(start);
        std::cout << "INFO: path rejected due to invalid goal\n";        

        return failedPath;
    }
    
    // Otherwise, use A* to find the path
    robot_path_t path =  search_for_path(start, goal, distances_, searchParams);
    if(isPathSafe(path)) 
        return path;
    else{
        path.path.clear();
        return path;
    }
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, const pose_xyt_t& goal) const
{
    return planPath(start, goal, searchParams_);
}


bool MotionPlanner::isValidGoal(const pose_xyt_t& goal) const
{
    // std::cout << "prev_goal x and y: " << prev_goal.x << " " << prev_goal.y << std::endl;
    // std::cout << "num of frontiers" << " " << num_frontiers << std::endl;
    float dx = goal.x - prev_goal.x, dy = goal.y - prev_goal.y;
    float distanceFromPrev = std::sqrt(dx * dx + dy * dy);

    //if there's more than 1 frontier, don't go to a target that is within a robot diameter of the current pose
    // if(num_frontiers != 1 && distanceFromPrev < 2 * searchParams_.minDistanceToObstacle) return false;

    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);

    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // std::cout << "distance = " << distances_(goalCell.x, goalCell.y)  << " " << "robotradius" << params_.robotRadius << std::endl;
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        if(distances_(goalCell.x, goalCell.y)) return true;
        return distances_(goalCell.x, goalCell.y) > params_.robotRadius;
    }
    
    // A goal must be in the map for the robot to reach it
    return false;
}


bool MotionPlanner::isPathSafe(const robot_path_t& path) const
{

    ///////////// TODO: Implement your test for a safe path here //////////////////
    for(auto& p : path.path){
        Point<double> temp;
        temp.x = p.x;
        temp.y = p.y;
        Point<double> temp2 = global_position_to_grid_cell(temp, distances_);
        std::cout<<distances_(temp2.x, temp2.y)<<std::endl;
        if(distances_(temp2.x, temp2.y) == -1) continue;
        if(distances_(temp2.x, temp2.y) <=  (float)searchParams_.minDistanceToObstacle);
            return false; 
    }
    return true;
}


void MotionPlanner::setMap(const OccupancyGrid& map)
{
    distances_.setDistances(map);
}


void MotionPlanner::setParams(const MotionPlannerParams& params)
{
    searchParams_.minDistanceToObstacle = params_.robotRadius;
    searchParams_.maxDistanceWithCost = 10.0 * searchParams_.minDistanceToObstacle;
    searchParams_.distanceCostExponent = 1.0;
}
