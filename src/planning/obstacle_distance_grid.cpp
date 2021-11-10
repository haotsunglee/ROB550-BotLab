#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::initializeDistances(const OccupancyGrid& map)
{
    int width = map.widthInCells();
    int height = map.heightInCells();
    for(int y=0; y<height; y++){
        for(int x=0; x<width; x++){
            // free
            if(map.logOdds(x, y)<0){
                distance(x, y) = -1;
            }// occupied
            else{
                distance(x, y) = 0;
            }
        }
    }
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    initializeDistances(map);


    std::priority_queue<DistanceNode> searchQueue;
    enqueue_obstacle_cells(*this, searchQueue);

    while(!searchQueue.empty()){
        DistanceNode nextNode = searchQueue.top();
        searchQueue.pop();
        expand_node(nextNode, *this, searchQueue);
    }
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}


void ObstacleDistanceGrid::enqueue_obstacle_cells(ObstacleDistanceGrid& grid, std::priority_queue<DistanceNode>& searchQueue){
    int width = grid.widthInCells();
    int height = grid.heightInCells();
    cell_t cell;

    for(cell.y = 0; cell.y < height; cell.y++){
        for(cell.x = 0; cell.x < width; cell.x++){
            if(distance(cell.x, cell.y)==0){
                expand_node(DistanceNode(cell, 0.0), grid, searchQueue);

            }
        }
    }
}


void ObstacleDistanceGrid::expand_node(const DistanceNode& node, ObstacleDistanceGrid& grid, std::priority_queue<DistanceNode>& searchQueue){
    const int xDeltas[8] = {1, 1, 0, 1, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1, 1, 1, 0, -1};
    const float diagnol = std::sqrt(2);
    for(int i=0; i<8; i++){
        if(i%2==1) continue;
        cell_t adjacentCell(node.cell.x + xDeltas[i],  node.cell.y + yDeltas[i]);
        if(grid.isCellInGrid(adjacentCell.x, adjacentCell.y)){
            if(grid(adjacentCell.x, adjacentCell.y) == -1){
                
                DistanceNode adjacentNode(adjacentCell, node.distance + 1.0);
                // if(i%2==1)
                //    adjacentNode.distance = node.distance + diagnol;

                grid(adjacentCell.x, adjacentCell.y) = adjacentNode.distance * grid.metersPerCell();
                searchQueue.push(adjacentNode);
            }
        }
    }
}