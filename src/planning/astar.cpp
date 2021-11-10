#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <algorithm>
#include <common/grid_utils.hpp>

double h_cost(Node* from, Node* goal) {
    int dx = std::abs(goal->cell.x - from->cell.x);
    int dy = std::abs(goal->cell.y - from->cell.y);
    double cost = 0;
    if (dx >= dy) cost = std::sqrt(2) * dy + (dx - dy);
    else cost = std::sqrt(2) * dx + (dy - dx);
    return cost;
}

double g_cost(Node* to, Node* parent){
    int dx = std::abs(to->cell.x - parent->cell.x);
    int dy = std::abs(to->cell.y - parent->cell.y);
    if(dx == 1 && dy == 1) return parent->g_cost + std::sqrt(2);
    else return parent->g_cost + 1;
}


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    int map_width = distances.widthInCells();
    int map_height = distances.heightInCells();
    Point<double> startPoint;
    startPoint.x = start.x;
    startPoint.y = start.y;
    cell_t startCell = global_position_to_grid_cell(startPoint, distances);

    Point<double> goalPoint;
    goalPoint.x = goal.x;
    goalPoint.y = goal.y;
    cell_t goalCell = global_position_to_grid_cell(goalPoint, distances);

    Node start_node(startCell.x, startCell.y);
    Node goal_node(goalCell.x, goalCell.y);
    PriorityQueue open_list;
    NodeList closed_list;
    NodeList searched_list;

    open_list.push(&start_node);
    Node* current_node = open_list.pop();

    robot_path_t path;
    path.utime = start.utime;
    path.path_length = path.path.size();


    if((!current_node->is_in_map(distances) || current_node->is_obstacle(distances, params.minDistanceToObstacle)))
        return path;
    int count = 0;
    while(!(*current_node==goal_node)){
        closed_list.put(current_node);
        expand_node(current_node, distances, params, closed_list, searched_list, open_list, goal_node);
        current_node = open_list.pop();
        count++;
        if(count>10000) return path;
    }

    
    extract_pose_path(current_node, distances, path, start_node, path.utime);
    //path.path.push_back(start);    
    path.path_length = path.path.size();
    return path;
}

void expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params, NodeList& closed_list, NodeList& searched_list, PriorityQueue& open_list, Node& goal_node){
    const int xDeltas[8] = {1, 1, 0, 1, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1, 1, 1, 0, -1};

    for(int i=0; i<8; i++){
        cell_t cell(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);

        Node* neighbor;
        if(searched_list.is_member(cell))
            neighbor = searched_list.get(cell);
        else
            neighbor = new Node(cell.x, cell.y);

        if (!closed_list.is_member(neighbor->cell) && neighbor->is_in_map(distances) && !neighbor->is_obstacle(distances, params.minDistanceToObstacle)) {
            if (!searched_list.is_member(neighbor->cell)) {
                neighbor->g_cost = g_cost(neighbor, node);
                neighbor->h_cost = h_cost(neighbor, &goal_node);
                neighbor->parent = node;
                open_list.push(neighbor);
                searched_list.put(neighbor);
            }
            else if (neighbor->g_cost > g_cost(node, neighbor)) {
                neighbor->g_cost = g_cost(node, neighbor);
                neighbor->parent = node;
                open_list.push(neighbor);
            }
        }

    }
}

void extract_pose_path(Node* node, const ObstacleDistanceGrid& distances, robot_path_t& path, Node& start_node, int64_t utime){
    Node* current_node = node;
    while(!(*current_node == start_node)){
        Point<double> temp;
        temp.x = current_node->cell.x;
        temp.y = current_node->cell.y;

        
        Point<double> temp2 = grid_position_to_global_position(temp, distances);
        
        pose_xyt_t pose;
        pose.utime = utime;
        pose.x = temp2.x;
        pose.y = temp2.y;
        // std::cout<<pose.x<< " "<<pose.y<<std::endl;
        pose.theta = 0.0;
        path.path.push_back(pose);
        current_node = current_node->parent;
    }
    std::reverse(path.path.begin(), path.path.end());
}