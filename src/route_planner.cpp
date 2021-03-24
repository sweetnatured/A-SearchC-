#include "route_planner.h"
#include <algorithm>
#include <new>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    m_Model = model;

    RouteModel::Node* start = new RouteModel::Node;
    start = &model.FindClosestNode(start_x, start_y);
    start_node = start;
    RouteModel::Node* end = new RouteModel::Node;
    end = &model.FindClosestNode(end_x, end_y);
    end_node = end;
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float dist = node->distance(*end_node);
    return dist;
}



void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor:(current_node->neighbors)) {
        // if (neighbor->visited == false) {
        float n_h_value = CalculateHValue(neighbor);
        float n_g_value = current_node->g_value + current_node->distance(*neighbor);

        if (n_h_value + n_g_value < neighbor->h_value + neighbor->g_value) {    
            neighbor->parent = current_node;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
            open_list.push_back(neighbor);
            neighbor->visited = true;
        }
        // } 
    }
    // current_node->visited = true;
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort (open_list.begin(), open_list.end(), [] (const auto &lhs, const auto &rhs) {
        return (lhs->h_value + lhs->g_value > rhs->h_value + rhs->g_value); 
    });
    auto current = open_list.back();
    open_list.pop_back();
    // current->visited = true;
    return current;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while(current_node->parent) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    // std::cout << "Distance = " << distance << "\n";
    // std::cout << "Path = " << path_found.size() << "\n";
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;

    // TODO: Implement your solution here.

    // open_list.push_back(start_node);
    start_node->visited = true;
    while (open_list.size() > 0 || current_node == start_node) {
        AddNeighbors(current_node);
        // std::cout << "After adding size = " << open_list.size() << "\n";
        current_node = NextNode();
        if (current_node->x == end_node->x && current_node->y == end_node->y) {
            // std::cout << "123 Path size = " << m_Model.path.size() << "\n";
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
    }
   
}
