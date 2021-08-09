#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates and
    // store the nodes in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Calculate Heuristic (distance of a node to the end node)

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Using the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();
    for(auto node: current_node->neighbors) {
        // For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
        node->parent = current_node;
        node->g_value = node->distance(*current_node) + current_node->g_value;
        node->h_value = CalculateHValue(node);
        // For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
        node->visited = true;
        open_list.push_back(node);
    }
}

// Sort the open list according to the f-value (sum of the h-value and g-value)
void RoutePlanner::NodeSort() {
    sort(open_list.begin(),open_list.end(), [] (const RouteModel::Node *node,const RouteModel::Node *other) 
    {float f1 = node->g_value + node->h_value;
    float f2 = other->g_value + other->h_value;
    return f1 > f2;});
}

RouteModel::Node *RoutePlanner::NextNode() {
    this->NodeSort();
    // Return pointer to the node in the list with lowest f-value and remove it from the open_list
    RouteModel::Node *current = open_list.back();
    open_list.pop_back();
    return current;


}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // This method takes the current (final) node as an argument and iteratively follows the 
    // chain of parents of nodes until the starting node is found.
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    bool done = false;
    RouteModel::Node *node = current_node;
    while (node != start_node) {
            path_found.push_back(*node);
            // For each node in the chain, add the distance from the node to its parent to the distance variable.
            distance += node->distance(*(node->parent));
            node = node->parent;
    }
    // The returned vector should be in the correct order: the start node should be the first element
    // of the vector, the end node should be the last element.
    path_found.push_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

// A* Search algorithm
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    open_list.push_back(start_node);
    while (!open_list.empty()) {
        // Sort the open_list and return the next node
        current_node = this->NextNode();
        if (current_node == end_node) {
            // When the search has reached the end_node, the ConstructFinalPath method will return the final path that was found.
            // Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // Add all of the neighbors of the current node to the open_list.
        this->AddNeighbors(current_node);
    }
}