/*******************************************************************************
 * File:        imomt_rrt_star.h
 * 
 * Author:      Dongmyeong Lee (dongmyeong[at]umich.edu)
 * Created:     02/20/2022
 * 
 * Description: Informable Multi-Objective and Multi-Directiobal RRT*
*******************************************************************************/
#ifndef IMOMT_RRT_STAR_H
#define IMOMT_RRT_STAR_H

#include <iostream>
#include <set>
#include <unordered_set>
#include <vector>
#include <list>
#include <unordered_map>
#include <queue>
#include <memory>
#include <math.h>
#include <random>
#include <iterator>
#include <algorithm>
#include <numeric>
#include <pthread.h>

#include "../location.h"
#include "../utils/debugger.h"

typedef struct tree
{
    size_t id;
    size_t root;
    std::unordered_set<int> nodes;
    std::unordered_map<int, int> parent;
    std::unordered_map<int, double> cost;
    std::unordered_map<int, std::unordered_set<int>> children;

    // Constructor
    tree() = default;
    tree(int id, int root): id(id), root(root)
    {
        bipedlab::debugger::debugColorOutput("Construct Tree rooted at : ", root, 1, BG);
        nodes = {root};
        parent = { {root, root} };
        children = { {root, {}} };
        cost = { {root, 0} };
    }

    bool checkExistenceNode(int node) const
    {
        return nodes.count(node);
    }

    /**
     * @brief update the cost(path from root of tree) for node and its nested children
     * 
     * @param node node which its parent has changed
     * @param new_cost new path cost from root of tree
     */
    void updateCost(int rewired_node, double new_cost)
    {
        std::queue<int> nodes_need_cost_update;
        nodes_need_cost_update.push(rewired_node);

        while (!nodes_need_cost_update.empty())
        {
            int parent = nodes_need_cost_update.front();
            nodes_need_cost_update.pop();

            for (int child : children[parent])
            {
                cost[child] = new_cost + cost[child] - cost[rewired_node];
                nodes_need_cost_update.push(child);
            }
        }
        cost[rewired_node] = new_cost;
    }
} tree_t;

class ImomtRRT
{
public:
    ImomtRRT() = default;
    ImomtRRT(int source, int target, std::set<int> objectives, 
        std::shared_ptr<std::vector<location_t>> map,
        std::shared_ptr<std::vector<std::vector<std::pair<int, double>>>> connection);
    
    static void* findShortestPath(void* arg);
    static void* printPath(void* arg);

private:
    static const int MAX_ITER_ = 1000000;
    static const int GOAL_BIAS_ = 0.2;

    int source_tree_id_;
    int target_tree_id_;
    std::vector<int> destinations_;

    std::shared_ptr<std::vector<location_t>> map_ptr_;
    std::shared_ptr<std::vector<std::vector<std::pair<int, double>>>> connection_ptr_;

    std::vector<tree> tree_layers_;

    std::vector<std::vector<std::pair<double, int>>> adjacency_matrix_;

    std::unordered_map<int, int> union_status_;
    bool is_complete_union_;

    std::list<int> sequence_of_tree_id_rtsp_;
    std::list<int> sequence_of_tree_id_dijkstra_;

    std::vector<location_t> shortest_path_rtsp_;
    std::vector<location_t> shortest_path_dijkstra_;

    bool is_path_updated_;
    bool is_adjacency_updated_;
    pthread_cond_t cond_get_path_;
    pthread_mutex_t lock_;

    void expandTreeLayers_();
    void expandTree_(tree& tree, int random_point);
    int selectRandomVertex_();
    int searchNearestNode_(tree& tree, int random_point);
    int steerNewNode_(tree& tree, int x_nearest, int random_point);
    
    /**
     * @brief Connecting the node(x_min) of tree that makes the shorest cost for x_new
     * 
     * @param tree expanding tree rooted at one of {start, target, objectives}
     * @param x_nearest the closest node of tree from random_point
     * @param x_new the vertex that want to connect from tree
     * @return the ID of x_min(the parent of x_new)
     */
    int connectNewNode_(tree& tree, int x_nearest, int x_new);
    
    /**
     * @brief Rewiring Tree by finding the nodes that could have a shorter path when
     * it connected with x_new.
     * Updating the cost of nested children of rewired node.
     * 
     * @param tree expanding tree rooted at one of {start, target, objectives}
     * @param x_new the node that newly connected on tree
     * @param x_min the parent of x_new
     */
    void rewireTree_(tree& tree, int x_new, int x_min);

    bool solveRTSP_();

    bool solveDijkstra_();

    bool updatePath_();

    std::pair<double, std::pair<int, int>> bestInsertion_(int tree_id);

    void updateConnectivityTree_(const tree& tree, int x_new);

    bool connectTwoTree_(const tree& tree1, const tree& tree2);

    bool checkCompleteUnion_();

    /**
     * @brief Update the status(union_status) of connectivity of tree by setting 
     * the value of union_status as the minimum ID of root between roots of 
     * connected trees
     */
    void updateUnionStatus_();

    double computeGreatCircleDistance_(int node1, int node2);
    double getDistanceFromConnection_(int node1, int node2);

    double getDistanceFromLatLon_(double lat1, double lon1, double lat2, double lon2);
    void extractSequence_(const std::unordered_map<int, int>& parent, int source,
                           int target, std::list<int>& sequence);
    void insertInPlace(std::list<int>& sequence, int value, int right_idx);
    void insertInSequence(std::list<int>& sequence, int value, int right_idx);

    void printDijkstraId();
    void printAdjacentMatrix();
    void printRTSPId();
};
#endif /* IMOMT_RRT_STAR_H */
