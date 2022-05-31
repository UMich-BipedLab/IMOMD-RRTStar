/* Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 */
/*******************************************************************************
 * File:        imomd_rrt_star.h
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     02/20/2022
 * 
 * Description: Informable Multi-Objective and Multi-Directiobal RRT*
*******************************************************************************/
#ifndef IMOMD_RRT_STAR_H
#define IMOMD_RRT_STAR_H

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

#include "data/location_t.h"
#include "setting/imomd_setting_t.h"

#include "osm_converter/compute_haversine.hpp"
#include "osm_converter/compute_bearing.hpp"

#include "eci_gen_tsp_solver.h"

#include "utils/debugger.h"
#include "utils/csv.h"
#include "utils/utils.h"
#include "utils/timing.h"

typedef struct tree
{
    int id;
    size_t root;

    std::unordered_map<size_t, size_t> parent;
    std::unordered_map<size_t, std::unordered_set<size_t>> children;
    std::unordered_map<size_t, double> cost;

    std::unordered_set<size_t> expandables;

    bool is_done;

    // Constructor
    tree() = default;
    tree(int id, size_t root)
        :id(id), root(root), is_done(false)
    {
        bipedlab::debugger::debugColorOutput("Construct Tree rooted at : ", root, 1, BG);
        parent = { {root, root} };
        children = { {root, {}} };
        cost = { {root, 0} };
    }

    // Check whether the node is explored
    bool checkVisitedNode(size_t node) const
    {
        return (parent.find(node) != parent.end());
    }

    bool operator==(const tree& rhs) const
    {
        return id == rhs.id;
    }

    // update the cost(path from root of tree) for node and its nested children
    std::vector<size_t> updateCost(size_t rewired_node, double new_cost)
    {
        std::vector<size_t> updated_nodes;
        std::queue<size_t> nodes_need_cost_update;
        nodes_need_cost_update.push(rewired_node);

        while (!nodes_need_cost_update.empty())
        {
            size_t parent = nodes_need_cost_update.front();
            updated_nodes.push_back(parent);
            nodes_need_cost_update.pop();

            for (size_t child : children[parent])
            {
                cost[child] = new_cost + cost[child] - cost[rewired_node];
                nodes_need_cost_update.push(child);
            }
        }
        cost[rewired_node] = new_cost;

        return updated_nodes;
    }
} tree_t;

class ImomdRRT
{
public:
    ImomdRRT() = default;
    ImomdRRT(size_t source, size_t target, std::vector<size_t> objectives, 
        const std::shared_ptr<std::vector<location_t>> map,
        const std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> connection,
        const imomd_setting_t& setting);
    
    static void* findShortestPath(void* arg);
    static void* printPath(void* arg);

    std::shared_ptr<std::vector<std::vector<double>>> getDistanceMatrix();

private:
    imomd_setting_t setting_; 

    int iteration_count_ = 0;
    std::mt19937 random_gen_;

    // nodes of map 
    std::shared_ptr<std::vector<location_t>> map_ptr_;
    // edges of map
    std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> connection_ptr_;
    // idx of destinations in the map
    std::vector<size_t> destinations_; 

    int source_tree_id_;
    int target_tree_id_;
    std::vector<tree> tree_layers_;

    // Probability of Selecting Destinations as Random Point
    // idx of Row represents exploring tree & idx of Column represents destinations to steer
    std::vector<std::vector<double>> probability_matrix_;
    
    // Symetric Matrix
    // Matrix of {node_id, heuristic_distance}, 
    // which have minimum heuristice distance for two destinations
    // Heuristic Distance = Dist(destination_1, node) + Dist(destination_2, node)
    std::vector<std::vector<std::pair<size_t, double>>> expandables_min_heuristic_matrix_;
    // Distance Matrix for each pair of destinations
    std::vector<std::vector<double>> distance_matrix_;
    // Matrix of connection nodes that have the shortest path for each pair of destinations
    std::vector<std::vector<size_t>> connection_node_matrix_;

    // idx : i*(i-1)/2 + j when (i > j)
    // save only lower matrix
    std::vector<std::unique_ptr<std::unordered_set<size_t>>> connection_nodes_set_;

    // Disjoint Set sructure for check connectivity
    std::vector<int> disjoint_set_parent_;
    std::unordered_map<int, std::vector<int>> disjoint_set_children_;
    bool is_connected_graph_;

    // RTSP Solver
    EciGenSolver eci_gen_solver_;
    // Result of RTSP Problem
    std::shared_ptr<std::vector<int>> sequence_of_tree_id_rtsp_;
    
    // Anytime result From IMOMD-RRT* System
    double shortest_path_cost_;
    std::vector<size_t> shortest_path_;

    // Pthread signal values and mutex
    bool is_distance_matrix_updated_;
    bool is_merge_done_;
    bool is_path_updated_;
    bool is_computation_finished_;
    pthread_cond_t cond_get_path_;
    pthread_mutex_t lock_;

    // Log
    std::chrono::steady_clock::time_point start_time_;
    CSVFile command_csv_;

    // Expand All Tree
    void expandTreeLayers_();

    // Expand Tree with 5 main step
    void expandTree_(tree_t& tree);

    // Sample Random Node in the given graph
    location_t selectRandomVertex_(tree_t& tree);

    // Choose the closest expandable node x_new to the x_rand
    size_t steerNewNode_(tree_t& tree, location_t& random_point);

    // Update expandable nodes of tree
    void updateExpandables(tree_t& tree, size_t x_new, bool search_neighber);
    
    // Connect the node x_new to the parent that results in the smallest cost-to-come.
    void connectNewNode_(tree_t& tree, size_t x_new);
    
    // Rewiring Tree by finding the nodes that could have a shorter path when
    // it connected with x_new.
    void rewireTree_(tree_t& tree, size_t x_new);

    // Update Selection Probability by comparing minimum heuristic and distance matrix
    void updateSelectionProbability(tree_t& tree, location_t& random_point);

    // Update connection by checking whether x_new connect tree and other_tree
    // Update Distance Matrix & Connection Nodes
    // Update Expandables
    void updateConnectionTree_(tree_t& tree, size_t x_new);

    // Connect two tree and check trees are forming connected graphs
    void connectTwoTree_(const tree_t& tree1, const tree_t& tree2);

    // Merge trees rooted at pseudo-destinations after forming connected graph
    void mergePseudoTrees_();

    // Merge child_tree into parent_tree
    void mergeTwoTree_(tree_t& parent_tree, tree_t& child_tree);

    // Update the path with the visiting sequence of destinations resulted from RTSP Solver
    void updatePath_();

    // Solve RTSP Probelm with Distance Matrix
    // When it is Pseudo-Mode, we solve Dijkstra Problem.
    void solveRTSP_();

    // Save Elapsed time, Path Cost, Tree size to CSV file
    void logData_();
};
#endif /* IMOMD_RRT_STAR_H */
