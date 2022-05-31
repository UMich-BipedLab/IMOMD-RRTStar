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
 * File:        ana_star.h
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     04/23/2022
 * 
 * Description: Expand from destinations separately in ANA* way.
 *              Each tree grows until it connects to all trees.
*******************************************************************************/
#ifndef ANA_STAR_H
#define ANA_STAR_H

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
#include <cfloat>

#include "data/location_t.h"
#include "setting/imomd_setting_t.h"

#include "imomd_rrt_star/eci_gen_tsp_solver.h"

#include "osm_converter/compute_haversine.hpp"

#include "utils/debugger.h"
#include "utils/csv.h"
#include "utils/utils.h"
#include "utils/timing.h"

typedef struct tree_ana_star
{
    int id;
    int start_id;
    int goal_id;

    size_t start_root;
    size_t goal_root;

    std::unordered_map<size_t, size_t> parent;
    std::unordered_map<size_t, double> cost;

    std::priority_queue<std::pair<double, size_t>> open_queue;

    double G;
    double E;

    // Constructor
    tree_ana_star() = default;
    tree_ana_star(int start_id, int goal_id, size_t start_root, size_t goal_root)
        :start_id(start_id), goal_id(goal_id), 
         start_root(start_root), goal_root(goal_root)
    {
        bipedlab::debugger::debugColorOutput("Construct Tree rooted at : ", start_root, 8, BG);
        bipedlab::debugger::debugColorOutput("and to goal : ", goal_root, 8, BG);

        parent = { {start_root, start_root} };
        cost = { {start_root, 0} };

        G = INT_MAX;
        E = INT_MAX;

        open_queue.push(std::make_pair(E, start_root));
    }
} tree_ana_star_t;

class ANAStar
{
public:
    ANAStar() = default;
    ANAStar(size_t source, size_t target, std::vector<size_t> objectives, 
        const std::shared_ptr<std::vector<location_t>> map_ptr,
        const std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> connection_ptr,
        const imomd_setting_t& setting);
    
    static void* findShortestPath(void* arg);
    std::shared_ptr<std::vector<size_t>> getShortestPath();
    static void* printPath(void* arg);

    std::shared_ptr<std::vector<std::vector<double>>> getDistanceMatrix();

private:
    imomd_setting_t setting_;
    
    int iteration_count_ = 0;
    std::mt19937 random_gen_;

    std::shared_ptr<std::vector<location_t>> map_ptr_;
    std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> connection_ptr_;

    std::vector<size_t> destinations_; // root_node_id

    int source_tree_id_;
    int target_tree_id_;
    std::vector<tree_ana_star_t> tree_layers_;

    std::unordered_set<int> unexplored_tree_;

    std::vector<std::vector<double>> distance_matrix_;

    std::vector<int> disjoint_set_parent_;
    std::unordered_map<int, std::vector<int>> disjoint_set_children_;
    bool is_connected_graph_;

    EciGenSolver eci_gen_solver_;
    std::shared_ptr<std::vector<int>> sequence_of_tree_id_rtsp_;

    double shortest_path_cost_;
    std::vector<size_t> shortest_path_;

    bool is_distance_matrix_updated_;
    bool is_path_updated_;
    bool reached_max_iter_;
    pthread_cond_t cond_get_path_;
    pthread_mutex_t lock_;

    // Log
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_log_time_;
    CSVFile command_csv_;

    // Expand all tree alternatively
    void expandTreeLayers_();

    // Expand each tree
    void expandTree_(tree_ana_star_t& tree);

    // Update connection by checking whether x_nconew connect tree and other_tree
    // Update Distance Matrix & Connection Nodes
    void updateConnectionTree_(const tree_ana_star_t& tree, size_t x_new);

    // Connect two tree and check trees are forming connected graphs
    void connectTwoTree_(int tree1_id, int tree2_id);

    bool updatePath_();

    void solveRTSP_();

    void logData_();
};

#endif /* ANA_STAR_H */
