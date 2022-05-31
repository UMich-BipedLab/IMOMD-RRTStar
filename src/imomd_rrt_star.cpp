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
 * File:        imomd_rrt_star.cpp
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     02/20/2022
 * 
 * Description: Informable Multi-Objective and Multi-Directiobal RRT*
*******************************************************************************/
#include <iostream>

#include "imomd_rrt_star/imomd_rrt_star.h"

ImomdRRT::ImomdRRT(size_t source, size_t target, std::vector<size_t> objectives, 
    const std::shared_ptr<std::vector<location_t>> map_ptr,
    const std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> connection_ptr,
    const imomd_setting_t& setting)
    : setting_(setting), map_ptr_(map_ptr), connection_ptr_(connection_ptr),
      is_connected_graph_(false), is_distance_matrix_updated_(false),
      is_merge_done_(false), is_path_updated_(false), is_computation_finished_(false),
      cond_get_path_(PTHREAD_COND_INITIALIZER), lock_(PTHREAD_MUTEX_INITIALIZER)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Constructing IMOMD-RRT*", 10, BC);

    destinations_ = {source};
    for (const size_t objective : objectives)
    {
        destinations_.push_back(objective);
    }
    destinations_.push_back(target);

    int dest_count = destinations_.size();
    tree_layers_.resize(dest_count);
    disjoint_set_parent_.resize(dest_count);

    probability_matrix_.resize(dest_count, std::vector<double>(dest_count, 0.0));
    std::vector<std::vector<std::pair<double, int>>> bearing_matrix(dest_count);
    std::vector<std::vector<double>> inversed_haversine_matrix(
        dest_count, std::vector<double>(dest_count, 0.0));
    std::vector<double> sum_haversine(dest_count, 0.0);

    expandables_min_heuristic_matrix_.resize(dest_count, 
        std::vector<std::pair<size_t, double>>(dest_count, {-1, INFINITY}));

    distance_matrix_.resize(dest_count, std::vector<double>(dest_count, 0.0));
    connection_node_matrix_.resize(dest_count, std::vector<size_t>(dest_count, -1));
    
    for (int i = 0; i < dest_count; ++i)
    {
        // Initialize Trees rooted at each destinations
        size_t root = destinations_[i];
        tree_layers_[i] = tree_t(i, root);
        updateExpandables(tree_layers_[i], root, true);

        // Initialize Disjoint-set Data Structure
        disjoint_set_parent_[i] = i;
        disjoint_set_children_[i] = {};

        // Initialize Distance Matrix & Connection Nodes of destinations
        for (int j = i + 1; j < dest_count; ++j)
        {
            // Calculate Bearing and Haversine Distance to dertmine the
            // probability of selecting random points
            bearing_matrix[i].push_back(
                {computeBearing((*map_ptr)[destinations_[i]], (*map_ptr)[destinations_[j]]), j});

            bearing_matrix[j].push_back(
                {computeBearing((*map_ptr)[destinations_[j]], (*map_ptr)[destinations_[i]]), i});
            
            inversed_haversine_matrix[i][j] = 1.0 /
                computeHaversineDistance((*map_ptr)[root], (*map_ptr)[destinations_[j]]);
            inversed_haversine_matrix[j][i] = inversed_haversine_matrix[i][j];

            sum_haversine[i] += inversed_haversine_matrix[i][j];
            sum_haversine[j] += inversed_haversine_matrix[j][i];

            // Initialize Distance Matrix
            distance_matrix_[i][j] = INFINITY;
            distance_matrix_[j][i] = INFINITY;

            // Initialize Connection Nodes of destinations
            connection_node_matrix_[i][j] = -1;
            connection_node_matrix_[j][i] = -1;

            // Initialize the set of connection nodes for two tree
            // Totally, (n Choose 2) number of set
            connection_nodes_set_.emplace_back(new std::unordered_set<size_t>);
        }
    }

    // Set source and target tree_id as first and last idx of tree_layers, respectively
    source_tree_id_ = 0;
    target_tree_id_ = dest_count - 1;

    // Construct Matrix of Probability of Sampling Destinations
    // Calculate Probability with Bearing
    // The probability of Selection become larger when ther's not much destinations
    // in that direction.
    for (int i = 0; i < dest_count && dest_count > 3; ++i)
    {
        // Sort destinations counter-clockwisely
        std::sort(bearing_matrix[i].begin(), bearing_matrix[i].end());

        double sum_bearing = 0.0;
        for (int j = 0; j < dest_count - 1; ++j)
        {
            double curr_angle = bearing_matrix[i][j].first;
            double prev_angle = bearing_matrix[i][(j - 2 + dest_count) % (dest_count - 1)].first;
            double next_angle = bearing_matrix[i][(j + 1) % (dest_count - 1)].first;

            double angle_diff_prev = curr_angle - prev_angle;
                angle_diff_prev += angle_diff_prev < 0 ? 2* M_PI : 0;

            double angle_diff_next = next_angle - curr_angle;
                angle_diff_next += angle_diff_next < 0 ? 2* M_PI : 0;

            // Probability of selection is propotional to sum of the angles
            // between two destinations on each side
            int idx = bearing_matrix[i][j].second;
            probability_matrix_[i][idx] = angle_diff_prev + angle_diff_next;
            sum_bearing += probability_matrix_[i][idx];
        }

        for (int j = 0; j < dest_count; ++j)
        {
            probability_matrix_[i][j] /= sum_bearing;
        }
    }

    // Calculate Probability with Haversine Distance
    // Probability of selection is inversely propotional to the haversine distace
    for (int i = 0; i < dest_count; ++i)
    {
        for (int j = 0; j < dest_count; ++j)
        {
            probability_matrix_[i][j] += inversed_haversine_matrix[i][j] / sum_haversine[i];
        }
    }

    // Balance two Probability of selecting destinations
    // Example:
    // d1(exploring tree) -> d2
    // d2(exploring tree) -> d1
    std::vector<double> sum_probability(dest_count, 0.0);
    for (int i = 0; i < dest_count; ++i)
    {   
        for (int j = i + 1; j < dest_count; ++j)
        {
            probability_matrix_[i][j] += probability_matrix_[j][i];
            probability_matrix_[j][i] = probability_matrix_[i][j];

            sum_probability[i] += probability_matrix_[i][j];
            sum_probability[j] += probability_matrix_[j][i];
        }
        
        for (int j = 0; j < dest_count; ++j)
        {
            probability_matrix_[i][j] /= sum_probability[i];
        }
    }
    
    // Construct RTSP solver
    eci_gen_solver_ = EciGenSolver(setting.rtsp_setting);
    shortest_path_ = {};
    shortest_path_cost_ = INFINITY;

    // Initialize Random genrator
    if (setting_.random_seed)
    {
        std::random_device rd;
        random_gen_ = std::mt19937{rd()};
    }
    else
    {
        random_gen_ = std::mt19937{0};
    }

    // Logging data
    start_time_ = bipedlab::timing::getCurrentTime();
    if (setting_.log_data)
    {
        try
        {
            std::string date = bipedlab::utils::getTimeNDate();
            std::string path = bipedlab::utils::getCurrentDirectory();

            command_csv_.open(std::string(path) + "/../../../src/IMOMD-RRT/experiments/IMOMD_" + 
                              date + "_command_history.csv");

            // Header
            command_csv_ << "CPU_time" << "path_cost" << "tree_size" << endrow;
        }
        catch (const std::exception& ex)
        {
            std::cout << "Exception was thrown: " << ex.what() << std::endl;
        }
    }
}

void* ImomdRRT::findShortestPath(void* arg)
{   
    ImomdRRT* this_thread = static_cast<ImomdRRT*>(arg);

    bool is_done_iter = false;
    bool is_done_time = false;
    bool is_done_expansion = false;
    while (!is_done_iter && !is_done_time && !is_done_expansion)
    {
        this_thread->expandTreeLayers_();
        if (this_thread->iteration_count_ % 100 == 0)
        {
            this_thread->solveRTSP_();

            is_done_expansion = true;
            for (const tree_t& tree : this_thread->tree_layers_)
            {
                if (!tree.is_done)
                {
                    is_done_expansion = false;
                }
            }
        }

        bipedlab::debugger::debugColorOutput("[IMOMD] count : ", 
                this_thread->iteration_count_, 3);
        
        if (this_thread->iteration_count_++ > this_thread->setting_.max_iter)
        {
            is_done_iter = true;
        }
        else if (bipedlab::timing::spendElapsedTime(this_thread->start_time_) > 
            this_thread->setting_.max_time)
        {
            is_done_time = true;
        }
    }
    
    if (is_done_iter)
    {
        bipedlab::debugger::debugTitleTextOutput("[IMOMD]", "Reached Max Iteration", 10, BM);
    }
    else if (is_done_time)
    {
        bipedlab::debugger::debugTitleTextOutput("[IMOMD]", "Reached Max Time", 10, BM);
    }

    if (is_done_expansion)
    {
        bipedlab::debugger::debugTitleTextOutput("[IMOMD]", "Tree Exploration Done", 10, BM);
    }
    
    int count = 0;
    while (count++ < 10)
    {
        this_thread->solveRTSP_();
    }
    
    pthread_mutex_lock(&this_thread->lock_);
    this_thread->is_computation_finished_ = true;
    pthread_mutex_unlock(&this_thread->lock_);
    pthread_cond_signal(&this_thread->cond_get_path_);

    pthread_exit(NULL);
}

std::shared_ptr<std::vector<size_t>> ImomdRRT::getShortestPath()
{
    pthread_mutex_lock(&lock_);
    
    // Wait for path update or termination of getShortestPath
    while (!is_path_updated_ && !is_computation_finished_)
    {  
        pthread_cond_wait(&cond_get_path_, &lock_);
        bipedlab::debugger::debugTitleTextOutput("[main]", "Find new path", 9, BG);
    }

    auto shortest_path_ptr = std::make_shared<std::vector<size_t>>(shortest_path_);
    is_path_updated_ = false;

    pthread_mutex_unlock(&lock_);

    return shortest_path_ptr;
}

std::shared_ptr<std::unordered_map<size_t, size_t>> ImomdRRT::getTreeBranch(int tree_id)
{
    pthread_mutex_lock(&lock_);
    std::unordered_map<size_t, size_t> tree_branch;

    tree_branch = tree_layers_[tree_id].parent;
    pthread_mutex_unlock(&lock_);

    return std::make_shared<std::unordered_map<size_t, size_t>>(tree_branch);
}

std::vector<size_t> ImomdRRT::getShortSubPath(int tree1_id, int tree2_id)
{
    std::vector<size_t> sub_path;

    pthread_mutex_lock(&lock_);
    int connection_set_idx = (tree1_id > tree2_id) ? tree1_id * (tree1_id - 1) / 2 + tree2_id : 
                                                     tree2_id * (tree2_id - 1) / 2 + tree1_id;

    if (!(*(connection_nodes_set_[connection_set_idx])).empty())
    {
        // tree1 ----- connection_node ------ tree2
        size_t connection_node = connection_node_matrix_[tree1_id][tree2_id];
        tree_t& tree1 = tree_layers_[tree1_id];
        tree_t& tree2 = tree_layers_[tree2_id];

        // connection_node(exclude) ---> tree1.root(include)
        size_t node = connection_node;
        while (node != tree1.root)
        {
            node = tree1.parent[node];
            sub_path.push_back((*map_ptr_)[node].id);
        }

        // tree1.root(include) ---> connection_node(exclude) 
        std::reverse(sub_path.begin(), sub_path.end());

        // connection_node(include) ---> tree2.root(exclude) 
        node = connection_node;
        while (node != tree2.root)
        {
            sub_path.push_back((*map_ptr_)[node].id);
            node = tree2.parent[node];
        }
    }
    pthread_mutex_unlock(&lock_);

    return sub_path;
}

void* ImomdRRT::printPath(void* arg)
{
    ImomdRRT* this_thread = static_cast<ImomdRRT*>(arg);

    int count = 0;
    while (!this_thread->is_computation_finished_)
    {
        pthread_mutex_lock(&this_thread->lock_);

        // Wait for path update or termination of getShortestPath
        while (!this_thread->is_path_updated_ && !this_thread->is_computation_finished_)
        {  
            pthread_cond_wait(&this_thread->cond_get_path_, &this_thread->lock_);
        }

        // Export the shortest path
        bipedlab::debugger::debugTitleTextOutput("[main]", 
                "print path: " + std::to_string(count++), 10, BG);
        for (auto node : this_thread->shortest_path_)
        {
            std::cout << node << " -> ";
        }
        std::cout << "#" << std::endl;

        this_thread->is_path_updated_ = false;
        pthread_mutex_unlock(&this_thread->lock_);
    }
    
    pthread_exit(NULL);
}

bool ImomdRRT::checkComputationFinished()
{
    return is_computation_finished_;
}

std::shared_ptr<std::vector<std::vector<double>>> ImomdRRT::getDistanceMatrix()
{
    return std::make_shared<std::vector<std::vector<double>>>(distance_matrix_);
};

void ImomdRRT::expandTreeLayers_()
{
    for (auto& tree : tree_layers_)
    {   
        expandTree_(tree);
    }
}

void ImomdRRT::expandTree_(tree_t& tree)
{
    if (tree.expandables.empty() || tree.is_done)
    {
        return;
    }

    bipedlab::debugger::debugColorOutput("[IMOMD] Expand Tree : ", tree.root, 3, BG);
    bipedlab::debugger::debugColorOutput("[IMOMD] Total Nodes : ", tree.parent.size(), 1, BK);
    bipedlab::debugger::debugColorOutput("[IMOMD] Expandable Nodes : ", tree.expandables.size(), 1, BK);

    location_t random_point = selectRandomVertex_(tree);
    size_t x_new = steerNewNode_(tree, random_point);
    connectNewNode_(tree, x_new);
    updateExpandables(tree, x_new, true);
    rewireTree_(tree, x_new);

    updateSelectionProbability(tree, random_point);
    updateConnectionTree_(tree, x_new);
}

location_t ImomdRRT::selectRandomVertex_(tree_t& tree)
{
    static std::uniform_real_distribution<> dist(0, 1);
    static std::uniform_real_distribution<> ratio_dist(0.0, 0.5);
    static std::uniform_int_distribution<size_t> uni_map(0, (*map_ptr_).size() - 1);
    
    location_t random_point;
    if (dist(random_gen_) < setting_.goal_bias)
    {
        // Select Random Points between destinations based on probability
        double rand = dist(random_gen_);
        int i = 0;
        while (rand >= 0)
        {
            rand -= probability_matrix_[tree.id][i++];
        }
        size_t random_dest = tree_layers_[i - 1].root;

        random_point = (*map_ptr_)[random_dest];

        // Select random location between root of tree and random_point
        if (tree.checkVisitedNode(random_dest))
        {
            double ratio = ratio_dist(random_gen_);

            random_point.id = random_dest;
            random_point.latitude *= ratio;
            random_point.longitude *= ratio;

            // Ratio Randomly selected between 0.0 ~ 0.5
            // Try to avoid tree overlaped, which rarely provide useful connection nodes
            random_point.latitude += (1.0 - ratio) * (*map_ptr_)[tree.root].latitude;
            random_point.longitude += (1.0 - ratio) * (*map_ptr_)[tree.root].longitude;
        }
    }
    else
    {
        random_point = (*map_ptr_)[uni_map(random_gen_)];
    }

    bipedlab::debugger::debugColorOutput("[IMOMD] Random Point(Graph) : ", random_point.id, 1, BW);
    return random_point;
}

size_t ImomdRRT::steerNewNode_(tree_t& tree, location_t& random_point)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Steer New Node", 2, BC);

    size_t x_new;
    double min_distance = INFINITY;
    // Find x_new that minimizes the Dist(x, x_rand)
    for (const auto& expandable : tree.expandables)
    {
        double distance = computeHaversineDistance((*map_ptr_)[expandable], random_point);

        if (distance < min_distance)
        {
            min_distance = distance;
            x_new = expandable;
        }
    }

    // Jumping Point Search
    // Jump until it meet intersection or already visited nodes
    if ((*connection_ptr_)[x_new].size() == 2)
    {
        updateExpandables(tree, x_new, false);
    }

    bool is_jps_done = false;
    while ((*connection_ptr_)[x_new].size() == 2 && !is_jps_done)
    {
        auto it = (*connection_ptr_)[x_new].begin();
        size_t x_1 = (*it).first;
        size_t x_2 = (*(++it)).first;

        if (tree.parent.find(x_1) != tree.parent.end())
        {
            // x_1 (visited) --> x_new --> x_2 (visited)
            if (tree.parent.find(x_2) != tree.parent.end())
            {
                is_jps_done = true;
            }
            else // x_1 (visited) --> x_new --> x_2 (not visited)
            {
                tree.parent[x_new] = x_1;
                tree.children[x_1] = {x_new};
                tree.cost[x_new] = tree.cost[x_1] + (*connection_ptr_)[x_1][x_new];
                updateConnectionTree_(tree, x_new);
                x_new = x_2;
            }
        }
        // x_2 (visited) --> x_new --> x_1 (not visited)
        else
        {
            tree.parent[x_new] = x_2;
            tree.children[x_2] = {x_new};
            tree.cost[x_new] = tree.cost[x_2] + (*connection_ptr_)[x_2][x_new];
            updateConnectionTree_(tree, x_new);
            x_new = x_1;
        }
    }

    bipedlab::debugger::debugColorOutput("[IMOMD] x_new : ", x_new, 1, BW);
    return x_new;
}

void ImomdRRT::connectNewNode_(tree_t& tree, size_t x_new)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Connect New Node", 2, BC);
    
    // Select x_parent of x_new that make x_new have the lowest cost-to-come
    size_t x_parent;
    double min_cost_x_new = INFINITY;
    bool is_connectable = false;
    for (const auto& x_near : (*connection_ptr_)[x_new])
    {
        if (tree.checkVisitedNode(x_near.first))
        {
            double cost_x_new = tree.cost[x_near.first] + x_near.second;
            if (cost_x_new < min_cost_x_new)
            {
                x_parent = x_near.first;
                min_cost_x_new = cost_x_new;
                is_connectable = true;
            }
        }
    }
    
    // Update Tree structure by adding node and edge
    if (is_connectable)
    {
        tree.parent[x_new] = x_parent;
        tree.children[x_parent].insert(x_new);
        tree.cost[x_new] = min_cost_x_new;
    }
    
    bipedlab::debugger::debugColorOutput("[IMOMD] x_parent : ", x_parent, 1, BW);
}

void ImomdRRT::updateExpandables(tree_t& tree, size_t x_new, bool search_neighbor)
{
    tree.expandables.erase(x_new);

    // Update expandables_min_heuristic_matrix_ if x_new is the min expandable
    for (tree_t& other_tree : tree_layers_)
    {
        if (other_tree.id == tree.id)
        {
            continue;
        }

        if (x_new == expandables_min_heuristic_matrix_[tree.id][other_tree.id].first)
        {
            expandables_min_heuristic_matrix_[tree.id][other_tree.id] = {-1, INFINITY};
            for (const auto& expandable : tree.expandables)
            {
                double heuristic_distance = 
                    computeHaversineDistance((*map_ptr_)[expandable], (*map_ptr_)[tree.root]) +
                    computeHaversineDistance((*map_ptr_)[expandable], (*map_ptr_)[other_tree.root]);

                if (heuristic_distance < expandables_min_heuristic_matrix_[tree.id][other_tree.id].second)
                {
                    expandables_min_heuristic_matrix_[tree.id][other_tree.id] = {expandable, heuristic_distance};
                }
            }
        }
    }

    if (!search_neighbor)
    {
        return;
    }

    // Search neighborhood of x_new and update expandables and selection probability
    for (const auto& x_near : (*connection_ptr_)[x_new])
    {
        // unexplored neighbor nodes of x_new are added to expandable nodes
        if (!tree.checkVisitedNode(x_near.first))
        {
            tree.expandables.insert(x_near.first);
            
            // Update expandables_min_heuristic_matrix_
            for (tree_t& other_tree : tree_layers_)
            {
                if (other_tree == tree)
                {
                    continue;
                }

                double heuristic_distace = 
                    computeHaversineDistance((*map_ptr_)[x_near.first], (*map_ptr_)[tree.root]) +
                    computeHaversineDistance((*map_ptr_)[x_near.first], (*map_ptr_)[other_tree.root]);

                if (heuristic_distace < expandables_min_heuristic_matrix_[tree.id][other_tree.id].second)
                {
                    expandables_min_heuristic_matrix_[tree.id][other_tree.id] = {x_near.first, heuristic_distace};
                }              
            }
        }
    }
}

void ImomdRRT::rewireTree_(tree_t& tree, size_t x_new)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Rewiring", 2, BC);
    
    for (const auto& x_near : (*connection_ptr_)[x_new])
    {
        // Search visited neighbors except the parent of x_new
        // Check whether x_new could create shorter path for neighbors
        if (tree.checkVisitedNode(x_near.first) && x_near.first != tree.parent[x_new])
        {
            double new_cost_x_near = tree.cost[x_new] + x_near.second;
            if (tree.cost[x_near.first] > new_cost_x_near)
            {
                bipedlab::debugger::debugColorOutput("[IMOMD] x_near(rewired) : ", 
                                                     x_near.first, 3, BW);

                // Disconnect with x_near and its parent
                // Connect with x_near and x_new
                tree.children[tree.parent[x_near.first]].erase(x_near.first);
                tree.parent[x_near.first] = x_new;
                tree.children[x_new].insert(x_near.first);

                // Update cost of x_near and its nested children
                std::vector<size_t> updated_nodes = tree.updateCost(x_near.first, new_cost_x_near);

                updateConnectionTree_(tree, x_near.first);

                // Rewiring recursively
                for (auto rit = updated_nodes.rbegin(); rit != updated_nodes.rend(); ++rit)
                {
                    if (x_near.first == *rit)
                    {
                        continue;
                    }
                    rewireTree_(tree, *rit);
                }
            }
        }
    }
}

void ImomdRRT::updateSelectionProbability(tree_t& tree, location_t& random_point)
{
    // Update probabilty of selecting destinations
    // check whether random_point is generated from destinations
    auto it = std::find(destinations_.begin(), destinations_.end(), random_point.id);
    bool is_destination = it != destinations_.end();
    int other_tree_id = std::distance(destinations_.begin(), it);

    // Update Explore Flag Matrix
    // If all expandable nodes have larger heuristic distance than current distance,
    // It stop explore between two destinations. 
    if (is_destination &&
        expandables_min_heuristic_matrix_[tree.id][other_tree_id].second >
        distance_matrix_[tree.id][other_tree_id])
    {
        tree_t& other_tree = tree_layers_[other_tree_id];

        probability_matrix_[tree.id][other_tree.id] = 0.0;
        probability_matrix_[other_tree.id][tree.id] = 0.0;

        tree.is_done = std::all_of(probability_matrix_[tree.id].begin(), 
            probability_matrix_[tree.id].end(), [](double d){return d == 0;});

        other_tree.is_done = std::all_of(probability_matrix_[other_tree.id].begin(), 
            probability_matrix_[other_tree.id].end(), [](double d){return d == 0;});

        double sum_probability_tree = 
            std::accumulate(probability_matrix_[tree.id].begin(), 
                            probability_matrix_[tree.id].end(), 0.0);
        
        double sum_probability_other_tree = 
            std::accumulate(probability_matrix_[other_tree.id].begin(), 
                            probability_matrix_[other_tree.id].end(), 0.0);
        
        // Update Probability of selecting destinations as random point
        for (int i = 0; i < (int)destinations_.size(); ++i)
        {
            probability_matrix_[tree.id][i] /= sum_probability_tree;
            probability_matrix_[other_tree.id][i] /= sum_probability_other_tree;
        }
    }
}

void ImomdRRT::updateConnectionTree_(tree_t& tree, size_t x_new)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Update Connection of tree", 2, BC);

    // Check whether x_new could connect different two trees
    for (auto& other_tree : tree_layers_)
    {
        if (other_tree.id == tree.id)
        {
            continue;
        }

        // Check x_new connects tree and other_tree
        if (other_tree.checkVisitedNode(x_new))
        {
            // idx : i*(i-1)/2 + j when (i > j)
            int connection_set_idx = (other_tree.id > tree.id) ? 
                other_tree.id * (other_tree.id - 1) / 2 + tree.id : 
                tree.id * (tree.id - 1) / 2 + other_tree.id;

            // Update Connection between two trees
            // When x_new is the first connection node
            static int connection_number = 0;
            if ((*(connection_nodes_set_[connection_set_idx])).empty())
            {
                (*(connection_nodes_set_[connection_set_idx])).insert(x_new);

                // Connect Two tree
                connectTwoTree_(tree, other_tree);
                connection_number++;
            }

            // Update Distance Matrix & Optimal Connection Node
            double distance = tree.cost.at(x_new) + other_tree.cost.at(x_new);

            if (distance < distance_matrix_[tree.id][other_tree.id] - 0.1)
            {
                distance_matrix_[tree.id][other_tree.id] = distance;
                distance_matrix_[other_tree.id][tree.id] = distance;

                connection_node_matrix_[tree.id][other_tree.id] = x_new;
                connection_node_matrix_[other_tree.id][tree.id] = x_new;

                is_distance_matrix_updated_ = true;
            }

            // Save the list of connection node for merge tree later
            if (setting_.pseudo_mode)
            {
                // Check edge(x_new, x_near) are already extended from other trees
                size_t x_parent = tree.parent.at(x_new);
                if (!other_tree.checkVisitedNode(x_parent) ||
                    (other_tree.parent.at(x_parent) != x_new &&
                     other_tree.parent.at(x_new) != x_parent))
                {
                    // update connection nodes set only it is not overlapped
                    (*connection_nodes_set_[connection_set_idx]).insert(x_new);
                }
            }
        }
    }
}

void ImomdRRT::connectTwoTree_(const tree_t& tree1, const tree_t& tree2)
{
    if (is_connected_graph_)
    {
        return;
    }

    int min_tree_id = disjoint_set_parent_[tree1.id];
    int max_tree_id = disjoint_set_parent_[tree2.id];

    if (max_tree_id == min_tree_id)
    {
        return;
    }
    else if (max_tree_id < min_tree_id)
    {
        std::swap(min_tree_id, max_tree_id);
    }

    // unifiy the disjoint set
    // value of disjoint set represents the smallest index of connected trees.
    disjoint_set_parent_[max_tree_id] = min_tree_id;
    disjoint_set_children_[min_tree_id].push_back(max_tree_id);

    for (int child : disjoint_set_children_[max_tree_id])
    {
        disjoint_set_parent_[child] = min_tree_id;
        disjoint_set_children_[min_tree_id].push_back(child);
    }
    disjoint_set_children_[max_tree_id].clear();

    // Check whether the value of disjoint set is all same, which means connected graph
    is_connected_graph_ = std::equal(disjoint_set_parent_.begin() + 1, 
        disjoint_set_parent_.end(), disjoint_set_parent_.begin());
}

void ImomdRRT::mergePseudoTrees_()
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Merge Pseudo Trees", 5, BY);
    std::unordered_set<int> unmerged_pseudo_trees;
    for (int i = 1; i < (int)destinations_.size() - 1; ++i)
    {
        unmerged_pseudo_trees.insert(i);
    }

    // Merge pseudo trees into source_tree and target_tree alternatively
    int pseudo_tree_merge;
    int connection_set_idx;
    while (!unmerged_pseudo_trees.empty())
    {   
        // Find pseudo tree that has the largest number of connection nodes with source_tree
        int max_connection_number = 0;
        for (int pseudo_tree_id : unmerged_pseudo_trees)
        {
            connection_set_idx = pseudo_tree_id * (pseudo_tree_id - 1) / 2;
            int connection_number = (*connection_nodes_set_[connection_set_idx]).size();
            if (connection_number > max_connection_number)
            {
                max_connection_number = connection_number;
                pseudo_tree_merge = pseudo_tree_id;
            }
        }
        // Merge pseudo tree with source_tree
        if (max_connection_number > 0)
        {
            mergeTwoTree_(tree_layers_[source_tree_id_], tree_layers_[pseudo_tree_merge]);
            unmerged_pseudo_trees.erase(pseudo_tree_merge);
        }

        // Find pseudo tree that has the largest number of connection nodes with target_tree
        max_connection_number = 0;
        for (int pseudo_tree_id : unmerged_pseudo_trees)
        {
            connection_set_idx = target_tree_id_ * (target_tree_id_ - 1) / 2 + pseudo_tree_id;
            int connection_number = (*connection_nodes_set_[connection_set_idx]).size();
            if (connection_number > max_connection_number)
            {
                max_connection_number = connection_number;
                pseudo_tree_merge = pseudo_tree_id;
            }
        }
        // Merge pseudo tree with target_tree
        if (max_connection_number > 0)
        {
            mergeTwoTree_(tree_layers_[target_tree_id_], tree_layers_[pseudo_tree_merge]);
            unmerged_pseudo_trees.erase(pseudo_tree_merge);
        }
    }

    // Update Distance Matrix & Optimal Connection Node 
    double min_distance = distance_matrix_[source_tree_id_][target_tree_id_];
    connection_set_idx = target_tree_id_ * (target_tree_id_ - 1) / 2;
    for (auto connection_node : (*connection_nodes_set_[connection_set_idx]))
    {
        double distance = tree_layers_[source_tree_id_].cost.at(connection_node) + 
                          tree_layers_[target_tree_id_].cost.at(connection_node);

        if (distance < min_distance)
        {
            distance_matrix_[source_tree_id_][target_tree_id_] = distance;
            distance_matrix_[target_tree_id_][source_tree_id_] = distance;

            connection_node_matrix_[source_tree_id_][target_tree_id_] = connection_node;
            connection_node_matrix_[target_tree_id_][source_tree_id_] = connection_node;

            is_distance_matrix_updated_ = true;
        }
    }
    is_merge_done_ = true;
}

void ImomdRRT::mergeTwoTree_(tree_t& parent_tree, tree_t& child_tree)
{
    bipedlab::debugger::debugColorTextOutput("[IMOMD] Merge Tree ", 4, BG);

    int connection_set_idx = (parent_tree.id > child_tree.id) ? 
        parent_tree.id * (parent_tree.id - 1) / 2 + child_tree.id : 
        child_tree.id * (child_tree.id - 1) / 2 + parent_tree.id;

    std::unordered_set<int> merged_nodes;
    // Trecing back the child_tree from connection node to root of child_tree
    // Merging Main Brach
    // connection_node ----> child_tree(root)
    for (size_t connection_node : (*connection_nodes_set_[connection_set_idx]))
    {
        size_t current_node = connection_node;
        bool is_optimal = true;

        while (current_node != child_tree.root && is_optimal)
        {
            size_t x_new = child_tree.parent[current_node];
            if (parent_tree.checkVisitedNode(x_new))
            {
                if (merged_nodes.find(x_new) != merged_nodes.end())
                {
                    is_optimal = false;
                }
            }
            else
            {
                connectNewNode_(parent_tree, x_new);
                updateExpandables(parent_tree, x_new, true);
                rewireTree_(parent_tree, x_new);
                updateConnectionTree_(parent_tree, x_new);

                merged_nodes.insert(x_new);
            }
            current_node = x_new;
        }
    }

    // Merging Sub branches of child_tree to parent_tree
    for (const auto& node : child_tree.parent)
    {
        if (parent_tree.checkVisitedNode(node.first))
        {
            continue;
        }

        size_t current_node = node.first;
        // Find branch need to merge by backtracking from fringe
        std::vector<size_t> branch = {current_node};
        while (!parent_tree.expandables.count(current_node))
        {
            current_node = child_tree.parent[current_node];
            branch.push_back(current_node);
        }

        // Merge branch
        for (auto it = branch.rbegin(); it != branch.rend(); ++it)
        {
            size_t x_new = *it;

            if (!parent_tree.checkVisitedNode(x_new))
            {
                connectNewNode_(parent_tree, x_new);
                updateExpandables(parent_tree, x_new, true);
                rewireTree_(parent_tree, x_new);
                updateConnectionTree_(parent_tree, x_new);
            }
        }
    }

    // Merge connection nodes of child_tree into parent_tree
    for (const auto& tree : tree_layers_)
    {
        if (tree.id != child_tree.id && tree.id != parent_tree.id)
        {
            // define connection_set idx
            int child_connection_set_idx = (child_tree.id > tree.id) ? 
                child_tree.id * (child_tree.id - 1) / 2 + tree.id : 
                tree.id * (tree.id - 1) / 2 + child_tree.id;
            
            int parent_connection_set_idx = (parent_tree.id > tree.id) ? 
                parent_tree.id * (parent_tree.id - 1) / 2 + tree.id : 
                tree.id * (tree.id - 1) / 2 + parent_tree.id;

            for (size_t connection_node : (*connection_nodes_set_[child_connection_set_idx]))
            {
                if (!parent_tree.checkVisitedNode(connection_node))
                {
                    (*connection_nodes_set_[parent_connection_set_idx]).insert(connection_node);
                }
            }
        }
    }

    child_tree.is_done = true;
}

void ImomdRRT::updatePath_()
{
    pthread_mutex_lock(&lock_);

    shortest_path_.clear();

    for (auto it = (*sequence_of_tree_id_rtsp_).begin();
         it != std::prev((*sequence_of_tree_id_rtsp_).end()); ++it)
    {
        int current_tree_id = *it;
        int parent_tree_id = *std::next(it,1);

        // current_tree ----- connection_node ------ parent_tree
        size_t connection_node = connection_node_matrix_[current_tree_id][parent_tree_id];
        
        std::vector<size_t> tmp_path;
        // connection_node(exclude) ---> current_tree_root(include)
        size_t node = connection_node;
        while (node != tree_layers_[current_tree_id].root)
        {
            node = tree_layers_[current_tree_id].parent[node];
            tmp_path.push_back((*map_ptr_)[node].id);
        }

        // current_tree_root(include) ---> connection_node(exclude) 
        for (size_t i = tmp_path.size() - 1; i < tmp_path.size(); --i)
        {
            shortest_path_.push_back(tmp_path[i]);
        }

        // connection_node(include) ---> parent_tree_root(exclude) 
        node = connection_node;
        while (node != tree_layers_[parent_tree_id].root)
        {
            shortest_path_.push_back((*map_ptr_)[node].id);
            node = tree_layers_[parent_tree_id].parent[node];
        }
    }
    shortest_path_.push_back(destinations_.back());

    // Thread Lock and Siangl Condition
    is_path_updated_ = true;
    is_distance_matrix_updated_ = false;
    
    pthread_mutex_unlock(&lock_);
    pthread_cond_signal(&cond_get_path_);
}

void ImomdRRT::solveRTSP_()
{
    double path_cost = 0;
    std::shared_ptr<std::vector<int>> tmp_sequence;

    if (is_connected_graph_ && is_distance_matrix_updated_)
    {
        if (!setting_.pseudo_mode)
        {
            std::tie(path_cost, tmp_sequence) = eci_gen_solver_.solveRTSP(
                getDistanceMatrix(), source_tree_id_, target_tree_id_);
            
            if (path_cost < shortest_path_cost_)
            {
                sequence_of_tree_id_rtsp_ = tmp_sequence;
                shortest_path_cost_ = path_cost;
                updatePath_();
                logData_();
            }
            else
            {
                bipedlab::debugger::debugColorOutput("Not found the shorter path | ", path_cost, 8, BY);
            }
        }
        else
        {
            if (!is_merge_done_)
            {
                mergePseudoTrees_();
                sequence_of_tree_id_rtsp_ = std::make_shared<std::vector<int>>(
                    std::vector<int>{source_tree_id_, target_tree_id_});
            }
            shortest_path_cost_ = distance_matrix_[source_tree_id_][target_tree_id_];
            updatePath_();
            logData_();
        }
    }
}

void ImomdRRT::logData_()
{
    double cpu_time = bipedlab::timing::spendElapsedTime(start_time_);
    int tree_size = 0;
    for (const auto& tree : tree_layers_)
    {
        tree_size += tree.parent.size();
    }

    std::cout << "Elapsed time[s]: " << std::setw(10) << cpu_time << " | "
            << "Path Cost[m]: " << std::setw(10) << shortest_path_cost_ << " | "
            << "Tree Size: " << std::setw(10) << tree_size
            << std::endl;
    
    size_t precision = 4;

    if (setting_.log_data)
    {
        command_csv_
        << bipedlab::utils::toStringWithPrecision(cpu_time, precision)
        << bipedlab::utils::toStringWithPrecision(shortest_path_cost_, precision)
        << bipedlab::utils::toStringWithPrecision(tree_size, precision)
        << endrow;
    }
}
