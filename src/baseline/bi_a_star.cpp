/*******************************************************************************
 * File:        bi_a_star.cpp
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     04/23/2022
 * 
 * Description: Expand from destinations separately in Bidirectional A* way.
 *              Each tree grows until it connects to all trees.
*******************************************************************************/
#include <iostream>
#include "baseline/bi_a_star.h"

BiAstar::BiAstar(size_t source, size_t target, std::vector<size_t> objectives, 
    const std::shared_ptr<std::vector<location_t>> map_ptr,
    const std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> connection_ptr,
    const imomd_setting_t& setting)
    : setting_(setting), map_ptr_(map_ptr), connection_ptr_(connection_ptr),
      is_connected_graph_(false), is_distance_matrix_updated_(false),
      is_path_updated_(false), reached_max_iter_(false),
      cond_get_path_(PTHREAD_COND_INITIALIZER), lock_(PTHREAD_MUTEX_INITIALIZER)
{
    bipedlab::debugger::debugColorTextOutput("[Bi-A*] Constructing Baseline", 10, BC);

    destinations_ = {source};
    for (const auto& objective : objectives)
    {
        destinations_.push_back(objective);
    }
    destinations_.push_back(target);

    /**
     * Initialize Trees rooted at each {source, target, objectives}
     * Initialize Union Status of each {source, target, objectives}
     * Initialize Adjacency Matrix of {source, target, objectives}
     * Initialize Connection Node Matrix of {source, target, objectives} 
     */
    int dest_count = destinations_.size();
    tree_layers_.resize(dest_count * (dest_count - 1) / 2);
    disjoint_set_parent_.resize(dest_count);

    distance_matrix_.resize(dest_count, std::vector<double>(dest_count));
    connection_node_matrix_.resize(dest_count, std::vector<int>(dest_count));
    
    for (int i = 0; i < dest_count; ++i)
    {
        // Initialize Disjoint-set Data Structure
        disjoint_set_parent_[i] = i;
        disjoint_set_children_[i] = {};

        distance_matrix_[i][i] = 0;
        connection_node_matrix_[i][i] = -1;
        for (int j = i + 1; j < dest_count; ++j)
        {
            tree_layers_[j * (j - 1) / 2 + i] = 
                tree_bi_astar_t(i, j, destinations_[i], destinations_[j]);
            
            unexplored_tree_.insert(j * (j - 1) / 2 + i);

            distance_matrix_[i][j] = INFINITY;
            distance_matrix_[j][i] = INFINITY;

            connection_node_matrix_[i][j] = -1;
            connection_node_matrix_[j][i] = -1;
        }
    }
    source_tree_id_ = 0;
    target_tree_id_ = dest_count - 1;
    
    // Initialize RTSP solver
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

            command_csv_.open(std::string(path) + "/experiments/Bi_Astar_" + date + "_command_history.csv");

            // Header
            command_csv_ << "CPU_time" << "path_cost" << "tree_size" << endrow;
        }
        catch (const std::exception& ex)
        {
            std::cout << "Exception was thrown: " << ex.what() << std::endl;
        }
    }
}

void* BiAstar::findShortestPath(void* arg)
{   
    BiAstar* this_thread = static_cast<BiAstar*>(arg);

    this_thread->sequence_of_tree_id_rtsp_ = std::make_shared<std::vector<int>>(
        std::vector<int>{this_thread->source_tree_id_, this_thread->target_tree_id_});
    
    bool is_done_iter = false;
    bool is_done_time = false;
    while (!is_done_iter && !is_done_time && !this_thread->unexplored_tree_.empty())
    {
        this_thread->expandTreeLayers_();
        this_thread->solveRTSP_();

        this_thread->iteration_count_++;
        bipedlab::debugger::debugColorOutput("[bi-A*] count : ", 
                this_thread->iteration_count_, 3);
        
        if (this_thread->iteration_count_ > this_thread->setting_.max_iter)
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
        bipedlab::debugger::debugTitleTextOutput("[bi-A*]", "Reached Max Iteration", 10, BM);
    }
    else if (is_done_time)
    {
        bipedlab::debugger::debugTitleTextOutput("[bi-A*]", "Reached Max Time", 10, BM);
    }
    
    if (this_thread->unexplored_tree_.empty())
    {
        bipedlab::debugger::debugTitleTextOutput("[bi-A*]", "Tree Exploration Done", 10, BM);
    }

    int count = 0;
    while (count++ < 10)
    {
        this_thread->solveRTSP_();
    }

    pthread_mutex_lock(&this_thread->lock_);
    this_thread->reached_max_iter_ = true;
    pthread_mutex_unlock(&this_thread->lock_);
    pthread_cond_signal(&this_thread->cond_get_path_);

    pthread_exit(NULL);
}

std::shared_ptr<std::vector<size_t>> BiAstar::getShortestPath()
{   
    pthread_mutex_lock(&lock_);
    
    // Wait for path update or termination of getShortestPath
    while (!is_path_updated_ && !reached_max_iter_)
    {  
        pthread_cond_wait(&cond_get_path_, &lock_);
        bipedlab::debugger::debugTitleTextOutput("[main]", "Find new path", 10, BG);
    }

    auto shortest_path_ptr = std::make_shared<std::vector<size_t>>(shortest_path_);
    is_path_updated_ = false;

    pthread_mutex_unlock(&lock_);

    return shortest_path_ptr;
}

void* BiAstar::printPath(void* arg)
{
    BiAstar* this_local = static_cast<BiAstar*>(arg);

    int count = 0;
    while (!this_local->reached_max_iter_)
    {
        pthread_mutex_lock(&this_local->lock_);

        // Wait for path update or termination of getShortestPath
        while (!this_local->is_path_updated_ && !this_local->reached_max_iter_)
        {  
            pthread_cond_wait(&this_local->cond_get_path_, &this_local->lock_);
        }

        // Export the shortest path
        bipedlab::debugger::debugTitleTextOutput("[main]", 
                "print path: " + std::to_string(count++), 10, BG);
        for (auto node : this_local->shortest_path_)
        {
            std::cout << node << " -> ";
        }
        std::cout << "#" << std::endl;

        this_local->is_path_updated_ = false;
        pthread_mutex_unlock(&this_local->lock_);
    }
    
    pthread_exit(NULL);
}

void BiAstar::expandTreeLayers_()
{
    static std::uniform_int_distribution<> uni_tree(0, (int)tree_layers_.size() - 1);

    int tree_id = uni_tree(random_gen_);
    while (unexplored_tree_.find(tree_id) == unexplored_tree_.end())
    {
        tree_id = uni_tree(random_gen_);
    }

    expandTree_(tree_layers_[tree_id]);
    unexplored_tree_.erase(tree_id);
}

void BiAstar::expandTree_(tree_bi_astar_t& tree)
{
    bipedlab::debugger::debugColorOutput("[Bi-A*] Expand from Start : ", tree.start_root, 3, BG);
    bipedlab::debugger::debugColorOutput("[Bi-A*] Expand from Goal : ", tree.goal_root, 3, BG);
    
    size_t connection_node;

    double f_value;
    size_t state;
    while (!tree.open_queue_forward.empty() && !tree.open_queue_forward.empty())
    {
        // Forward Search
        std::tie(f_value, state) = tree.open_queue_forward.top();
        tree.open_queue_forward.pop();

        if (tree.parent_backward.find(state) != tree.parent_backward.end())
        {
            connection_node = state;
            break;
        }

        for (const auto& x_near : (*connection_ptr_)[state])
        {
            double new_cost = tree.cost_forward[state] + x_near.second;
            
            if (tree.cost_forward.find(x_near.first) == tree.cost_forward.end())
            {
                tree.cost_forward[x_near.first] = INFINITY;
            }

            if (new_cost < tree.cost_forward[x_near.first])
            {
                tree.cost_forward[x_near.first] = new_cost;
                tree.parent_forward[x_near.first] = state;

                double new_f_value = new_cost + 
                    computeHaversineDistance((*map_ptr_)[x_near.first], (*map_ptr_)[tree.goal_root]);

                tree.open_queue_forward.push(std::make_pair(-new_f_value, x_near.first));
            }
        }

        // Backward Search
        std::tie(f_value, state) = tree.open_queue_backward.top();
        tree.open_queue_backward.pop();

        if (tree.parent_forward.find(state) != tree.parent_forward.end())
        {
            connection_node = state;
            break;
        }

        for (const auto& x_near : (*connection_ptr_)[state])
        {
            double new_cost = tree.cost_backward[state] + x_near.second;
            
            if (tree.cost_backward.find(x_near.first) == tree.cost_backward.end())
            {
                tree.cost_backward[x_near.first] = INFINITY;
            }

            if (new_cost < tree.cost_backward[x_near.first])
            {
                tree.cost_backward[x_near.first] = new_cost;
                tree.parent_backward[x_near.first] = state;

                double new_f_value = new_cost + 
                    computeHaversineDistance((*map_ptr_)[x_near.first], (*map_ptr_)[tree.start_root]);

                tree.open_queue_backward.push(std::make_pair(-new_f_value, x_near.first));
            }
        }
    }

    double distance = tree.cost_forward[connection_node] + tree.cost_backward[connection_node];

    if (distance < distance_matrix_[tree.start_id][tree.goal_id] - 0.1)
    {
        distance_matrix_[tree.start_id][tree.goal_id] = distance;
        distance_matrix_[tree.goal_id][tree.start_id] = distance;

        connection_node_matrix_[tree.start_id][tree.goal_id] = connection_node;
        connection_node_matrix_[tree.goal_id][tree.start_id] = connection_node;

        is_distance_matrix_updated_ = true;
    }
    
    connectTwoTree_(tree.start_id, tree.goal_id);

    bipedlab::debugger::debugColorOutput("[Bi-A*] Connection Node : ", connection_node, 1, BK);
    bipedlab::debugger::debugColorOutput("[Bi-A*] Total Nodes : ", tree.parent_forward.size(), 1, BK);
    bipedlab::debugger::debugColorOutput("[Bi-A*] Total Nodes : ", tree.parent_backward.size(), 1, BK);
}

void BiAstar::connectTwoTree_(int tree1_id, int tree2_id)
{
    if (is_connected_graph_)
    {
        return;
    }

    int min_tree_id = disjoint_set_parent_[tree1_id];
    int max_tree_id = disjoint_set_parent_[tree2_id];

    if (max_tree_id == min_tree_id)
    {
        return;
    }
    else if (max_tree_id < min_tree_id)
    {
        std::swap(min_tree_id, max_tree_id);
    }

    // unifiy the disjoint set
    // value represents the smallest index of connected trees.
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

bool BiAstar::updatePath_()
{
    shortest_path_.clear();

    for (int i = 0; i < (int)(*sequence_of_tree_id_rtsp_).size() - 1; ++i)
    {
        int start_id = (*sequence_of_tree_id_rtsp_)[i];
        int goal_id = (*sequence_of_tree_id_rtsp_)[i + 1];

        int tree_idx = goal_id * (goal_id - 1) / 2 + start_id;
        if (goal_id < start_id)
        {
            tree_idx = start_id * (start_id - 1) / 2 + goal_id;
        }

        // start_root ----- connection_node ------ goal_root
        size_t connection_node = connection_node_matrix_[start_id][goal_id];
        size_t node = connection_node;

        std::vector<size_t> tmp_path;
        if (tree_layers_[tree_idx].start_id == start_id)
        {
            // start_root <----- connection_node
            while (node != tree_layers_[tree_idx].start_root)
            {
                node = tree_layers_[tree_idx].parent_forward[node];
                tmp_path.push_back(node);
            }
            
            // shortest_path.append (start_root, ..., connection_node) exclude(connection_node)
            for (int j = (int)tmp_path.size() - 1; j >= 0; --j)
            {
                shortest_path_.push_back(tmp_path[j]);
            }

            // connection_node ------> goal_root (exclude goal_root)
            node = connection_node;
            while (node != tree_layers_[tree_idx].goal_root)
            {
                shortest_path_.push_back(node);
                node = tree_layers_[tree_idx].parent_backward[node];
            }
        }
        else
        {
            // goal_root <----- connection_node
            while (node != tree_layers_[tree_idx].goal_root)
            {
                node = tree_layers_[tree_idx].parent_backward[node];
                tmp_path.push_back(node);
            }
            
            // shortest_path.append (goal_root, ..., connection_node) exclude(connection_node)
            for (int j = (int)tmp_path.size() - 1; j >= 0; --j)
            {
                shortest_path_.push_back(tmp_path[j]);
            }

            // connection_node ------> start_root (exclude start_root)
            node = connection_node;
            while (node != tree_layers_[tree_idx].start_root)
            {
                shortest_path_.push_back(node);
                node = tree_layers_[tree_idx].parent_forward[node];
            }
        }
    }
    shortest_path_.push_back(destinations_.back());

    // Thread Lock and Siangl Condition
    pthread_mutex_lock(&lock_);

    is_path_updated_ = true;
    is_distance_matrix_updated_ = false;
    
    pthread_mutex_unlock(&lock_);
    pthread_cond_signal(&cond_get_path_);

    return true;
}

std::shared_ptr<std::vector<std::vector<double>>> BiAstar::getDistanceMatrix()
{
    return std::make_shared<std::vector<std::vector<double>>>(distance_matrix_);
};

void BiAstar::solveRTSP_()
{
    double path_cost = 0;
    std::shared_ptr<std::vector<int>> tmp_sequence;

    if (is_connected_graph_ && is_distance_matrix_updated_)
    {
        if (!setting_.pseudo_mode)
        {
            std::tie(path_cost, tmp_sequence) = 
            eci_gen_solver_.solveRTSP(getDistanceMatrix(),
                source_tree_id_, target_tree_id_);
        }
        else
        {
            std::tie(path_cost, tmp_sequence) = 
            eci_gen_solver_.solveDijkstra(getDistanceMatrix(),
                source_tree_id_, target_tree_id_);
        }
        
        // Update path and cost
        if (path_cost < shortest_path_cost_)
        {
            sequence_of_tree_id_rtsp_ = tmp_sequence;
            shortest_path_cost_ = path_cost;
            updatePath_();
            logData_();
        }
        else
        {
            bipedlab::debugger::debugColorOutput("Not found the shorter path than ", path_cost, 8);
        }
    }
}

void BiAstar::logData_()
{    
    double cpu_time = bipedlab::timing::spendElapsedTime(start_time_);
    int tree_size = 0;
    for (const auto& tree : tree_layers_)
    {
        tree_size += tree.parent_forward.size();
        tree_size += tree.parent_backward.size();
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