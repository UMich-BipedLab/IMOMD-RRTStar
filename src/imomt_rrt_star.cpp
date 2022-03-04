/*******************************************************************************
 * File:        imomt_rrt_star.cpp
 * 
 * Author:      Dongmyeong Lee (dongmyeong[at]umich.edu)
 * Created:     02/20/2022
 * 
 * Description: Informable Multi-Objective and Multi-Directiobal RRT*
*******************************************************************************/
#include <iostream>

#include "../include/osm_parser/osm_parser.h"
#include "../include/imomt_rrt_star/imomt_rrt_star.h"

int DEBUG_LEVEL = 10;

ImomtRRT::ImomtRRT(int source, int target, std::set<int> objectives, 
    std::shared_ptr<std::vector<location_t>> map_ptr,
    std::shared_ptr<std::vector<std::vector<std::pair<int, double>>>> connection_ptr)
{
    bipedlab::debugger::debugColorTextOutput("Construct ImomtRRT", 10);
    objectives.erase(source);
    objectives.erase(target);

    destinations_.assign(objectives.begin(), objectives.end());
    destinations_.push_back(source);
    destinations_.push_back(target);

    map_ptr_ = map_ptr;
    connection_ptr_ = connection_ptr;

    /**
     * Initialize Trees rooted at each {source, target, objectives}
     * Initialize Union Status of each {source, target, objectives}
     * Initialize Adjacency Matrix of {source, target, objectives} 
     */
    int dest_count = destinations_.size();
    tree_layers_.resize(dest_count);
    union_status_.reserve(dest_count);
    adjacency_matrix_.resize(dest_count, 
                             std::vector<std::pair<double, int>>(dest_count));
    
    for (int i = 0; i < dest_count; ++i)
    {
        tree_layers_[i] = tree_t(i, destinations_[i]);
        union_status_[i] = i;

        adjacency_matrix_[i][i] = {0, i};
        for (int j = i + 1; j < dest_count; ++j)
        {
            adjacency_matrix_[i][j] = {INFINITY, -1};
            adjacency_matrix_[j][i] = {INFINITY, -1};
        }

        if (destinations_[i] == source)
        {
            source_tree_id_ = i;
        }
        else if (destinations_[i] == target)
        {
            target_tree_id_ = i;
        }
    }

    is_complete_union_ = false;
    sequence_of_tree_id_rtsp_ = {};
    sequence_of_tree_id_dijkstra_ = {};
    shortest_path_rtsp_ = {};
    shortest_path_dijkstra_ = {};

    is_adjacency_updated_ = false;
    is_path_updated_ = false;
    cond_get_path_ = PTHREAD_COND_INITIALIZER;
    lock_ = PTHREAD_MUTEX_INITIALIZER;
}

void* ImomtRRT::findShortestPath(void* arg)
{   
    ImomtRRT* this_local = static_cast<ImomtRRT*>(arg);
    int count = 0;

    while (count < MAX_ITER_) // might include more conditions later (i.e., time)
    {
        this_local->expandTreeLayers_();

        if (this_local->is_complete_union_ && this_local->is_adjacency_updated_)
        {
            this_local->solveRTSP_();
            this_local->updatePath_();
        }
        count ++;

        if (count%10 == 0)
        {
            this_local->is_adjacency_updated_ = true;
        }
        bipedlab::debugger::debugColorOutput("count : ", count, 10);
    }
    return 0;
}

void* ImomtRRT::printPath(void* arg)
{
    ImomtRRT* this_local = static_cast<ImomtRRT*>(arg);
    int count = 0;
    while (true)
    {
        pthread_mutex_lock(&this_local->lock_);

        // Wait for path update
        while (!this_local->is_path_updated_)
        {  
            pthread_cond_wait(&this_local->cond_get_path_, &this_local->lock_);
        }

        // Export the shortest path
        bipedlab::debugger::debugTitleTextOutput("[main]", 
                "print path: " + std::to_string(count++), 10, BG);
        for (auto node : this_local->shortest_path_rtsp_)
        {
            std::cout <<  node.id << " (Latitude: " << node.latitude 
                                << ", Longitude: "<< node.longitude 
                                << ")" << std::endl;
        }
        this_local->is_path_updated_ = false;

        pthread_mutex_unlock(&this_local->lock_);
    }
    return 0;
}

void ImomtRRT::expandTreeLayers_()
{   
    int random_point = selectRandomVertex_();
    for (auto& tree : tree_layers_)
    {   
        expandTree_(tree, random_point);
    }
}

void ImomtRRT::expandTree_(tree_t& tree, int random_point)
{
    bipedlab::debugger::debugColorOutput("Expand Tree : ", tree.root, 5, BG);
    bipedlab::debugger::debugColorOutput("Total Nodes : ", tree.nodes.size(), 4, BK);
    bipedlab::debugger::debugColorOutput("Random Point : ", random_point, 3, BW);
    if (tree.checkExistenceNode(random_point))
    {
        return;
    }

    int x_nearest = searchNearestNode_(tree, random_point);
    int x_new = steerNewNode_(tree, x_nearest, random_point);

    if (tree.checkExistenceNode(x_new))
    {
        return;
    }

    int x_min = connectNewNode_(tree, x_nearest, x_new);
    rewireTree_(tree, x_new, x_min);
    updateConnectivityTree_(tree, x_new);
}

// TODO: change to KD-tree
int ImomtRRT::searchNearestNode_(tree_t& tree, int point)
{
    bipedlab::debugger::debugColorTextOutput("Search Nearest Node", 5, BC);
    int x_nearest;
    double min_distance = INFINITY;
    
    for (const auto& node : tree.nodes)
    {
        double distance = computeGreatCircleDistance_(node, point);
        
        if (distance < min_distance)
        {
            min_distance = distance;
            x_nearest = node;
        }
    }
    bipedlab::debugger::debugColorOutput("x_nearest: ", x_nearest, 3, BW);
    return x_nearest;
}

int ImomtRRT::steerNewNode_(tree_t& tree, int x_nearest, int random_point)
{
    bipedlab::debugger::debugColorTextOutput("Steer New Node", 5, BC);
    double cost_x_nearest = tree.cost[x_nearest];

    int x_new = x_nearest;
    double tmp_cost_x_new = cost_x_nearest;

    double min_distance = INFINITY;

    // Iterate for vertice that connected with x_nearest
    for (const auto& connected_vertex : (*connection_ptr_)[x_nearest])
    {
        // check for vertex that not inserted in tree
        if (!tree.checkExistenceNode(connected_vertex.first))
        {
            double distance = computeGreatCircleDistance_(connected_vertex.first, 
                                                          random_point);

            if (distance < min_distance)
            {
                min_distance = distance;
                x_new = connected_vertex.first;
                tmp_cost_x_new = cost_x_nearest + connected_vertex.second;
            }
        }
    }
    tree.cost[x_new] = tmp_cost_x_new; // If x_new already exists in tree, the cost remain same.

    return x_new;
}

int ImomtRRT::connectNewNode_(tree_t& tree, int x_nearest, int x_new)
{
    bipedlab::debugger::debugColorTextOutput("Connect New Node", 5, BC);
    tree.nodes.insert(x_new);

    int x_min = x_nearest;
    int min_cost_x_new = tree.cost[x_new];
    
    // find best parent for x_new that minimize the cost from root to x_new
    for (const auto& x_near : (*connection_ptr_)[x_new])
    {
        if (tree.checkExistenceNode(x_near.first))
        {
            double cost_x_new = tree.cost[x_near.first] + x_near.second;
            if (cost_x_new < min_cost_x_new)
            {
                x_min = x_near.first;
                min_cost_x_new = cost_x_new;
            }
        }
    }

    tree.parent[x_new] = x_min;
    tree.children[x_new] = {};
    tree.cost[x_new] = min_cost_x_new;

    tree.children[x_min].insert(x_new);
    
    bipedlab::debugger::debugColorOutput("x_min : ", x_min, 3, BW);
    return x_min;
}

void ImomtRRT::rewireTree_(tree_t& tree, int x_new, int x_min)
{
    bipedlab::debugger::debugColorTextOutput("Rewiring", 5, BC);

    for (const auto& x_near : (*connection_ptr_)[x_new])
    {
        if (tree.checkExistenceNode(x_near.first) && x_near.first != x_min)
        {
            double new_cost_x_near = tree.cost[x_new] + x_near.second;
            if (tree.cost[x_near.first] > new_cost_x_near)
            {
                // Disconnect with x_near and its parent
                int x_near_old_parent = tree.parent[x_near.first];
                tree.children[x_near_old_parent].erase(x_near.first);

                // Connect with x_near and x_new
                tree.parent[x_near.first] = x_new;
                tree.children[x_new].insert(x_near.first);
                
                // Update cost of x_near and its nested children
                tree.updateCost(x_near.first, new_cost_x_near);

                bipedlab::debugger::debugColorOutput("x_near : ", 
                                                     x_near.first, 3, BW);
            }
        }
    }
}

void ImomtRRT::updateConnectivityTree_(const tree_t& tree, int x_new)
{
    bipedlab::debugger::debugColorTextOutput("Update Connectivity of tree", 5, BC);
    for (const auto& other_tree : tree_layers_)
    {
        if (other_tree.id == tree.id)
        {
            continue;
        }

        if (other_tree.checkExistenceNode(x_new))
        {
            connectTwoTree_(tree, other_tree);

            // Update adjacency_matrix of destinations
            double distance = tree.cost.at(x_new) + other_tree.cost.at(x_new);
            if (distance < adjacency_matrix_[tree.id][other_tree.id].first)
            {
                adjacency_matrix_[tree.id][other_tree.id].first = distance;
                adjacency_matrix_[other_tree.id][tree.id].first = distance;

                adjacency_matrix_[tree.id][other_tree.id].second = x_new;
                adjacency_matrix_[other_tree.id][tree.id].second = x_new;

                is_adjacency_updated_ = true;
            }
        }
    }
}

bool ImomtRRT::connectTwoTree_(const tree_t& tree1, const tree_t& tree2)
{
    if (is_complete_union_)
    {
        return false;
    }

    if (tree1.id < tree2.id)
    {
        union_status_[tree2.id] = tree1.id;
    }
    else
    {
        union_status_[tree1.id] = tree2.id;
    }

    updateUnionStatus_();
    is_complete_union_ = checkCompleteUnion_();

    return true;
}

bool ImomtRRT::solveRTSP_()
{
    solveDijkstra_();

    bipedlab::debugger::debugColorTextOutput("Solve RTSP", 6);
    sequence_of_tree_id_rtsp_.clear();
    sequence_of_tree_id_rtsp_.insert(sequence_of_tree_id_rtsp_.begin(),
        sequence_of_tree_id_dijkstra_.begin(), sequence_of_tree_id_dijkstra_.end());

    // Initialize not_visited_trees
    std::unordered_set<int> not_visited_trees;
    for (std::size_t i = 0; i < tree_layers_.size(); ++i)
    {
        not_visited_trees.insert(i);
    }

    for (int visited_tree : sequence_of_tree_id_dijkstra_)
    {
        not_visited_trees.erase(visited_tree);
    } // Finish initialize not_visited_trees

    while (!not_visited_trees.empty())
    {
        double min_insert_cost = INFINITY;
        int right_idx_insert = -1;
        bool is_method_in_place = true;

        int tree_id_insert = *not_visited_trees.begin();
        for (int candidate_tree_id : not_visited_trees)
        {
            // {min_insert_cost, {right_idx_insert, is_method_in_place}}
            std::pair<double, std::pair<int, bool>> insertion_plan;
            insertion_plan = bestInsertion_(candidate_tree_id);

            if (insertion_plan.first < min_insert_cost)
            {
                tree_id_insert = candidate_tree_id;
                right_idx_insert = insertion_plan.second.first;
                is_method_in_place = insertion_plan.second.second;

                min_insert_cost = insertion_plan.first;
            }
        }

        // Execute in-place insertion
        if (is_method_in_place)
        {
            insertInPlace(sequence_of_tree_id_rtsp_, tree_id_insert, right_idx_insert);
        }
        // Execute in-sequence insertion
        else
        {
            insertInSequence(sequence_of_tree_id_rtsp_, tree_id_insert, right_idx_insert);
        }

        not_visited_trees.erase(tree_id_insert);
    }

    if (DEBUG_LEVEL <= 4)
    {
        printRTSPId();
    }

    return true;
}

std::pair<double, std::pair<int, int>> ImomtRRT::bestInsertion_(int tree_id)
{
    double min_insert_cost = INFINITY;

    int best_right_idx = -1;
    bool is_method_in_place = true;

    bipedlab::debugger::debugColorOutput("Best Insertion : ", tree_id, 3, BY);

    std::list<int>::iterator it = sequence_of_tree_id_rtsp_.begin();
    while (it != sequence_of_tree_id_rtsp_.end())
    {   
        double in_place_cost = 2 * adjacency_matrix_[tree_id][*it].first;
        double in_sequence_cost;

        // If the iterator is the last element, there is no option for in-sequence
        if (std::distance(it, sequence_of_tree_id_rtsp_.end()) == 1)
        {
            in_sequence_cost = in_place_cost;
            ++it;
        }
        else
        {
            in_sequence_cost = adjacency_matrix_[tree_id][*it].first -
                               adjacency_matrix_[*it][*it++].first +
                               adjacency_matrix_[*it][tree_id].first;
        }
        
        if (in_place_cost < min_insert_cost)
        {
            best_right_idx = std::distance(sequence_of_tree_id_rtsp_.begin(), it);
            min_insert_cost = in_place_cost;
            is_method_in_place = true;
        }
        
        if (in_sequence_cost < min_insert_cost)
        {
            best_right_idx = std::distance(sequence_of_tree_id_rtsp_.begin(), it);
            min_insert_cost = in_sequence_cost;
            is_method_in_place = false;
        }
    }
    return std::make_pair(min_insert_cost, std::make_pair(best_right_idx, is_method_in_place));
}

bool ImomtRRT::solveDijkstra_()
{
    bipedlab::debugger::debugColorTextOutput("Solve Dijkstra", 6);
    std::unordered_map<int, int> parent = { {source_tree_id_, source_tree_id_} };
    std::unordered_map<int, double> cost_from_source = {
        {source_tree_id_, 0}, {target_tree_id_, INFINITY}
    };

    std::priority_queue<std::pair<double, int>> open_queue;
    open_queue.push(std::make_pair(0, source_tree_id_));

    bool flag = false;
    while(!open_queue.empty())
    {
        std::pair<double, int> state = open_queue.top();
        open_queue.pop();

        // Reached to the target
        if (state.second == target_tree_id_)
        {
            flag = true;
            break;
        }

        // Search for neighbors of state
        for (std::size_t i = 0; i < adjacency_matrix_.size(); ++i)
        {
            if (state.second == i)
            {
                continue;
            }

            double new_cost = cost_from_source[state.second] + 
                              adjacency_matrix_[state.second][i].first;

            if (!cost_from_source.count(i))
            {
                cost_from_source[i] = INFINITY;
            }
            
            if (new_cost < cost_from_source[i])
            {
                cost_from_source[i] = new_cost;
                parent[i] = state.second;

                open_queue.push(std::make_pair(-new_cost, i));
            }
        }
    }
    extractSequence_(parent, source_tree_id_, target_tree_id_, sequence_of_tree_id_dijkstra_);

    if (DEBUG_LEVEL <= 4)
    {
        printDijkstraId();
    }

    return flag;
}

bool ImomtRRT::updatePath_()
{
    if (DEBUG_LEVEL <= 4)
    {
        printAdjacentMatrix();
    }

    std::deque<location_t> tmp_shortest_path_rtsp = {};
    int count = 0;
    std::list<int>::reverse_iterator rit = sequence_of_tree_id_rtsp_.rbegin();
    while(rit != sequence_of_tree_id_rtsp_.rend())
    {   
        int current_tree_id = *rit;
        int parent_tree_id = *++rit;

        if (rit == sequence_of_tree_id_rtsp_.rend())
        {
            break;
        }

        // parent_tree ----- connection_node ------ current_tree
        int connection_node_id = adjacency_matrix_[current_tree_id][parent_tree_id].second;
        std::vector<location_t> tmp_path = {(*map_ptr_)[connection_node_id]};

        // connection_node --------> current_tree_root
        int node = connection_node_id;
        while (true)
        {
            node = tree_layers_[current_tree_id].parent[node];
            tmp_path.push_back((*map_ptr_)[node]);
            if (node == tree_layers_[current_tree_id].root)
            {
                break;
            }
        }
        for (std::vector<location_t>::reverse_iterator rit = tmp_path.rbegin();
            rit != tmp_path.rend(); ++rit)
        {
            tmp_shortest_path_rtsp.push_front(*rit);
        }

        // parent_tree_root <-------- connection_node
        node = connection_node_id;
        while (true)
        {
            node = tree_layers_[parent_tree_id].parent[node];
            if (node == tree_layers_[parent_tree_id].root)
            {
                break;
            }
            tmp_shortest_path_rtsp.push_front((*map_ptr_)[node]);
        }
    }
    tmp_shortest_path_rtsp.push_front((*map_ptr_)[tree_layers_[source_tree_id_].root]);

    // Thread Lock and Siangl Condition
    pthread_mutex_lock(&lock_);

    shortest_path_rtsp_.assign(tmp_shortest_path_rtsp.begin(), tmp_shortest_path_rtsp.end());
    is_path_updated_ = true;
    is_adjacency_updated_ = false;
    
    pthread_mutex_unlock(&lock_);
    pthread_cond_signal(&cond_get_path_);

    return true;
}

bool ImomtRRT::checkCompleteUnion_()
{
    int standard = union_status_.begin()->second;
    for (const auto& element : union_status_)
    {
        if (element.second != standard)
        {
            return false;
        }
    }
    return true;
}

void ImomtRRT::updateUnionStatus_()
{   
    for (const auto& tree : tree_layers_)
    {
        // find min root_id between trees that connected with input tree
        int min_connected_tree_id = tree.id;
        while (union_status_[min_connected_tree_id] != min_connected_tree_id)
        {
            min_connected_tree_id = union_status_[min_connected_tree_id];
        }

        // Update union_status_ for all trees that connected with input tree
        int next_tree_id = tree.id;
        while (union_status_[next_tree_id] != min_connected_tree_id)
        {
            int tmp_tree_id = union_status_[next_tree_id];
            union_status_[next_tree_id] = min_connected_tree_id;
            next_tree_id = tmp_tree_id;
        }
    }
}

int ImomtRRT::selectRandomVertex_()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dist(0, 1);
    static std::uniform_int_distribution<int> uni_goal(0, destinations_.size() - 1);
    static std::uniform_int_distribution<int> uni_map(0, (*map_ptr_).size() - 1);

    int random_point;
    if (dist(gen) < GOAL_BIAS_) // Destinations 
    {
        random_point = destinations_[uni_goal(gen)];
    }
    else // Random Point
    {
        random_point = uni_map(gen);
    }
    return random_point;
}

double ImomtRRT::computeGreatCircleDistance_(int node1, int node2)
{
    double distance = getDistanceFromLatLon_(
        (*map_ptr_)[node1].latitude, (*map_ptr_)[node1].longitude,
        (*map_ptr_)[node2].latitude, (*map_ptr_)[node2].longitude);
    
    return distance;
}

double ImomtRRT::getDistanceFromConnection_(int node1, int node2)
{
    for (const auto& connection : (*connection_ptr_)[node1])
    {
        if (connection.first == node2)
        {
            return connection.second;
        }
    }
    return INFINITY;
}

double ImomtRRT::getDistanceFromLatLon_(double lat1, double lon1, double lat2, double lon2)
{
    static constexpr double DEG2RAD = M_PI / 180;
    static const int EARTH_RADIUS = 6371e3; // in metres;

    double lat1_rad = lat1 * DEG2RAD;
    double lat2_rad = lat2 * DEG2RAD; 
    double diff_lat_rad = (lat2 - lat1) * DEG2RAD;
    double diff_lon_rad = (lon2 - lon1) * DEG2RAD;

    // 'Harversine' Formula
    // http://www.movable-type.co.uk/scripts/latlong.html
    double a = std::pow(std::sin(diff_lat_rad / 2), 2) + std::cos(lat1_rad) * 
               std::cos(lat2_rad) * std::pow(std::sin(diff_lon_rad / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double d = EARTH_RADIUS * c; // in metres

    return d;
}

void ImomtRRT::extractSequence_(const std::unordered_map<int, int>& parent,
    int source, int target, std::list<int>& sequence)
{
    sequence = {target};
    int state = target;

    while (true)
    {
        state = parent.at(state);
        sequence.push_back(state);
        if (state == source)
        {
            break;
        }
    }
    std::reverse(sequence.begin(), sequence.end());
}

void ImomtRRT::insertInPlace(std::list<int>& sequence, int value, int right_idx)
{
    auto it = sequence.begin();
    std::advance(it, right_idx);

    sequence.insert(it, value);
    sequence.insert(--it, *it);
}


void ImomtRRT::insertInSequence(std::list<int>& sequence, int value, int right_idx)
{
    auto it = sequence.begin();
    std::advance(it, right_idx);

    sequence.insert(it, value);
}

void ImomtRRT::printDijkstraId()
{
    bipedlab::debugger::debugColorTextOutput("Sequence of tree_id from Dijkstra", 
                                             4, BM);

    for (auto i : sequence_of_tree_id_dijkstra_)
    {
        std::cout << tree_layers_[i].id << " -> ";
    }
    std::cout << "#" << std::endl;
}

void ImomtRRT::printAdjacentMatrix()
{
    bipedlab::debugger::debugColorTextOutput("Adjacency Matrix", 4);
    bipedlab::debugger::debugColorTextOutput("Connection Node", 3, BB);
    for (auto row : adjacency_matrix_)
    {
        for (auto item : row)
        {
            std::cout << item.second << " ";
        }
        std::cout << std::endl;
    }
    bipedlab::debugger::debugColorTextOutput("Shortest Direct Path", 3, BB);
    for (auto row : adjacency_matrix_)
    {
        for (auto item : row)
        {
            std::cout << item.first << " ";
        }
        std::cout << std::endl;
    }
}

void ImomtRRT::printRTSPId()
{
    bipedlab::debugger::debugColorTextOutput("Sequence of tree_id from RTSP", 4, BM);
    for (auto i : sequence_of_tree_id_rtsp_)
    {
        std::cout << tree_layers_[i].id << " -> ";
    }
    std::cout << "#" << std::endl;
}

int main(){
    std::string osm_file = "/home/dongmyeong/catkin_ws/src/imomt_rrt_star/osm_data/sanf.osm";
    auto osmparser = OSMParser(osm_file);
    osmparser.parse();

    ImomtRRT imomt(0, 6355, {2163}, osmparser.getMap(), osmparser.getConnection());
    
    pthread_t tid[2];

    bipedlab::debugger::debugTitleTextOutput("[main]", "IMOMT running", 10);
    pthread_create(&tid[0], NULL, ImomtRRT::findShortestPath, &imomt);
    pthread_create(&tid[1], NULL, ImomtRRT::printPath, &imomt);

    pthread_join(tid[0], NULL);
    pthread_join(tid[1], NULL);
}