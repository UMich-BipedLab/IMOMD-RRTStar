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
 * File:        greedy_tsp.h
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     03/15/2022
 * 
 * Description: solve tsp by brute-force method
*******************************************************************************/
#ifndef GREEDY_TSP_H
#define GREEDY_TSP_H

#include <iostream>
#include <memory>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <math.h>
#include <queue>

#include "utils/debugger.h"

void nested_loop(std::unordered_set<int> objectives, std::vector<int>& path,
    std::vector<std::vector<int>>& all_options);

std::pair<double, std::vector<int>> find_connection_dijkstra(
    std::shared_ptr<std::vector<std::vector<double>>> adjacency_matrix_ptr,
    int source, int target);

std::pair<double, std::shared_ptr<std::vector<int>>> tsp_greed(
    std::shared_ptr<std::vector<std::vector<double>>> adjacency_matrix_ptr,
    int source_id, int target_id)
{
    int n = (*adjacency_matrix_ptr).size();
    std::unordered_set<int> objectives;
    for (int i = 0; i < n; ++i)
    {
        objectives.insert(i);
    }
    objectives.erase(source_id);
    objectives.erase(target_id);

    // Make Adjacency Matrix as Complete Graph with Dijkstra
    std::vector<std::vector<double>> complete_adjacency_matrix(*adjacency_matrix_ptr);
    std::unordered_map<int, std::unordered_map<int, std::vector<int>>> way_points;
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            if (std::isinf((*adjacency_matrix_ptr)[i][j]))
            {
                std::tie(complete_adjacency_matrix[i][j], way_points[i][j]) = 
                    find_connection_dijkstra(adjacency_matrix_ptr, i, j);
            }
        }
    }

    // Solve with Brute Force
    double min_cost = INFINITY;

    std::vector<int> min_path;
    std::vector<int> tmp_path;
    std::vector<std::vector<int>> all_options;
    nested_loop(objectives, tmp_path, all_options);

    for (auto option : all_options)
    {
        double tmp_cost = complete_adjacency_matrix[source_id][option[0]];
        for (size_t i = 0; i < option.size() - 1; ++i)
        {
            tmp_cost += complete_adjacency_matrix[option[i]][option[i + 1]];
        }
        tmp_cost += complete_adjacency_matrix[option[option.size() - 1]][target_id];

        if (tmp_cost < min_cost)
        {
            min_cost = tmp_cost;
            min_path = option;
        }
    }
    min_path.insert(min_path.begin(), source_id);
    min_path.push_back(target_id);

    // Reorganize
    std::vector<int> shortest_path;

    for (auto it = min_path.begin(); it != std::prev(min_path.end()); ++it)
    {
        auto nx = std::next(it);

        shortest_path.push_back(*it);

        if (way_points.count(*it) && way_points[*it].count(*nx))
        {
            for (auto waypoint : way_points[*it][*nx])
            {
                shortest_path.push_back(waypoint);
            }
        }
    }
    shortest_path.push_back(target_id);

    return std::make_pair(min_cost, std::make_shared<std::vector<int>>(shortest_path));
}

void nested_loop(std::unordered_set<int> objectives, std::vector<int>& path,
                 std::vector<std::vector<int>>& all_options)
{
    if (objectives.size() > 1)
    {
        for (int obj : objectives)
        {
            std::unordered_set<int> tmp_objectives = objectives;
            std::vector<int> tmp_path = path;
            tmp_objectives.erase(obj);
            tmp_path.push_back(obj);
            nested_loop(tmp_objectives, tmp_path, all_options);
        }
    }
    else
    {
        path.push_back(*objectives.begin());
        all_options.push_back(path);
    }
}

std::pair<double, std::vector<int>> find_connection_dijkstra(
    std::shared_ptr<std::vector<std::vector<double>>> adjacency_matrix_ptr,
    int source, int target)
{
    if (std::isinf((*adjacency_matrix_ptr)[source][target]))
    {
        std::unordered_map<int, int> parent = { {source, source} };
        std::unordered_map<int, double> cost_from_source = {
            {source, 0}, {target, INFINITY}
        };

        // { cost-to-come, destination_id }
        std::priority_queue<std::pair<double, int>> open_queue;
        open_queue.push(std::make_pair(0, source));

        while (!open_queue.empty())
        {
            std::pair<double, int> state = open_queue.top();
            open_queue.pop();

            // Reached to the target
            if (state.second == target)
            {
                break;
            }

            // Search for neighbor of state
            for (std::size_t i = 0; i < (*adjacency_matrix_ptr).size(); ++i)
            {
                if (state.second == (int)i)
                {
                    continue;
                }

                double new_cost = cost_from_source[state.second] + 
                                  (*adjacency_matrix_ptr)[state.second][i];

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

        std::vector<int> waypoints;
        int state = parent[target];

        while (state != source)
        {
            waypoints.push_back(state);
            state = parent[state];        
        }
        std::reverse(waypoints.begin(), waypoints.end());

        return std::make_pair(cost_from_source[target], waypoints);
    }

    return {(*adjacency_matrix_ptr)[source][target], {}};
}

#endif /* GREEDY_TSP_H */
