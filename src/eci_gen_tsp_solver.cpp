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
 * File:        eci_gen_tsp_solver.cpp
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     03/10/2022
 * 
 * Description: Solving Relaxed Traveling Salesman Problem with 
 *              Advanced Cheapest Insertion Method with Genetic Algorithm
*******************************************************************************/
#include "imomd_rrt_star/eci_gen_tsp_solver.h"

EciGenSolver::EciGenSolver(const rtsp_setting_t& setting): setting_(setting)
{
    bipedlab::debugger::debugColorTextOutput("[RTSP] Initialized ECI-GEN Solver", 10, BC);

    // Initialize Random genrator
    if (setting_.ga_random_seed)
    {
        std::random_device rd;
        random_gen_ = std::mt19937{rd()};
    }
    else
    {
        random_gen_ = std::mt19937{0};
    }
}

std::pair<double, std::shared_ptr<std::vector<int>>> EciGenSolver::solveDijkstra(
    std::shared_ptr<std::vector<std::vector<double>>> distance_matrix_ptr,
    int source_id, int target_id)
{
    distance_matrix_ptr_ = distance_matrix_ptr;
    source_id_ = source_id;
    target_id_ = target_id;

    sequence_rtsp_.clear();
    sequence_best_insertion_.clear();
    sequence_dijkstra_.clear();

    chromosomes_.clear();
    ga_fitness_.clear();

    min_path_cost_ = solveDijkstra_();

    return std::make_pair(min_path_cost_, std::make_shared<std::vector<int>>(
        sequence_dijkstra_.begin(), sequence_dijkstra_.end()));
}

std::pair<double, std::shared_ptr<std::vector<int>>> EciGenSolver::solveRTSP(
    std::shared_ptr<std::vector<std::vector<double>>> adjacency_matrix_ptr,
    int source_id, int target_id)
{
    distance_matrix_ptr_ = adjacency_matrix_ptr;
    source_id_ = source_id;
    target_id_ = target_id;

    min_path_cost_ = 0;

    sequence_rtsp_.clear();
    sequence_best_insertion_.clear();
    sequence_dijkstra_.clear();

    chromosomes_.clear();
    ga_fitness_.clear();

    solveBestInsertion_();

    if (setting_.genetic)
    {   
        solveGeneticAlgorithm_();
    }

    return std::make_pair(min_path_cost_, std::make_shared<std::vector<int>>(sequence_rtsp_));
}

void EciGenSolver::solveBestInsertion_()
{   
    double path_cost = solveDijkstra_();
    bipedlab::debugger::debugColorTextOutput("[ECI-GEN] Solve RTSP", 8);
    sequence_best_insertion_.clear();
    sequence_best_insertion_.insert(sequence_best_insertion_.begin(),
                          sequence_dijkstra_.begin(), sequence_dijkstra_.end());

    // Initialize not_visited destinations
    std::unordered_set<int> not_visited;
    for (std::size_t i = 0; i < (*distance_matrix_ptr_).size(); ++i)
    {
        not_visited.insert(i);
    }

    for (int visited : sequence_dijkstra_)
    {
        not_visited.erase(visited);
    } // Finish initialize not_visited

    // Insert Not Visited Destinations in Dijkstra Sequence
    while (!not_visited.empty())
    {
        double min_insert_cost = INFINITY;
        int left_idx_insert = -1;
        Insertion_types_t insertion_method;

        int id_insert = -1;
        for (int tmp_insert_id : not_visited)
        {
            // { min_insert_cost, {left_idx_insert, insertion_method} }
            std::pair<double, std::pair<int, Insertion_types_t>> insertion_plan;
            insertion_plan = bestInsertion_(tmp_insert_id);

            if (insertion_plan.first < min_insert_cost)
            {
                id_insert = tmp_insert_id;
                left_idx_insert = insertion_plan.second.first;
                insertion_method = insertion_plan.second.second;

                min_insert_cost = insertion_plan.first;
            }
        }

        switch(insertion_method)
        {
            case Insertion_types_t::REVISIT:
            {
                insertRevisit_(sequence_best_insertion_, id_insert, left_idx_insert);
                break;
            }

            case Insertion_types_t::SEQUENCE:
            {
                insertSequence_(sequence_best_insertion_, id_insert, left_idx_insert);
                break;
            }

            case Insertion_types_t::SEQUENCE_SWAP_LEFT:
            {
                insertSequenceSwapLeft_(sequence_best_insertion_, id_insert, left_idx_insert);
                break;
            }

            case Insertion_types_t::SEQUENCE_SWAP_RIGHT:
            {
                insertSequenceSwapRight_(sequence_best_insertion_, id_insert, left_idx_insert);
                break;
            }

            case Insertion_types_t::SEQUENCE_SWAP_BOTH:
            {
                insertSequenceSwapBoth_(sequence_best_insertion_, id_insert, left_idx_insert);
                break;
            }
        }
        path_cost += min_insert_cost;

        not_visited.erase(id_insert);

        bipedlab::debugger::debugColorOutput("[ECI-GEN] Best Insertion : ", id_insert, 5, BG);
        bipedlab::debugger::debugColorOutput("[ECI-GEN] Best Left Idx : ", left_idx_insert, 5, BW);
        bipedlab::debugger::debugColorOutput("[ECI-GEN] In Place : ", insertion_method, 5, BW);
        bipedlab::debugger::debugColorOutput("[ECI-GEN] Min Insert Cost : ", min_insert_cost, 5, BW);
        
        if (DEBUG_LEVEL <= 5)
        {
            for (auto id : sequence_best_insertion_)
            {
                std::cout << id << " -> ";
            }
            std::cout << "#" << std::endl;
        }        
    }
    min_path_cost_ = path_cost;
    sequence_rtsp_ = std::vector<int>(std::begin(sequence_best_insertion_), 
                                      std::end(sequence_best_insertion_));

    if (sequence_rtsp_.size() > (*distance_matrix_ptr_).size())
    {
        refineSequence_(sequence_rtsp_);
    }
}
    
double EciGenSolver::solveDijkstra_()
{
    bipedlab::debugger::debugColorTextOutput("[ECI-GEN] Solve Dijkstra", 6);
    std::unordered_map<int, int> parent = { {source_id_, source_id_} };
    std::unordered_map<int, double> cost_from_source = {
        {source_id_, 0}, {target_id_, INFINITY}
    };

    // { cost-to-come, destination_id }
    std::priority_queue<std::pair<double, int>> open_queue;
    open_queue.push(std::make_pair(0, source_id_));

    while (!open_queue.empty())
    {
        std::pair<double, int> state = open_queue.top();
        open_queue.pop();

        // Reached to the target
        if (state.second == target_id_)
        {
            break;
        }

        // Search for neighbor of state
        for (int i = 0; i < (int)(*distance_matrix_ptr_).size(); ++i)
        {
            if (state.second == i)
            {
                continue;
            }

            double new_cost = cost_from_source[state.second] + 
                              (*distance_matrix_ptr_)[state.second][i];

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
    extractSequence_(parent, source_id_, target_id_, sequence_dijkstra_);

    printDijkstraId_();

    return cost_from_source[target_id_];
}

std::pair<double, std::pair<int, Insertion_types_t>> EciGenSolver::bestInsertion_(int insert_id)
{
    double min_insert_cost = INFINITY;
    int best_left_idx;
    Insertion_types_t insertion_method;

    std::list<int>::iterator it = sequence_best_insertion_.begin();
    while (it != sequence_best_insertion_.end())
    {   
        double revisit_cost = 2 * (*distance_matrix_ptr_)[insert_id][*it];
        double sequence_cost = INFINITY;
        double sequence_swap_left_cost = INFINITY;
        double sequence_swap_right_cost = INFINITY;
        double sequence_swap_both_cost = INFINITY;

        // If the iterator is the last element, there is no option for in-sequence
        if (std::distance(it, sequence_best_insertion_.end()) == 1)
        {
            sequence_cost = revisit_cost;
        }
        else
        {
            auto nx = std::next(it, 1);
            sequence_cost = (*distance_matrix_ptr_)[*it][insert_id] +
                            (*distance_matrix_ptr_)[insert_id][*nx] -
                            (*distance_matrix_ptr_)[*it][*nx];

            if (setting_.swapping)
            {
                if (std::distance(sequence_best_insertion_.begin(), it) > 1)
                {
                    auto pv = std::prev(it, 1);
                    auto ppv = std::prev(it, 2);
                    sequence_swap_left_cost = (*distance_matrix_ptr_)[insert_id][*nx] -
                                              (*distance_matrix_ptr_)[*it][*nx] +
                                              (*distance_matrix_ptr_)[*pv][insert_id] +
                                              (*distance_matrix_ptr_)[*ppv][*it] -
                                              (*distance_matrix_ptr_)[*ppv][*pv];
                }
                if (std::distance(nx, sequence_best_insertion_.end()) > 2)
                {
                    auto nnx = std::next(nx, 1);
                    auto nnnx = std::next(nx, 2);
                    sequence_swap_right_cost = (*distance_matrix_ptr_)[*it][insert_id] -
                                               (*distance_matrix_ptr_)[*it][*nx] +
                                               (*distance_matrix_ptr_)[*nnx][insert_id]+
                                               (*distance_matrix_ptr_)[*nnnx][*nx] -
                                               (*distance_matrix_ptr_)[*nnnx][*nnx];
                }
                if (std::distance(sequence_best_insertion_.begin(), it) > 1 &&
                    std::distance(nx, sequence_best_insertion_.end()) > 2)
                {
                    auto pv = std::prev(it, 1);
                    auto ppv = std::prev(it, 2);
                    auto nnx = std::next(nx, 1);
                    auto nnnx = std::next(nx, 2);
                    sequence_swap_both_cost = (*distance_matrix_ptr_)[*pv][insert_id] +
                                              (*distance_matrix_ptr_)[*ppv][*it] -
                                              (*distance_matrix_ptr_)[*ppv][*pv] -
                                              (*distance_matrix_ptr_)[*it][*nx] +
                                              (*distance_matrix_ptr_)[*nnx][insert_id]+
                                              (*distance_matrix_ptr_)[*nnnx][*nx] -
                                              (*distance_matrix_ptr_)[*nnnx][*nnx];
                }
            }
        }

        if (revisit_cost < min_insert_cost)
        {
            best_left_idx = std::distance(sequence_best_insertion_.begin(), it);
            min_insert_cost = revisit_cost;
            insertion_method = Insertion_types_t::REVISIT;
        }
        
        if (sequence_cost < min_insert_cost)
        {
            best_left_idx = std::distance(sequence_best_insertion_.begin(), it);
            min_insert_cost = sequence_cost;
            insertion_method = Insertion_types_t::SEQUENCE;
        }

        if (setting_.swapping)
        {
            if (sequence_swap_left_cost < min_insert_cost)
            {
                best_left_idx = std::distance(sequence_best_insertion_.begin(), it);
                min_insert_cost = sequence_swap_left_cost;
                insertion_method = Insertion_types_t::SEQUENCE_SWAP_LEFT;
            }

            if (sequence_swap_right_cost < min_insert_cost)
            {
                best_left_idx = std::distance(sequence_best_insertion_.begin(), it);
                min_insert_cost = sequence_swap_right_cost;
                insertion_method = Insertion_types_t::SEQUENCE_SWAP_RIGHT;
            }

            if (sequence_swap_both_cost < min_insert_cost)
            {
                best_left_idx = std::distance(sequence_best_insertion_.begin(), it);
                min_insert_cost = sequence_swap_both_cost;
                insertion_method = Insertion_types_t::SEQUENCE_SWAP_BOTH;
            }
        }
        ++it;
    }
    bipedlab::debugger::debugColorOutput("[ECI-GEN] tmp Best Insertion : ", insert_id, 4, BY);
    bipedlab::debugger::debugColorOutput("[ECI-GEN] Best Left Idx : ", best_left_idx, 4, BW);
    bipedlab::debugger::debugColorOutput("[ECI-GEN] Insertion Method : ", insertion_method, 4, BW);
    bipedlab::debugger::debugColorOutput("[ECI-GEN] Min Insert Cost : ", min_insert_cost, 4, BW);

    return std::make_pair(min_insert_cost, std::make_pair(best_left_idx, insertion_method));
}

void EciGenSolver::insertRevisit_(std::list<int>& sequence, int value, int left_idx)
{
    auto it = sequence.begin();
    std::advance(it, left_idx);

    sequence.insert(it, value);
    sequence.insert(std::prev(it), *it);
}

void EciGenSolver::insertSequence_(std::list<int>& sequence, int value, int left_idx)
{
    auto it = sequence.begin();
    std::advance(it, left_idx + 1);

    sequence.insert(it, value);
}

void EciGenSolver::insertSequenceSwapLeft_(std::list<int>& sequence, int value, int left_idx)
{
    auto it = sequence.begin();
    std::advance(it, left_idx);
    auto pv = std::prev(it, 1);
    std::swap(*it, *pv);

    sequence.insert(std::next(it), value);
}

void EciGenSolver::insertSequenceSwapRight_(std::list<int>& sequence, int value, int left_idx)
{
    auto nx = sequence.begin();
    std::advance(nx, left_idx + 1);
    auto nnx = std::next(nx, 1);
    std::swap(*nx, *nnx);

    sequence.insert(nx, value);
}

void EciGenSolver::insertSequenceSwapBoth_(std::list<int>& sequence, int value, int left_idx)
{
    auto it = sequence.begin();
    std::advance(it, left_idx);
    auto pv = std::prev(it, 1);
    std::swap(*it, *pv);

    auto nx = sequence.begin();
    std::advance(nx, left_idx + 1);
    auto nnx = std::next(nx, 1);
    std::swap(*nx, *nnx);

    sequence.insert(nx, value);
}

void EciGenSolver::extractSequence_(const std::unordered_map<int, int>& parent,
    int source, int target, std::list<int>& sequence)
{
    sequence = {target};
    int state = target;

    while (state != source)
    {
        state = parent.at(state);
        sequence.push_back(state);
    }
    std::reverse(sequence.begin(), sequence.end());
}

double EciGenSolver::calculatePathCost_(const std::vector<int>& sequence) const
{
    double distance = 0;
    for (size_t i = 0; i < sequence.size() - 1; ++i)
    {
        distance += (*distance_matrix_ptr_)[sequence[i]][sequence[i+1]];
    }
    return distance;
}

void EciGenSolver::printRtspId_()
{
    int debug_level = 7;
    bipedlab::debugger::debugColorTextOutput(
        "[ECI-GEN] Sequence of id from RTSP", debug_level, BM);

    if (DEBUG_LEVEL <= debug_level)
    {
        for (auto id : sequence_rtsp_)
        {
            std::cout << id << " -> ";
        }
        std::cout << "#" << std::endl;
    }
}

void EciGenSolver::printDijkstraId_()
{
    int debug_level = 6;
    bipedlab::debugger::debugColorTextOutput(
        "[ECI-GEN] Sequence of id from Dijkstra", debug_level, BM);

    if (DEBUG_LEVEL <= debug_level)
    {
        for (auto id : sequence_dijkstra_)
        {
            std::cout << id << " -> ";
        }
        std::cout << "#" << std::endl;
    }
}

void EciGenSolver::printDistanceMatrix_(const int debug_level = 5)
{
    bipedlab::debugger::debugColorTextOutput("[ECI-GEN] Distance Matrix", debug_level);

    bipedlab::debugger::debugColorTextOutput("[ECI-GEN] Shortest Direct Path", debug_level, BB);
    if (DEBUG_LEVEL <= debug_level)
    {
        for (auto row : (*distance_matrix_ptr_))
        {
            for (auto item : row)
            {
                std::cout << item << " ";
            }
            std::cout << std::endl;
        }
    }
}

void EciGenSolver::solveGeneticAlgorithm_()
{
    generateChromosomes_();

    // Not execute when the number of chromosomes is less than 2
    if (chromosomes_.size() < 2)
    {
        return;
    }

    // Crossover
    static std::uniform_int_distribution<> rand_bool(0, 1);
    for (int gen = 0; gen < setting_.ga_generation && chromosomes_.size() >= 2; ++gen)
    {
        bipedlab::debugger::debugColorTextOutput("Generation: " + std::to_string(gen + 1), 7, BR);

        int offspring_count = 0;
        int min_path_idx = 0;
        double tmp_min_path_cost = min_path_cost_;

        std::unordered_set<size_t> hashed_chromosome_table;
        std::vector<double> tmp_fitness;
        std::vector<std::vector<int>> offsprings;
        double sum_ga_fitness = std::accumulate(ga_fitness_.begin(),
                                                ga_fitness_.end(), 0.0);
        std::uniform_real_distribution<> uni_fitness(0.0, sum_ga_fitness);
        for (int i = 0; i < setting_.ga_population; ++i)
        {
            // Select Parent A
            double rnd_a = uni_fitness(random_gen_);
            int a = 0;
            while (rnd_a >= 0)
            {
                rnd_a -= ga_fitness_[a++];
            }
            std::vector<int>& parent_a = chromosomes_[a - 1];

            // Select Parent B
            double rnd_b = uni_fitness(random_gen_);
            int b = 0;
            while (rnd_b >= 0)
            {
                rnd_b -= ga_fitness_[b++];
            }
            std::vector<int>& parent_b = chromosomes_[b - 1];

            // Generate location to slice parent A
            std::uniform_int_distribution<> uni_sequence(1, parent_a.size() - 1);
            int slice_1 = uni_sequence(random_gen_);
            int slice_2 = uni_sequence(random_gen_);

            if (slice_2 < slice_1)
            {
                std::swap(slice_1, slice_2);
            }
            std::unordered_set<int> sub_parent_a_set(parent_a.begin() + slice_1,
                                                     parent_a.begin() + slice_2);

            // Generate starting location to paste sliced part of parent A
             std::uniform_int_distribution<> uni_offset(1 - slice_1, 
                    std::max(parent_a.size(), parent_b.size()) - slice_2 - 1);
            int offset = uni_offset(random_gen_);

            // Execute Crossover (Parent_B + Sliced_part_of_Parent_A + Parent_B)
            std::vector<int> offspring;
            offspring.push_back(source_id_);
            int idx = 1;
            int b_idx = 1;

            while (idx < offset + slice_1 && b_idx < (int)parent_b.size() - 1)
            {
                if (!sub_parent_a_set.count(parent_b[b_idx]))
                {
                    offspring.push_back(parent_b[b_idx]);
                    idx++;
                }
                b_idx++;
            }

            if (rand_bool(random_gen_)) // Paste parts of Parent A in order
            {
                for (idx = offset + slice_1; idx < offset + slice_2; ++idx)
                {
                    offspring.push_back(parent_a[idx - offset]);
                }   
            }
            else // Paste parts of Parent A in reverse order
            {
                for (idx = offset + slice_2 - 1; idx > offset + slice_1 - 1; --idx)
                {
                    offspring.push_back(parent_a[idx - offset]);
                }
            }

            idx = offset + slice_2;
            while (idx >= offset + slice_2 && b_idx < (int)parent_b.size() - 1)
            {
                if (!sub_parent_a_set.count(parent_b[b_idx]))
                {
                    offspring.push_back(parent_b[b_idx]);
                    idx++;
                }
                b_idx++;
            }

            if (offspring.back() != target_id_)
            {
                offspring.push_back(target_id_);
            }
            // Finish Crossover

            // Refine Chromosome removing consecutive duplicate node (Incomplete Graph)
            // Example: 0-0-1-2-3-4-4  =>  0-1-2-3-4
            // Example: 0-2-5-1-5-1-4-3-4-3-6 => 0-2-5-1-4-3-6
            if (offspring.size() > (*distance_matrix_ptr_).size())
            {
                refineSequence_(offspring);
            }
            
            // Prevent Duplicate Chromosome
            size_t hashed_chromosome = hashing_chromosome(offspring);

            if (hashed_chromosome_table.count(hashed_chromosome))
            {
                continue;
            }
            hashed_chromosome_table.insert(hashed_chromosome);

            // Save offspring and it's fitness
            // Only save the offspring that have shorter path than parent
            double path_cost = calculatePathCost_(offspring);

            if (path_cost <= min_path_cost_ * 1.1)
            {
                tmp_fitness.push_back(1 / path_cost);
                offsprings.push_back(offspring);

                if (path_cost < tmp_min_path_cost)
                {
                    tmp_min_path_cost = path_cost;
                    min_path_idx = offspring_count;
                }
                offspring_count++;
            }
        }

        ga_fitness_ = tmp_fitness;
        chromosomes_ = offsprings;

        // Update sequence_rtsp to the chromosome that have minimum path cost
        if (chromosomes_.size() >= 1)
        {
            min_path_cost_ = tmp_min_path_cost;
            sequence_rtsp_ = chromosomes_[min_path_idx];
        }
    }
}

void EciGenSolver::generateChromosomes_()
{
    std::vector<int>& original = sequence_rtsp_;
    int chromosome_count = 0;
    int min_path_idx = 0;
    double tmp_min_path_cost = min_path_cost_;

    std::unordered_set<size_t> hashed_chromosome_table;
    std::uniform_int_distribution<> uni_sequence(1, (int)sequence_rtsp_.size() - 1);
    static std::uniform_int_distribution<> mutate_reverse_options(0, 7);
    static std::uniform_int_distribution<> mutate_order_options(0, 4);
    for (int i = 0; i < setting_.ga_mutation_iter; ++i)
    {
        // Generate Position to slice the Sequence
        std::vector<int> slice(4);
        for (int j = 0; j < 4; ++j)
        {
            slice[j] = uni_sequence(random_gen_);
        }
        std::sort(slice.begin(), slice.end());

        // Mutate the original sequence
        auto part1 = std::vector<int>(original.begin(), original.begin() + slice[0]);
        auto part2 = std::vector<int>(original.begin() + slice[0], original.begin() + slice[1]);
        auto part3 = std::vector<int>(original.begin() + slice[1], original.begin() + slice[2]);
        auto part4 = std::vector<int>(original.begin() + slice[2], original.begin() + slice[3]);
        auto part5 = std::vector<int>(original.begin() + slice[3], original.end());

        // Decide Mutate Reverse option radomly
        int mutate_reverse_option = mutate_reverse_options(random_gen_);
        switch (mutate_reverse_option)
        {
            case 0:
                break;
            case 1:
                std::reverse(part2.begin(), part2.end());
                break;
            case 2:
                std::reverse(part3.begin(), part3.end());
                break;
            case 3:
                std::reverse(part4.begin(), part4.end());
                break;
            case 4:
                std::reverse(part2.begin(), part2.end());
                std::reverse(part3.begin(), part3.end());
                break;
            case 5:
                std::reverse(part3.begin(), part3.end());
                std::reverse(part4.begin(), part4.end());
                break;
            case 6:
                std::reverse(part2.begin(), part2.end());
                std::reverse(part4.begin(), part4.end());
                break;
            case 7:
                std::reverse(part2.begin(), part2.end());
                std::reverse(part3.begin(), part3.end());
                std::reverse(part4.begin(), part4.end());
                break;
            default :
                break;
        }

        // Decide Mutate Order option radomly and Execute mutation
        std::vector<int> chromosome;
        chromosome.reserve(sequence_best_insertion_.size());

        int mutate_order_option = mutate_order_options(random_gen_);
        switch (mutate_order_option)
        {
            case 0:
                joinChromosomeParts_(part1, part2, part4, part3, part5, chromosome);
                break;
            case 1:
                joinChromosomeParts_(part1, part3, part2, part4, part5, chromosome);
                break;
            case 2:
                joinChromosomeParts_(part1, part3, part4, part2, part5, chromosome);
                break;
            case 3:
                joinChromosomeParts_(part1, part4, part2, part3, part5, chromosome);
                break;
            case 4:
                joinChromosomeParts_(part1, part4, part3, part2, part5, chromosome);
                break;
            default :
                break;
        }

        // Refine Chromosome removing redundant node (Incomplete Graph)
        // Example: 0-0-1-2-3-4-4  =>  0-1-2-3-4
        // Example: 0-2-5-1-5-1-4-3-4-3-6 => 0-2-5-1-4-3-6

        if (chromosome.size() > (*distance_matrix_ptr_).size())
        {
            refineSequence_(chromosome);
        }

        // Prevent Duplicate Chromosome
        size_t hashed_chromosome = hashing_chromosome(chromosome);
        if (hashed_chromosome_table.count(hashed_chromosome))
        {
            continue;
        }
        hashed_chromosome_table.insert(hashed_chromosome);

        // Only save the sequence that have shorter path than sequence_best_insertion
        double path_cost = calculatePathCost_(chromosome);
        if (path_cost <= min_path_cost_ * 1.1)
        {
            ga_fitness_.push_back(1 / path_cost);
            chromosomes_.push_back(chromosome);

            if (path_cost < tmp_min_path_cost)
            {
                tmp_min_path_cost = path_cost;
                min_path_idx = chromosome_count;
            }
            chromosome_count++;
        }
    }
    // Update sequence_rtsp to the chromosome that have minimum path cost
    if (chromosomes_.size() >= 1)
    {
        min_path_cost_ = tmp_min_path_cost;
        sequence_rtsp_ = chromosomes_[min_path_idx];
    }
}

void EciGenSolver::joinChromosomeParts_(std::vector<int>& part1, std::vector<int>& part2,
    std::vector<int>& part3, std::vector<int>& part4, std::vector<int>& part5,
    std::vector<int>& chromosome)
{
    chromosome.insert(chromosome.end(), part1.begin(), part1.end());
    chromosome.insert(chromosome.end(), part2.begin(), part2.end());
    chromosome.insert(chromosome.end(), part3.begin(), part3.end());
    chromosome.insert(chromosome.end(), part4.begin(), part4.end());
    chromosome.insert(chromosome.end(), part5.begin(), part5.end());
}

size_t EciGenSolver::hashing_chromosome(const std::vector<int>& chromosome)
{
    size_t hashed_chromosome = chromosome.size();
    for (int i : chromosome)
    {
        hashed_chromosome ^= i + 0x9e3779b9 + (hashed_chromosome << 6) + 
                             (hashed_chromosome >> 2);
    }
    return hashed_chromosome;
}

void EciGenSolver::printSequence_(const std::vector<int>& sequence,
        const std::string& name, int debug_level) const
{
    bipedlab::debugger::debugColorTextOutput(name, debug_level, BB);

    if (DEBUG_LEVEL <= debug_level)
    {
        for (auto item : sequence)
        {
            std::cout << item << "-";
        }
        std::cout << " Cost: " << calculatePathCost_(sequence) << std::endl;
    }
}

void EciGenSolver::refineSequence_(std::vector<int>& sequence)
{
    std::unordered_map<int, int> last_idx;
    int idx = 0;
    for (auto item : sequence)
    {
        last_idx[item] = idx++;
    }

    printSequence_(sequence, "Mutate", 5);
    std::unordered_set<int> visited_node;
    std::vector<int> refined_sequence;
    for (auto it = sequence.begin(); it != std::prev(sequence.end()); ++it)
    {
        if (visited_node.count(*it))
        {
            bool shortcut = false;
            auto nx = std::next(it);
            while (std::isinf((*distance_matrix_ptr_)[refined_sequence.back()][*nx]) &&
                nx != std::prev(sequence.end()) &&
                (visited_node.count(*nx) || std::distance(sequence.begin(), nx) < last_idx[*nx]))
            {
                std::advance(nx, 1);

                if (!visited_node.count(*nx) &&
                    !std::isinf((*distance_matrix_ptr_)[refined_sequence.back()][*nx]))
                {
                    refined_sequence.push_back(*nx);
                    visited_node.insert(*nx);
                    shortcut = true;

                    it = --nx;
                }
            }

            if (!shortcut && (refined_sequence.back() != *it))
            {
                refined_sequence.push_back(*it);
                visited_node.insert(*it);
            }
        }
        else
        {
            refined_sequence.push_back(*it);
            visited_node.insert(*it);
        }
        printSequence_(refined_sequence, "refined", 5);
    }

    if (refined_sequence.back() != sequence.back())
    {
        refined_sequence.push_back(sequence.back());
    }

    sequence = refined_sequence;
}
