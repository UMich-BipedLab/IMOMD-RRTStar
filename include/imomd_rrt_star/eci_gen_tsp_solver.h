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
 * File:        eci_gen_tsp_solver.h
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     03/10/2022
 * 
 * Description: Solving Relaxed Traveling Salesman Problem with 
 *              Enhanced Cheapest Insertion Method with Genetic Algorithm
*******************************************************************************/
#ifndef ECI_GEN_TSP_H
#define ECI_GEN_TSP_H

#include <memory>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>
#include <math.h>
#include <iterator>
#include <algorithm>
#include <vector>
#include <random>

#include "setting/rtsp_setting_t.h"
#include "utils/debugger.h"

enum Insertion_types_t
{
    REVISIT,
    SEQUENCE,
    SEQUENCE_SWAP_LEFT,
    SEQUENCE_SWAP_RIGHT,
    SEQUENCE_SWAP_BOTH,
};

class EciGenSolver
{
public:
    EciGenSolver() = default;
    EciGenSolver(const rtsp_setting_t& setting);

    std::pair<double, std::shared_ptr<std::vector<int>>> solveRTSP(
        std::shared_ptr<std::vector<std::vector<double>>> adjacency_matrix_ptr,
        int source_id, int target_id);
    
    std::pair<double, std::shared_ptr<std::vector<int>>> solveDijkstra(
        std::shared_ptr<std::vector<std::vector<double>>> adjacency_matrix_ptr,
        int source_id, int target_id);

private:
    rtsp_setting_t setting_;
    std::mt19937 random_gen_;

    std::shared_ptr<std::vector<std::vector<double>>> distance_matrix_ptr_;
    int source_id_;
    int target_id_;

    double min_path_cost_;
    std::vector<int> sequence_rtsp_;
    std::list<int> sequence_best_insertion_;
    std::list<int> sequence_dijkstra_;

    std::vector<double> ga_fitness_;
    std::vector<std::vector<int>> chromosomes_;

    void getShortestAdjacencyMatrix_();

    void solveBestInsertion_();

    double solveDijkstra_();

    std::pair<double, std::pair<int, Insertion_types_t>> bestInsertion_(int insert_id);

    void insertRevisit_(std::list<int>& sequence, int value, int left_idx);

    void insertSequence_(std::list<int>& sequence, int value, int left_idx);

    void insertSequenceSwapLeft_(std::list<int>& sequence, int value, int left_idx);

    void insertSequenceSwapRight_(std::list<int>& sequence, int value, int left_idx);

    void insertSequenceSwapBoth_(std::list<int>& sequence, int value, int left_idx);

    void extractSequence_(const std::unordered_map<int, int>& parent,
        int source, int target, std::list<int>& sequence);

    double calculatePathCost_(const std::vector<int>& sequence) const;

    void printRtspId_();

    void printDijkstraId_();

    void printDistanceMatrix_(const int debug_level);

    void printSequence_(const std::vector<int>& sequence,
        const std::string& name, int debug_level) const;

    void solveGeneticAlgorithm_();

    void generateChromosomes_();

    void joinChromosomeParts_(std::vector<int>& part1, std::vector<int>& part2,
        std::vector<int>& part3, std::vector<int>& part4,
        std::vector<int>& part5, std::vector<int>& chromosome);

    void mutateGA_(std::list<int> sequence);

    size_t hashing_chromosome(const std::vector<int>& chromosome);

    void refineSequence_(std::vector<int>& sequence);
};

#endif /* ECI_GEN_TSP_H */
