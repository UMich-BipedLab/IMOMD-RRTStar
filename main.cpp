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
 * File:        main.cpp
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     02/20/2022
 * 
 * Description: Main Script for executing robot path planning
*******************************************************************************/
#include <unordered_set>
#include <stdexcept>

#include "imomd_rrt_star/imomd_rrt_star.h"
#include "baseline/bi_a_star.h"
#include "baseline/ana_star.h"

#include "osm_converter/osm_parser.h"

#include "fake_map/fake_map.h"
#include "setting/imomd_setting_t.h"
#include "setting/rtsp_setting_t.h"
#include "utils/utils.h"
#include "yaml/yaml_utils.h"

#define RYML_SINGLE_HDR_DEFINE_NOW
#include "yaml/yaml.h"

int DEBUG_LEVEL = 10;

int main()
{
    // load algorithm_config
    std::string contents = bipedlab::yaml_utils::getFileContents(
            "config/algorithm_config.yaml");
    ryml::Tree tree = ryml::parse_in_arena(ryml::to_csubstr(contents));

    // assign Map & Graph
    bipedlab::debugger::debugColorTextOutput(
            "[main] Building/Loading the map", 10, BC);
    int map_type = std::stoi(bipedlab::yaml_utils::convertToStr(
                    tree["map"]["type"].val()));

    std::shared_ptr<std::vector<location_t>> raw_map;
    std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> graph;

    if (map_type < 0)
    {
        bipedlab::debugger::debugColorOutput(
                "[main] Using customized prebuilt map: ", map_type, 10, BC);
        auto fake_map = FakeMap(map_type);

        raw_map = fake_map.getMap();
        graph = fake_map.getGraph();
    }
    else if (map_type == 0)
    {
        bipedlab::debugger::debugColorTextOutput(
            "[main] Write your own parser " 
            "to parse your own map to our graph strucure.", 10, BC);

        bipedlab::debugger::debugColorOutput(
            "Please take a look at fake_map.h for example", map_type, 10, BC);
        exit(0);

    }
    else if (map_type == 1)
    {
        bipedlab::debugger::debugColorOutput(
                "[main] Loading OpenStreetMap: ", map_type, 10, BC);

        // Get OSM file
        std::string map_path = bipedlab::yaml_utils::convertToStr(
                            tree["map"]["path"].val());
        std::string map_file = bipedlab::yaml_utils::convertToStr(
                            tree["map"]["name"].val());

        // filtering OSM Way Type
        map_properties_t map_properties;
        std::string osm_config = bipedlab::yaml_utils::getFileContents(
            "config/osm_way_config.yaml");
        ryml::Tree osm_config_tree = ryml::parse_in_arena(ryml::to_csubstr(osm_config));

        int filter = 0;
        filter = std::stoi(bipedlab::yaml_utils::convertToStr(
                                osm_config_tree["osm_show_ways"].val()));

        switch (filter)
        {
            case 0:
                map_properties.key = bipedlab::yaml_utils::convertToUnorderedSet<std::string>(
                                        osm_config_tree["osm_all"]["key"]);
                break;
            case 1:
                map_properties.key = bipedlab::yaml_utils::convertToUnorderedSet<std::string>(
                                        osm_config_tree["osm_cars"]["key"]);
                map_properties.value = bipedlab::yaml_utils::convertToUnorderedSet<std::string>(
                                        osm_config_tree["osm_cars"]["value"]);
                break;
            case 2:
                map_properties.key = bipedlab::yaml_utils::convertToUnorderedSet<std::string>(
                                        osm_config_tree["osm_walkers"]["key"]);
                map_properties.value = bipedlab::yaml_utils::convertToUnorderedSet<std::string>(
                                        osm_config_tree["osm_walkers"]["value"]);
                break;
            case 3:
                map_properties.key = bipedlab::yaml_utils::convertToUnorderedSet<std::string>(
                                        osm_config_tree["osm_cyclists"]["key"]);
                map_properties.value = bipedlab::yaml_utils::convertToUnorderedSet<std::string>(
                                        osm_config_tree["osm_cyclists"]["value"]);
                break;
            default:
                throw std::invalid_argument("Invalid OSM Type variable! Check config file!");
        }

        OSMParser osm_parser(map_path + map_file, map_properties);
        osm_parser.parse();

        raw_map = osm_parser.getMap();
        graph = osm_parser.getConnection();
    }
    else
    {
        throw std::invalid_argument("Invalid Map Type variable! Check config file!");
    }
    // End assiging map & graph

    // Assign Algorithm (IMOMD vs. BaseLine)
    int system = std::stoi(bipedlab::yaml_utils::convertToStr(
                tree["general"]["system"].val()));

    int flag_print_path = std::stoi(bipedlab::yaml_utils::convertToStr(
                tree["general"]["print_path"].val()));

    // assign source, target, and objectives as indices
    size_t source = std::stoi(bipedlab::yaml_utils::convertToStr(
                tree["destinations"]["source_id"].val()));
    size_t target = std::stoi(bipedlab::yaml_utils::convertToStr(
                tree["destinations"]["target_id"].val()));
    std::vector<size_t> objectives = 
        bipedlab::yaml_utils::convertToVector<size_t>(tree["destinations"]["objective_ids"]);

    bipedlab::debugger::debugColorOutput("[main] Source: ", source, 10, BC);
    bipedlab::debugger::debugColorOutput("[main] Target: ", target, 10, BC);
    bipedlab::debugger::debugColorContainerOutput("[main] Objectives: ", objectives, 
                                                  10, BC);

    // rrt parameters and settings
    imomd_setting_t setting;

    setting.max_iter = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["general"]["max_iter"].val()));

    setting.max_time = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["general"]["max_time"].val()));

    setting.pseudo_mode = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["general"]["pseudo"].val()));

    setting.log_data = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["general"]["log_data"].val()));

    setting.goal_bias = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["rrt_params"]["goal_bias"].val()));

    setting.random_seed = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["rrt_params"]["random_seed"].val()));

    // RTSP Solver Settings
    setting.rtsp_setting.swapping = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["rtsp_settings"]["swapping"].val()));

    setting.rtsp_setting.genetic = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["rtsp_settings"]["genetic"].val()));

    setting.rtsp_setting.ga_random_seed = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["rtsp_settings"]["ga"]["random_seed"].val()));
    
    setting.rtsp_setting.ga_mutation_iter = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["rtsp_settings"]["ga"]["mutation_iter"].val()));

    setting.rtsp_setting.ga_population = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["rtsp_settings"]["ga"]["population"].val()));

    setting.rtsp_setting.ga_generation = std::stoi(bipedlab::yaml_utils::convertToStr(
                                    tree["rtsp_settings"]["ga"]["generation"].val()));

    // call the algorithm
    pthread_t tid[2];

    switch (system)
    {
        case 0:
        {
            ImomdRRT imomt(source, target, objectives, raw_map, graph, setting);

            bipedlab::debugger::debugTitleTextOutput("[main]", "IMOMD running", 10, BG);
            pthread_create(&tid[0], NULL, ImomdRRT::findShortestPath, &imomt);

            if (flag_print_path)
            {
                pthread_create(&tid[1], NULL, ImomdRRT::printPath, &imomt);
            }

            pthread_join(tid[0], NULL);
            
            if (flag_print_path)
            {
                pthread_join(tid[1], NULL); 
            }

            exit(0);
        }
        case 1:
        {
            BiAstar bi_astar(source, target, objectives, raw_map, graph, setting);

            bipedlab::debugger::debugTitleTextOutput("[main]", "Bi-A* running", 10, BG);
            pthread_create(&tid[0], NULL, BiAstar::findShortestPath, &bi_astar);
            
            if (flag_print_path)
            {
                pthread_create(&tid[1], NULL, BiAstar::printPath, &bi_astar);
            }

            pthread_join(tid[0], NULL);
            
            if (flag_print_path)
            {
                pthread_join(tid[1], NULL); 
            }

            exit(0);
        }
        case 2:
        {
            ANAStar ana_star(source, target, objectives, raw_map, graph, setting);

            bipedlab::debugger::debugTitleTextOutput("[main]", "ANA* running", 10, BG);
            pthread_create(&tid[0], NULL, ANAStar::findShortestPath, &ana_star);
            if (flag_print_path)
            {
                pthread_create(&tid[1], NULL, ANAStar::printPath, &ana_star);
            }

            pthread_join(tid[0], NULL);
            
            if (flag_print_path)
            {
                pthread_join(tid[1], NULL); 
            }

            exit(0);
        }
    }
}
