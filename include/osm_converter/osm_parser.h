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
 * File:        osm_parser.h
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     02/24/2022
 * 
 * Description: Parsing Open Street Map(OSM) to vector data in c++
*******************************************************************************/
#ifndef OSM_PARSER_H
#define OSM_PARSER_H

#include "tinyxml2/tinyxml2.h"
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <math.h>
#include <iostream>
#include <sstream>

#include "osm_converter/compute_haversine.hpp"

#include "utils/debugger.h"

#include "data/location_t.h"

#include "setting/map_setting_t.h"

class OSMParser
{
public:
    typedef struct osm_node
    {
        size_t id;
        double latitude;
        double longitude;
    } osm_node_t;

    typedef struct osm_way
    {
        size_t id;
        std::vector<size_t> nodes_id;
    } osm_way_t;

    // Constructor
    OSMParser();
    OSMParser(std::string xml, map_properties_t map_properties):
        xml_(xml), map_properties_(map_properties), edge_number_(0) {}

    // Parse OSM data into nodes and edges
    void parse();
    
    // get pointer of nodes of map
    std::shared_ptr<std::vector<location_t>> getMap();

    // get pointer of edges of map
    std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> getConnection();

private:
    std::string xml_;
    map_properties_t map_properties_;

    std::vector<location_t> nodes_;
    std::unordered_map<size_t, location_t> nodes_map_;
    std::unordered_map<size_t, size_t> nodes_id_idx_table_;

    std::vector<osm_way_t> ways_;
    std::vector<std::unordered_map<size_t, double>> connection_;

    int edge_number_;

    // Parse way in OSM data
    void createWays_(tinyxml2::XMLHandle* h_root_way, tinyxml2::XMLHandle* h_root_node);
    
    // Filter out based on osm_way_config file
    bool filterWay_(tinyxml2::XMLElement* tag);
    
    // Stack nodes in the way
    void getNodesInWay_(tinyxml2::XMLElement* way_element, osm_way_t& way);
    
    // Generate nodes of map and re-index the nodes 
    void createNodes_();
    
    // Generate edges of map and compute haversine distance of the edge
    void createNetwork_();
}; /* OSMParser */
#endif /* OSM_PARSER_H */
