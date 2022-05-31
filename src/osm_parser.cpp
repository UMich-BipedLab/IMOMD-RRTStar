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
 * File:        osm_parser.cpp
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     02/24/2022
 * 
 * Description: Parser Open Street Map(xml) to vector data in c++
*******************************************************************************/
#include "osm_converter/osm_parser.h"
#include "utils/utils.h"

std::shared_ptr<std::vector<location_t>> OSMParser::getMap()
{
    return std::make_shared<std::vector<location_t>>(nodes_);
}

std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> OSMParser::getConnection()
{
    return std::make_shared<std::vector<std::unordered_map<size_t, double>>>(connection_);
}

void OSMParser::parse()
{
    tinyxml2::XMLDocument doc;

    if (doc.LoadFile(xml_.c_str()) == tinyxml2::XML_SUCCESS)
    {
        bipedlab::debugger::debugColorOutput("[OSMParser] Loaded OSM file: ", xml_, 10, G);
    }
    else
    {
        throw std::runtime_error("Failed to load xml: " + xml_);
    }

    tinyxml2::XMLElement* osm = doc.FirstChildElement();

    tinyxml2::XMLElement* node_element = osm->FirstChildElement("node");
    tinyxml2::XMLElement* way_element = osm->FirstChildElement("way");

    tinyxml2::XMLHandle h_root_node = tinyxml2::XMLHandle(node_element);
    tinyxml2::XMLHandle h_root_way = tinyxml2::XMLHandle(way_element);

    createWays_(&h_root_way, &h_root_node);
    createNodes_();
    createNetwork_();

    bipedlab::debugger::debugTitleTextOutput("[OSMParser]",
        "Create Network (Nodes : " + std::to_string(nodes_.size()) +
        ") (Edges : " + std::to_string(edge_number_) + ")", 10, BG);
}

void OSMParser::createWays_(tinyxml2::XMLHandle* h_root_way, tinyxml2::XMLHandle* h_root_node)
{   
    tinyxml2::XMLElement* node_element = h_root_node->ToElement();
    location_t location_tmp;

    size_t node_id;
    for (; node_element; node_element = node_element->NextSiblingElement("node"))
    {
        std::sscanf(node_element->Attribute("id"), "%zu", &node_id);
        location_tmp.latitude = node_element->DoubleAttribute("lat");
        location_tmp.longitude = node_element->DoubleAttribute("lon");
        
        nodes_map_.insert({node_id, location_tmp});
    }

    ways_.clear();
    tinyxml2::XMLElement* way_element = h_root_way->ToElement();
    osm_way_t way_tmp;
    tinyxml2::XMLElement* tag;

    for (; way_element; way_element = way_element->NextSiblingElement("way"))
    {
        tag = way_element->FirstChildElement("tag");
        std::sscanf(way_element->Attribute("id"), "%zu", &way_tmp.id);

        while (tag != NULL)
        {
            if (filterWay_(tag))
            {
                //finding all nodes located in selected way
                getNodesInWay_(way_element, way_tmp); 
                ways_.push_back(way_tmp);
                break;
            }
            tag = tag->NextSiblingElement("tag");
        }
    }
}

bool OSMParser::filterWay_(tinyxml2::XMLElement* tag)
{
    std::string key(tag->Attribute("k"));
    std::string value(tag->Attribute("v"));

    if (map_properties_.key.count(key) || map_properties_.value.count(value))
    {
        return true;
    }
    return false;
}

void OSMParser::getNodesInWay_(tinyxml2::XMLElement* way_element, osm_way_t& way)
{
    tinyxml2::XMLElement* node_element;

    node_element = way_element->FirstChildElement("nd");
    tinyxml2::XMLHandle h_root_node(node_element);
    node_element = h_root_node.ToElement();

    way.nodes_id.clear();
    size_t node_id;
    for (; node_element; node_element = node_element->NextSiblingElement("nd"))
    {
        std::sscanf(node_element->Attribute("ref"), "%zu", &node_id);
        way.nodes_id.push_back(node_id);
        nodes_id_idx_table_[node_id] = 0;
    }
}

void OSMParser::createNodes_()
{
    nodes_.clear();
    nodes_.resize(nodes_id_idx_table_.size());

    size_t idx = 0;
    for (const auto& node_id_idx : nodes_id_idx_table_)
    {
        // Setting ID of location_t as index of vector 'nodes_'
        nodes_map_[node_id_idx.first].id = idx;
        nodes_[idx] = nodes_map_[node_id_idx.first];
        nodes_id_idx_table_[node_id_idx.first] = idx++;
    }
}

void OSMParser::createNetwork_()
{
    connection_.clear();
    connection_.resize(nodes_.size());

    double distance = 0;

    for (const auto& way : ways_)
    {
        for (std::size_t i = 0; i < way.nodes_id.size() - 1; ++i)
        {
            distance = computeHaversineDistance(
                nodes_map_[way.nodes_id[i]], nodes_map_[way.nodes_id[i+1]]);

            size_t left_node_idx = nodes_id_idx_table_[way.nodes_id[i]];
            size_t right_node_idx = nodes_id_idx_table_[way.nodes_id[i+1]];

            connection_[left_node_idx].insert({right_node_idx, distance});
            connection_[right_node_idx].insert({left_node_idx, distance});
            edge_number_++;
        }
    }
}
