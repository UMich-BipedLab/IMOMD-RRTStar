/*******************************************************************************
 * File:        osm_parser.h
 * 
 * Author:      Dongmyeong Lee (dongmyeong[at]umich.edu)
 * Created:     02/24/2022
 * 
 * Description: Parsing Open Street Map(OSM) to vector data in c++
*******************************************************************************/
#ifndef OSM_PARSER_H
#define OSM_PARSER_H

#include <tinyxml.h>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <math.h>
#include <iostream>
#include <sstream>

#include "../location.h"
#include "../utils/debugger.h"

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

    OSMParser();
    OSMParser(std::string xml) : xml_(xml) {}

    void parse();
    std::shared_ptr<std::vector<location_t>> getMap();
    std::shared_ptr<std::vector<std::vector<std::pair<int, double>>>> getConnection();

private:
    std::string xml_;

    std::vector<location_t> nodes_;
    std::unordered_map<size_t, location_t> nodes_map_;
    std::unordered_map<size_t, size_t> nodes_id_idx_table_;

    std::vector<osm_way_t> ways_;
    std::vector<std::vector<std::pair<int, double>>> connection_;

    void createWays_(TiXmlHandle* h_root_way, TiXmlHandle* h_root_node);
    bool isHighway_(TiXmlElement* tag);
    void getNodesInWay_(TiXmlElement* way_element, osm_way_t& way);
    void createNodes_();
    void createNetwork_();

    double getDistanceFromNodes_(const location_t& node1, const location_t& node2);
}; /* OSMParser */
#endif /* OSM_PARSER_H */