/*******************************************************************************
 * File:        osm_converter.cpp
 * 
 * Author:      Dongmyeong Lee (dongmyeong[at]umich.edu)
 * Created:     02/24/2022
 * 
 * Description: Parser Open Street Map(xml) to vector data in c++
*******************************************************************************/
#include "../include/osm_parser/osm_parser.h"

std::shared_ptr<std::vector<location_t>> OSMParser::getMap()
{
    return std::make_shared<std::vector<location_t>>(nodes_);
}

std::shared_ptr<std::vector<std::vector<std::pair<int, double>>>> OSMParser::getConnection()
{
    return std::make_shared<std::vector<std::vector<std::pair<int, double>>>>(connection_);
}

void OSMParser::parse()
{
    TiXmlDocument doc(xml_);
    TiXmlNode* osm;
    TiXmlNode* node;
    TiXmlNode* way;

    TiXmlHandle h_root_node(0);
    TiXmlHandle h_root_way(0);

    bool loadOkay = doc.LoadFile();

    if (loadOkay)
    {
        bipedlab::debugger::debugColorOutput("Load OSM file: ", xml_, 10);
    }
    else
    {
        throw std::runtime_error("Failed to load xml : " + xml_);
    }

    osm = doc.FirstChildElement();
    node = osm->FirstChild("node");
    way = osm->FirstChild("way");

    TiXmlElement* node_element = node->ToElement();
    TiXmlElement* way_element = way->ToElement();

    h_root_node = TiXmlHandle(node_element);
    h_root_way = TiXmlHandle(way_element);

    createWays_(&h_root_way, &h_root_node);
    createNodes_();
    createNetwork_();
}

void OSMParser::createWays_(TiXmlHandle* h_root_way, TiXmlHandle* h_root_node)
{   
    TiXmlElement* node_element = h_root_node->Element();
    location_t location_tmp;

    size_t node_id;
    for (node_element; node_element; node_element = node_element->NextSiblingElement("node"))
    {
        std::sscanf(node_element->Attribute("id"), "%zu", &node_id);
        node_element->Attribute("lat", &location_tmp.latitude);
        node_element->Attribute("lon", &location_tmp.longitude);
        
        nodes_map_.insert({node_id, location_tmp});
    }

    ways_.clear();
    TiXmlElement* way_element = h_root_way->Element();
    osm_way_t way_tmp;
    TiXmlElement* tag;

    for (way_element; way_element; way_element = way_element->NextSiblingElement("way"))
    {
        tag = way_element->FirstChildElement("tag");
        std::sscanf(way_element->Attribute("id"), "%zu", &way_tmp.id);

        while (tag != NULL)
        {
            if (isHighway_(tag))
            {
                getNodesInWay_(way_element, way_tmp); //finding all nodes located in selected way
                ways_.push_back(way_tmp);
                break;
            }
            tag = tag->NextSiblingElement("tag");
        }
    }
}

bool OSMParser::isHighway_(TiXmlElement* tag)
{
    std::string key(tag->Attribute("k"));

    return (key == "highway");
}

void OSMParser::getNodesInWay_(TiXmlElement* way_element, osm_way_t& way)
{
    TiXmlElement* node_element;

    node_element = way_element->FirstChild("nd")->ToElement();
    TiXmlHandle h_root_node(node_element);
    node_element = h_root_node.Element();

    way.nodes_id.clear();
    size_t node_id;
    for (node_element; node_element; node_element = node_element->NextSiblingElement("nd"))
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
        nodes_map_[node_id_idx.first].id = idx; // Setting ID of location_t as index of vector 'nodes_'
        nodes_[idx] = nodes_map_[node_id_idx.first];
        nodes_id_idx_table_[node_id_idx.first] = idx++;
    }
}

void OSMParser::createNetwork_()
{
    connection_.clear();
    connection_.resize(nodes_.size());

    double distance = 0;
    bipedlab::debugger::debugTitleTextOutput("[OSM Parser]",
        "Create Network (Nodes : " + std::to_string(nodes_id_idx_table_.size()) +
        ") (Ways : " + std::to_string(ways_.size()) + ")", 10);

    for (const auto& way : ways_)
    {
        for (std::size_t i = 0; i < way.nodes_id.size() - 1; ++i)
        {
            distance = getDistanceFromNodes_(nodes_map_[way.nodes_id[i]],
                                             nodes_map_[way.nodes_id[i+1]]);

            int left_node_idx = nodes_id_idx_table_[way.nodes_id[i]];
            int right_node_idx = nodes_id_idx_table_[way.nodes_id[i+1]];

            connection_[left_node_idx].push_back({right_node_idx, distance});
            connection_[right_node_idx].push_back({left_node_idx, distance});
        }
    }
}

double OSMParser::getDistanceFromNodes_(const location_t& node1,
                                        const location_t& node2)
{
    static constexpr double DEG2RAD = M_PI / 180;
    static const int EARTH_RADIUS = 6371e3; // in metres;

    double lat1_rad = node1.latitude * DEG2RAD;
    double lat2_rad = node2.latitude * DEG2RAD; 
    double diff_lat_rad = (node2.latitude - node1.latitude) * DEG2RAD;
    double diff_lon_rad = (node2.longitude - node1.longitude) * DEG2RAD;

    // 'Harversine' Formula
    // http://www.movable-type.co.uk/scripts/latlong.html
    double a = std::pow(std::sin(diff_lat_rad / 2), 2) + std::cos(lat1_rad) * 
               std::cos(lat2_rad) * std::pow(std::sin(diff_lon_rad / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double d = EARTH_RADIUS * c; // in metres

    return d;
}