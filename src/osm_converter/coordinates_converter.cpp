/*******************************************************************************
 * File:        coordinates_converter.cpp
 * 
 * Author:      Dongmyeong Lee (dongmyeong[at]umich.edu)
 * Created:     02/24/2022
 * 
 * Description: Convert (latitude, longitude) to (x, y) with haversine distance
*******************************************************************************/
#include "osm_converter/coordinates_converter.h"

CoordinatesConverter::CoordinatesConverter(
    std::shared_ptr<std::vector<location_t>> map_ptr)
{
    max_x_ = -INFINITY;
    max_y_ = -INFINITY;

    double min_latitude = INFINITY;
    double min_longitude = INFINITY;
    double max_latitude = -INFINITY;
    double max_longitude = -INFINITY;

    // Initialize the origin of the map
    for (const auto& location : (*map_ptr))
    {
        // Latitude
        if (location.latitude < min_latitude)
        {
            min_latitude = location.latitude;
        }
        else if (location.latitude > max_latitude)
        {
            max_latitude = location.latitude;
        }
        // Longitude
        if (location.longitude < min_longitude)
        {
            min_longitude = location.longitude;
        }
        else if (location.longitude > max_longitude)
        {
            max_longitude = location.longitude;
        }
    }
    origin_.id = -1;
    origin_.latitude = (min_latitude + max_latitude) / 2.0;
    origin_.longitude = (min_longitude + max_longitude) / 2.0;

    // Converting Latitude, Longitude coordinates to x, y coordinates
    nodes_.resize((*map_ptr).size());

    size_t idx = 0;
    for (const auto& location : (*map_ptr))
    {
        convertNode_(location, nodes_[idx++]);
    }
}

std::shared_ptr<std::vector<node_t>> CoordinatesConverter::getConvertedMap()
{
    return std::make_shared<std::vector<node_t>>(nodes_);
}

size_t CoordinatesConverter::getNearestPoint(double x, double y)
{
    size_t id = -1;
    double min_distance = INFINITY;
    double distance;

    for (const auto& node : nodes_)
    {
        distance = std::sqrt(std::pow(node.x - x, 2.0) + std::pow(node.y - y, 2.0));

        if (distance < min_distance)
        {
            min_distance = distance;
            id = node.id;
        }
    }
    return id;
}

int CoordinatesConverter::getMapSize()
{
    return std::max(max_x_, max_y_);
}

void CoordinatesConverter::convertNode_(const location_t& location, node_t& node)
{
    node.id = location.id;
    std::tie(node.x, node.y) = getCoordinates_(location);
    node.z = 0;

    if (node.x > max_x_)
    {
        max_x_ = node.x;
    }

    if (node.y > max_y_)
    {
        max_y_ = node.y;
    }
}

std::pair<double, double> CoordinatesConverter::getCoordinates_(location_t location)
{
    double distance = getDistance_(origin_, location);
    double bearing = getBearing_(origin_, location);

    return std::make_pair(std::sin(bearing + OFFSET_) * distance,
                          std::cos(bearing + OFFSET_) * distance);
}

double CoordinatesConverter::getDistance_(
    const location_t& location1, const location_t& location2)
{
    double lat1_rad = location1.latitude * DEG2RAD_;
    double lat2_rad = location2.latitude * DEG2RAD_; 
    double diff_lat_rad = (location2.latitude - location1.latitude) * DEG2RAD_;
    double diff_lon_rad = (location2.longitude - location1.longitude) * DEG2RAD_;

    // 'Harversine' Formula
    // http://www.movable-type.co.uk/scripts/latlong.html
    double a = std::pow(std::sin(diff_lat_rad / 2), 2) + std::cos(lat1_rad) * 
               std::cos(lat2_rad) * std::pow(std::sin(diff_lon_rad / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    return EARTH_RADIUS_ * c; // in metres
}

double CoordinatesConverter::getBearing_(
    const location_t& location1, const location_t& location2)
{
    double lat1_rad = location1.latitude * DEG2RAD_;
    double lat2_rad = location2.latitude * DEG2RAD_;
    double diff_lon_rad = (location2.longitude - location1.longitude) * DEG2RAD_;

    // http://www.movable-type.co.uk/scripts/latlong.html
    double y = std::sin(diff_lon_rad) * std::cos(lat2_rad);
    double x = std::cos(lat1_rad) * std::sin(lat2_rad) -
               std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(diff_lon_rad);

    return std::atan2(y, x) + OFFSET_; // in radian
}