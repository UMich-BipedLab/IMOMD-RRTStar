/*******************************************************************************
 * File:        coordinates_converter.h
 * 
 * Author:      Dongmyeong Lee (dongmyeong[at]umich.edu)
 * Created:     02/24/2022
 * 
 * Description: Convert (latitude, longitude) to (x, y) with haversine distance
*******************************************************************************/
#ifndef COORDINATES_CONVERTER_H
#define COORDINATES_CONVERTER_H

#include <vector>
#include <math.h>
#include <memory>
#include <unordered_set>
#include <iostream>
#include <algorithm>

#include "data/location_t.h"
#include "data/node_t.h"

class CoordinatesConverter
{
public:
    CoordinatesConverter();
    CoordinatesConverter(std::shared_ptr<std::vector<location_t>> map_ptr);

    std::shared_ptr<std::vector<node_t>> getConvertedMap();
    size_t getNearestPoint(double x, double y);

    int getMapSize();

private:
    std::vector<node_t> nodes_;
    std::vector<std::vector<node_t>> ways_;
    location_t origin_;

    double max_x_;
    double max_y_;

    constexpr static double OFFSET_ = M_PI / 2.0;
    constexpr static double DEG2RAD_ = M_PI / 180;
    const static int EARTH_RADIUS_ = 6371e3; // in metres;

    void convertNode_(const location_t& location, node_t& node);

    std::pair<double, double> getCoordinates_(location_t location);
    
    double getDistance_(const location_t& location1, const location_t& location2);
    
    double getBearing_(const location_t& location1, const location_t& location2);
};

#endif /* COORDINATES_CONVERTER_H */
