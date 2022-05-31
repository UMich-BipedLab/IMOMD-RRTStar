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
 * File:        compute_haversine.h
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     02/24/2022
 * 
 * Description: Compute Haversine Distance
 *              http://www.movable-type.co.uk/scripts/latlong.html
*******************************************************************************/
#ifndef COMPUTE_HAVERSINE_H
#define COMPUTE_HAVERSINE_H

#include <math.h>

template <class T>
double computeHaversineDistance(const T& location1, const T& location2)
{
    static constexpr double DEG2RAD = M_PI / 180;
    static const int EARTH_RADIUS = 6371e3; // in metres;

    double lat1_rad = location1.latitude * DEG2RAD;
    double lat2_rad = location2.latitude * DEG2RAD; 
    double diff_lat_rad = (location2.latitude - location1.latitude) * DEG2RAD;
    double diff_long_rad = (location2.longitude - location1.longitude) * DEG2RAD;

    double a = std::pow(std::sin(diff_lat_rad / 2), 2) + std::cos(lat1_rad) * 
            std::cos(lat2_rad) * std::pow(std::sin(diff_long_rad / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double d = EARTH_RADIUS * c; // in metres

    return d;
}
#endif /* COMPUTE_HAVERSINE_H */
