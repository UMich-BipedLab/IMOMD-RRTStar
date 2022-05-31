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
 * File:        fake_map.h
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     03/08/2022
 * 
 * Description: Generate Fake Map for IMOMD
*******************************************************************************/
#ifndef FAKE_MAP_H
#define FAKE_MAP_H

#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <memory>
#include <string>

#include "data/location_t.h"
#include "utils/utils.h"

template <typename T>
T computeHaversineDistance(T lat1, T long1, T lat2, T long2);

class FakeMap
{
public:
    FakeMap(int map_type): map_type_(map_type)
    {
        map_.clear();
        graph_.clear();
        
        switch (map_type_)
        {
            case -1:
                load_map_1_();
                break;
            case -2:
                load_map_2_();
                break;
            default:
                throw std::invalid_argument("Invalid Fake Map variable!");
        }
        bipedlab::debugger::debugColorOutput("[Fake Map] Created Map (size : ",
            std::to_string(map_.size()) + ")", 10, BY);
    }

    std::shared_ptr<std::vector<location_t>> getMap()
    {
        return std::make_shared<std::vector<location_t>>(map_);
    }
        
    std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> getGraph()
    {
        return std::make_shared<std::vector<std::unordered_map<size_t, double>>>(graph_);
    }

private:
    int map_type_;
    std::vector<location_t> map_;
    std::vector<std::unordered_map<size_t, double>> graph_;

    /* Graph 1 */

    // 
    //  (0,0)  (2, 0) (4, 0)
    //    0  --- 1 --- 2
    //      \   /
    //       \ /
    //        3 (1, -1)

    void load_map_1_()
    {
        map_.resize(4);
        graph_.resize(4);

        map_[0].id = 0;
        map_[0].latitude = 0;
        map_[0].longitude = 0;

        map_[1].id = 1;
        map_[1].latitude = 2;
        map_[1].longitude = 0;

        map_[2].id = 2;
        map_[2].latitude = 4;
        map_[2].longitude = 0;

        map_[3].id = 3;
        map_[3].latitude = 1;
        map_[3].longitude = -1;

        graph_[0] = 
        {
            {1, computeHaversineDistance(map_[0].latitude, map_[0].longitude,
                                         map_[1].latitude, map_[1].longitude)
            },
            {3, computeHaversineDistance(map_[0].latitude, map_[0].longitude,
                                         map_[3].latitude, map_[3].longitude)
            }
        };

        graph_[1] =
        {
            {0, computeHaversineDistance(map_[1].latitude, map_[1].longitude,
                                         map_[0].latitude, map_[0].longitude)
            },
            {2, computeHaversineDistance(map_[1].latitude, map_[1].longitude,
                                         map_[2].latitude, map_[2].longitude)
            },
            {3, computeHaversineDistance(map_[1].latitude, map_[1].longitude,
                                         map_[3].latitude, map_[3].longitude)
            }
        };

        graph_[2] =
        {
            {1, computeHaversineDistance(map_[2].latitude, map_[2].longitude,
                                         map_[1].latitude, map_[1].longitude)
            }
        };

        graph_[3] =
        {
            {0, computeHaversineDistance(map_[3].latitude, map_[3].longitude,
                                         map_[0].latitude, map_[0].longitude)
            },
            {1, computeHaversineDistance(map_[3].latitude, map_[3].longitude,
                                         map_[1].latitude, map_[1].longitude)
            }
        };

    }
    
    /* Graph 1 */

    // 
    //     
    //    (-0.1, 2.1) 0 --- 1 (0.1, 2.1)
    //                 \   /
    //                   2 (0, 2)
    //                  / \
    //                 /   \
    //                /     \
    //               /       \
    //      (-1, 0) 3         4 (1, 0)
    //               \       /
    //                \     /
    //                 \   /
    //                  \ /
    //                   5 (0, -1)
    //                   |
    //                   6 (0, -1.1)

    void load_map_2_()
    {
        map_.resize(7);
        graph_.resize(7);

        map_[0].id = 0;
        map_[0].latitude = -0.1;
        map_[0].longitude = 2.1;

        map_[1].id = 1;
        map_[1].latitude = 0.1;
        map_[1].longitude = 2.1;

        map_[2].id = 2;
        map_[2].latitude = 0;
        map_[2].longitude = 2;

        map_[3].id = 3;
        map_[3].latitude = -1;
        map_[3].longitude = 0;

        map_[4].id = 4;
        map_[4].latitude = 1;
        map_[4].longitude = 0;

        map_[5].id = 5;
        map_[5].latitude = 0;
        map_[5].longitude = -1;

        map_[6].id = 6;
        map_[6].latitude = 0;
        map_[6].longitude = -1.1;
        
        graph_[0] = 
        {
            {1, computeHaversineDistance(map_[0].latitude, map_[0].longitude,
                                         map_[1].latitude, map_[1].longitude)
            },
            {2, computeHaversineDistance(map_[0].latitude, map_[0].longitude,
                                         map_[2].latitude, map_[2].longitude)
            }
        };

        graph_[1] =
        {
            {0, computeHaversineDistance(map_[1].latitude, map_[1].longitude,
                                         map_[0].latitude, map_[0].longitude)
            },
            {2, computeHaversineDistance(map_[1].latitude, map_[1].longitude,
                                         map_[2].latitude, map_[2].longitude)
            }
        };

        graph_[2] =
        {
            {0, computeHaversineDistance(map_[2].latitude, map_[2].longitude,
                                         map_[0].latitude, map_[0].longitude)
            },
            {1, computeHaversineDistance(map_[2].latitude, map_[2].longitude,
                                         map_[1].latitude, map_[1].longitude)
            },
            {3, computeHaversineDistance(map_[2].latitude, map_[2].longitude,
                                         map_[3].latitude, map_[3].longitude)
            },
            {4, computeHaversineDistance(map_[2].latitude, map_[2].longitude,
                                         map_[4].latitude, map_[4].longitude)
            }
        };

        graph_[3] =
        {
            {2, computeHaversineDistance(map_[3].latitude, map_[3].longitude,
                                         map_[2].latitude, map_[2].longitude)
            },
            {5, computeHaversineDistance(map_[3].latitude, map_[3].longitude,
                                         map_[5].latitude, map_[5].longitude)
            }
        };

        graph_[4] =
        {
            {2, computeHaversineDistance(map_[4].latitude, map_[4].longitude,
                                         map_[2].latitude, map_[2].longitude)
            },
            {5, computeHaversineDistance(map_[4].latitude, map_[4].longitude,
                                         map_[5].latitude, map_[5].longitude)
            }
        };

        graph_[5] =
        {
            {3, computeHaversineDistance(map_[5].latitude, map_[5].longitude,
                                         map_[3].latitude, map_[3].longitude)
            },
            {4, computeHaversineDistance(map_[5].latitude, map_[5].longitude,
                                         map_[4].latitude, map_[4].longitude)
            },
            {6, computeHaversineDistance(map_[5].latitude, map_[5].longitude,
                                         map_[6].latitude, map_[6].longitude)
            }
        };

        graph_[6] =
        {
            {5, computeHaversineDistance(map_[6].latitude, map_[6].longitude,
                                         map_[5].latitude, map_[5].longitude)
            }
        };
    }
};

template <typename T>
T computeHaversineDistance(T lat1, T long1, T lat2, T long2)
{
    static constexpr double DEG2RAD = M_PI / 180;
    static const int EARTH_RADIUS = 6371e3; // in metres;

    double lat1_rad = lat1 * DEG2RAD;
    double lat2_rad = lat2 * DEG2RAD; 
    double diff_lat_rad = (lat2 - lat1) * DEG2RAD;
    double diff_long_rad = (long2 - long1) * DEG2RAD;

    // 'Harversine' Formula
    // http://www.movable-type.co.uk/scripts/latlong.html
    double a = std::pow(std::sin(diff_lat_rad / 2), 2) + std::cos(lat1_rad) * 
            std::cos(lat2_rad) * std::pow(std::sin(diff_long_rad / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double d = EARTH_RADIUS * c; // in metres
    return d;
}

#endif /* FAKE_MAP_H */
