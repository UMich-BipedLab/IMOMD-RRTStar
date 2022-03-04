#ifndef MAP_VISUALIZER_H
#define MAP_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <utils/plotting.h>

class MapVisualizer
{
public:
    MapVisualizer(ros::NodeHandle& nh);
    virtual ~MapVisualizer();
private:
    ros::NodeHandle nh_;

    // publisher
    ros::Publisher global_map_pub_;

    // markers
    visualization_msgs::Marker marker_;

    void publishToRviz_();
};

#endif /* MAP_VISUALIZER_H */