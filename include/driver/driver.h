#ifndef DRIVER_H
#define DRIVER_H

#include <pthread.h>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

#include "osm_converter/osm_parser.h"
#include "osm_converter/coordinates_converter.h"

#include "imomd_rrt_star/imomd_rrt_star.h"
#include "baseline/bi_a_star.h"
#include "baseline/ana_star.h"

#include "utils/debugger.h"
#include "utils/ros_plotting.h"
#include "utils/ros_utils.h"

#include "data/location_t.h"

#include "setting/imomd_setting_t.h"
#include "setting/map_setting_t.h"

class Driver
{
public:
    Driver(ros::NodeHandle& nh);
    virtual ~Driver();

private:
    // Pthread
    pthread_mutex_t lock_;

    // ROS
    ros::NodeHandle nh_;

    // Publisher
    ros::Publisher global_map_node_pub_;
    ros::Publisher global_map_edge_pub_;
    ros::Publisher destinations_pub_;
    ros::Publisher shortest_path_pub_;
    int publishing_rate_;

    // Subscriber
    ros::Subscriber clicked_point_sub_;

    // markers
    visualization_msgs::Marker marker_;
    visualization_msgs::MarkerArray destinations_marker_array_;

    // to check whether data received
    bool is_source_updated_;
    bool is_target_updated_;
    bool is_destinations_updated_;
    bool is_computation_finished_;

    // Data
    std::shared_ptr<std::vector<location_t>> raw_map_ptr_;
    std::shared_ptr<std::vector<std::unordered_map<size_t, double>>> connection_ptr_;
    std::shared_ptr<std::vector<node_t>> converted_map_ptr_;

    // destination_t destinations_;
    size_t source_id_;
    size_t target_id_;
    std::vector<size_t> objectives_id_;
    int num_objectives_;

    // Coordinate Converter
    std::shared_ptr<CoordinatesConverter> coordinates_converter_;
    int map_size_;

    // general parameters & IMOMD parameters
    int driver_mode_;
    int system_;
    imomd_setting_t setting_;
    map_properties_t map_properties_;

    // Initialization Functions
    bool getParameters_();
    void waitForData_();
    void waitForSubscriber_();
    static void* spin_(void* arg);

    // Callback Functions
    void getClickedPointCallBack_(const geometry_msgs::PointStamped::ConstPtr& msg);

    // Publish Visualization msg
    void publishGlobalMapToRviz_();
    void publishDestinationsToRviz_();
    void publishShortestPathToRviz_(std::shared_ptr<std::vector<size_t>> path);
};

#endif /* DRIVER_H */
