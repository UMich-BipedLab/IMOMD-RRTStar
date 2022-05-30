#include "driver/driver.h"
#include "utils/nameof.h"

Driver::Driver(ros::NodeHandle& nh): 
    is_source_updated_(false), is_target_updated_(false),
    is_destinations_updated_(false), is_computation_finished_(false)
{   
    // ROS
    nh_ = nh;

    // Get Parameter
    if (!Driver::getParameters_())
    {
        bipedlab::debugger::debugTitleTextOutput("[Driver]", "NOTICE!!!!", 10, BR, BOLD);
        bipedlab::debugger::debugColorOutput("[Driver] Not enough parameters: ",
                "Using default values", 10, BR, BOLD);
        bipedlab::debugger::debugTitleTextOutput("[Driver]", "", 10, BR, BOLD);
        bipedlab::utils::pressEnterToContinue();
    }
    else
    {
        bipedlab::debugger::debugColorOutput("[Driver] Received all parameters", "", 10, BC);
    }

    // Publisher
    global_map_node_pub_ = nh_.advertise<visualization_msgs::Marker>("global_map_node", 1, true);
    global_map_edge_pub_ = nh_.advertise<visualization_msgs::Marker>("global_map_edge", 1, true);
    destinations_pub_= nh_.advertise<visualization_msgs::MarkerArray>("destinations", 1, true);
    shortest_path_pub_ = nh_.advertise<visualization_msgs::Marker>("shortest_path", 1, true);

    // Load OSM and Parsing
    std::string osm_file = map_properties_.path + map_properties_.name;
    auto osm_parser = OSMParser(osm_file, map_properties_);
    osm_parser.parse();

    raw_map_ptr_ = osm_parser.getMap();
    connection_ptr_ = osm_parser.getConnection();

    // Converting Location(Latitude, Longitude) to X,Y coordinate
    coordinates_converter_ = std::make_shared<CoordinatesConverter>(raw_map_ptr_);
    converted_map_ptr_ = coordinates_converter_->getConvertedMap();
    map_size_ = coordinates_converter_->getMapSize();
    
    // Spin to callback functions
    pthread_t tid[2];
    pthread_create(&tid[0], NULL, &Driver::spin_, this);

    // Publish Global Map
    waitForSubscriber_();
    publishGlobalMapToRviz_();
    
    if (driver_mode_ == 0)
    {
        // Subscriber
        clicked_point_sub_ = nh_.subscribe("/clicked_point", 1, 
                            &Driver::getClickedPointCallBack_, this);
        
        waitForData_();
    }
    else if (driver_mode_ == 1)
    {

        bipedlab::debugger::debugColorTextOutput(
                "[Driver] Using desinations from config file", 10, G);
        bipedlab::debugger::debugColorOutput("[Driver] Source: ", 
                                             source_id_, 10, G);
        bipedlab::debugger::debugColorContainerOutput("[Driver] Objectives: ", 
                                                      objectives_id_, 10, G);
        bipedlab::debugger::debugColorOutput("[Driver] Target: ", 
                                             target_id_, 10, G);
        is_target_updated_ = true;
        is_source_updated_ = true;
    }
    else
    {
        std::string error_msg = "ERROR: NO SUCH DRIVER MODE: " + 
            std::to_string(driver_mode_);
        bipedlab::debugger::debugExitColor(error_msg, __LINE__, __FILE__);
    }

    publishDestinationsToRviz_();

    // Run IMOMD
    ros::Rate r(publishing_rate_);
    switch (system_)
    {
        case 0:
        {
            ImomdRRT imomd(source_id_, target_id_, objectives_id_, 
                   raw_map_ptr_, connection_ptr_, setting_);

            pthread_create(&tid[1], NULL, &ImomdRRT::findShortestPath, &imomd);
            
            while(ros::ok() && !imomd.checkComputationFinished())
            {   
                publishShortestPathToRviz_(imomd.getShortestPath());
                r.sleep();
            }
            is_computation_finished_ = true;

            pthread_join(tid[1], NULL);
            pthread_join(tid[0], NULL);

            bipedlab::utils::pressEnterToContinue();
            exit(0);
        }
        case 1:
        {
            BiAstar bi_a_star(source_id_, target_id_, objectives_id_, 
                   raw_map_ptr_, connection_ptr_, setting_);

            pthread_create(&tid[1], NULL, &BiAstar::findShortestPath, &bi_a_star);
            
            while(ros::ok() && !bi_a_star.checkComputationFinished())
            {   
                publishShortestPathToRviz_(bi_a_star.getShortestPath());
                r.sleep();
            }
            is_computation_finished_ = true;

            pthread_join(tid[1], NULL);
            pthread_join(tid[0], NULL);

            bipedlab::utils::pressEnterToContinue();
            exit(0);
        }
        case 2:
        {
            ANAStar ana_star(source_id_, target_id_, objectives_id_, 
                   raw_map_ptr_, connection_ptr_, setting_);

            pthread_create(&tid[1], NULL, &ANAStar::findShortestPath, &ana_star);
            
            while(ros::ok() && !ana_star.checkComputationFinished())
            {   
                publishShortestPathToRviz_(ana_star.getShortestPath());
                r.sleep();
            }
            is_computation_finished_ = true;

            pthread_join(tid[1], NULL);
            pthread_join(tid[0], NULL);

            bipedlab::utils::pressEnterToContinue();
            exit(0);
        }
    }
}
    

void Driver::waitForSubscriber_()
{
    while (ros::ok() && global_map_edge_pub_.getNumSubscribers() < 1)
    {
        ROS_WARN_ONCE("Please create a subscriber to the driver");
        sleep(1);
    }
}

void* Driver::spin_(void* arg)
{
    Driver* this_thread = static_cast<Driver*>(arg);
    while (ros::ok() && !this_thread->is_computation_finished_)
    {
        ros::spinOnce();
    }
    pthread_exit(NULL);
}

void Driver::waitForData_()
{
    pthread_mutex_lock(&lock_);
    while (ros::ok() && !is_destinations_updated_)
    {
        ROS_WARN_ONCE("Please click destinations");
        sleep(1);
    }
    pthread_mutex_unlock(&lock_);
}

void Driver::getClickedPointCallBack_(const geometry_msgs::PointStamped::ConstPtr& msg)
{   
    size_t clicked_node_id = 
        coordinates_converter_->getNearestPoint(msg->point.x, msg->point.y);

    if (!is_source_updated_)
    {
        source_id_ = clicked_node_id;
        is_source_updated_ = true;
        bipedlab::debugger::debugColorTextOutput("[Driver] Source Point received", 10, BC);
        std::cout << "node_id : " << clicked_node_id << std::endl;
    }
    else if (objectives_id_.size() < num_objectives_)
    {
        objectives_id_.push_back(clicked_node_id);
        bipedlab::debugger::debugColorOutput("[Driver] Objective Point received [",
            std::to_string(objectives_id_.size()) + "/" + 
            std::to_string(num_objectives_) + "]", 10, BC);
        std::cout << "node_id : " << clicked_node_id << std::endl;
    }
    else if (!is_target_updated_)
    {
        target_id_ = clicked_node_id;
        is_target_updated_ = true;
        bipedlab::debugger::debugColorTextOutput("[Driver] Target Point received", 10, BC);
        std::cout << "node_id : " << clicked_node_id << std::endl;

        pthread_mutex_lock(&lock_);
        is_destinations_updated_ = true;
        pthread_mutex_unlock(&lock_);
    }
    publishDestinationsToRviz_();
}

void Driver::publishGlobalMapToRviz_()
{
    // Publish Nodes
    visualization_msgs::Marker points;

    points.id = 0;
    points.ns = "global_map_node";
    points.type = visualization_msgs::Marker::POINTS;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.lifetime = ros::Duration();
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.scale.x = map_size_ / 500;
    points.scale.y = map_size_ / 500;

    points.color.r = 1.0f;
    points.color.g = 1.0f;
    points.color.b = 1.0f;
    points.color.a = 1.0;

    for (const auto& node : *converted_map_ptr_)
    {
        geometry_msgs::Point p;
        p.x = node.x;
        p.y = node.y;
        p.z = node.z;

        points.points.push_back(p);
    }
    global_map_node_pub_.publish(points);


    // Publish Edges
    visualization_msgs::Marker line_list;

    line_list.id = 1;
    line_list.ns = "global_map_edge";
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.lifetime = ros::Duration();
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.scale.x = map_size_ / 600;

    line_list.color.g = 1.0f;
    line_list.color.a = 0.1;

    for (size_t i = 0; i < (*connection_ptr_).size(); ++i)
    {   
        geometry_msgs::Point p1;
        p1.x = (*converted_map_ptr_)[i].x;
        p1.y = (*converted_map_ptr_)[i].y;
        p1.z = (*converted_map_ptr_)[i].z;

        for (const auto& node : (*connection_ptr_)[i])
        {
            geometry_msgs::Point p2;
            p2.x = (*converted_map_ptr_)[node.first].x;
            p2.y = (*converted_map_ptr_)[node.first].y;
            p2.z = (*converted_map_ptr_)[node.first].z;

            line_list.points.push_back(p1);
            line_list.points.push_back(p2);
        }
    }
    global_map_edge_pub_.publish(line_list);
}

void Driver::publishDestinationsToRviz_()
{
    int sphere_scale = map_size_ / 50;
    destinations_marker_array_.markers.clear();

    // Source
    if (is_source_updated_)
    {
        bipedlab::plotting::addMarker(marker_,
            visualization_msgs::Marker::SPHERE,
            "source_node",
            0, 1, 0, 1, // color
            (*converted_map_ptr_)[source_id_].x,
            (*converted_map_ptr_)[source_id_].y,
            (*converted_map_ptr_)[source_id_].z + map_size_ / 200,
            0, 0, 0, 1, // orientation
            10, 0, "", // id, duration_time, text 
            sphere_scale, sphere_scale, sphere_scale, // scale (x, y, z)
            "map");
        destinations_marker_array_.markers.push_back(marker_);
    }

    // Target
    if (is_target_updated_)
    {
        bipedlab::plotting::addMarker(marker_,
            visualization_msgs::Marker::SPHERE,
            "target_node",
            0, 1, 1, 1, // color
            (*converted_map_ptr_)[target_id_].x,
            (*converted_map_ptr_)[target_id_].y,
            (*converted_map_ptr_)[target_id_].z + map_size_ / 200,
            0, 0, 0, 1, // orientation
            11, 0, "", // id, duration_time, text 
            sphere_scale, sphere_scale, sphere_scale, // scale (x, y, z)
            "map");
        destinations_marker_array_.markers.push_back(marker_);
    }

    // Objectives
    int count = 0;
    for (size_t objective_id : objectives_id_)
    {
        bipedlab::plotting::addMarker(marker_,
        visualization_msgs::Marker::SPHERE,
        "objective_node",
        1, 1, 0, 1, // color
        (*converted_map_ptr_)[objective_id].x,
        (*converted_map_ptr_)[objective_id].y,
        (*converted_map_ptr_)[objective_id].z + map_size_ / 200,
        0, 0, 0, 1, // orientation
        12 + count++, 0, "", // id, duration_time, text 
        sphere_scale, sphere_scale, sphere_scale, // scale (x, y, z)
        "map");
        destinations_marker_array_.markers.push_back(marker_);
    }

    // Publish
    destinations_pub_.publish(destinations_marker_array_);
}

void Driver::publishShortestPathToRviz_(std::shared_ptr<std::vector<size_t>> path)
{
    visualization_msgs::Marker line_strip;

    line_strip.id = 3;
    line_strip.ns = "shortest_path";
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.lifetime = ros::Duration();
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.scale.x = map_size_ / 100;
    line_strip.scale.y = map_size_ / 100;

    line_strip.color.r = 1.0f;
    // line_strip.color.b = 1.0f;
    line_strip.color.a = 1.0;

    for (size_t node_id : *path)
    {
        geometry_msgs::Point p;
        p.x = (*converted_map_ptr_)[node_id].x;
        p.y = (*converted_map_ptr_)[node_id].y;
        p.z = 10;

        line_strip.points.push_back(p);
    }
    shortest_path_pub_.publish(line_strip);
}

bool Driver::getParameters_()
{
    std::string title_name("[Driver]/[getParameters] ");
    bool received_all = true;

    // Assign Algorithm (IMOMD vs. BaseLine)
    bipedlab::ros_utils::checkROSParam(nh_, "general/system", system_,
            getNameOf(system_), title_name, received_all);

    // Pseudo Mode
    bipedlab::ros_utils::checkROSParam(nh_, "general/pseudo", setting_.pseudo_mode,
            getNameOf(setting_) + "." + getNameOf(setting_.pseudo_mode),
            title_name, received_all);

    // log command
    bipedlab::ros_utils::checkROSParam(nh_, "general/log_data", setting_.log_data,
            getNameOf(setting_) + "." + getNameOf(setting_.log_data),
            title_name, received_all);

    // Running constraints
    bipedlab::ros_utils::checkROSParam(nh_, "general/max_iter", setting_.max_iter,
             getNameOf(setting_) + "." + getNameOf(setting_.max_iter),
             title_name, received_all);

    bipedlab::ros_utils::checkROSParam(nh_, "general/max_time", setting_.max_time,
             getNameOf(setting_) + "." + getNameOf(setting_.max_time),
             title_name, received_all);

     // driver parameters
    bipedlab::ros_utils::checkROSParam(nh_, "driver_mode", driver_mode_,
            getNameOf(driver_mode_), title_name, received_all);

    bipedlab::ros_utils::checkROSParam(nh_, "num_objectives", num_objectives_,
            getNameOf(num_objectives_), title_name, received_all);
    
    bipedlab::ros_utils::checkROSParam(nh_, "publishing_rate", publishing_rate_,
            getNameOf(publishing_rate_), title_name, received_all);
    
    if (driver_mode_ == 1)
    {
        // destinations
        int source, target;
        bipedlab::ros_utils::checkROSParam(nh_, "destinations/source_id", source,
                getNameOf(source_id_), title_name, received_all);
        source_id_ = source;
        bipedlab::ros_utils::checkROSParam(nh_, "destinations/target_id", target,
                getNameOf(target_id_), title_name, received_all);
        target_id_ = target;
        std::vector<int> objectives;
        bipedlab::ros_utils::checkROSParam(nh_, "destinations/objective_ids", 
                objectives, getNameOf(objectives_id_), title_name, received_all);
        objectives_id_.assign(objectives.begin(), objectives.end());
    }

    // map
    bipedlab::ros_utils::checkROSParam(nh_, "map/type", map_properties_.type,
            getNameOf(map_properties_) + "." + getNameOf(map_properties_.type),
            title_name, received_all);

    bipedlab::ros_utils::checkROSParam(nh_, "map/path", map_properties_.path,
            getNameOf(map_properties_) + "." + getNameOf(map_properties_.path),
            title_name, received_all);

    bipedlab::ros_utils::checkROSParam(nh_, "map/name", map_properties_.name,
            getNameOf(map_properties_) + "." + getNameOf(map_properties_.name),
            title_name, received_all);

    // filtering OSM Way Type
    if (map_properties_.type == 1)
    {
        int filter = 0;
        std::vector<std::string> osm_filter = {"osm_all", "osm_cars",
                                               "osm_walkers", "osm_cyclists"};
        std::vector<std::string> osm_key;
        std::vector<std::string> osm_value;

        nh_.getParam("osm_ways_type", filter);

        nh_.getParam(osm_filter[filter] + "/key", osm_key);
        nh_.getParam(osm_filter[filter] + "/value", osm_value);

        std::copy(osm_key.begin(), osm_key.end(), 
                std::inserter(map_properties_.key, map_properties_.key.end()));
        std::copy(osm_value.begin(), osm_value.end(), 
                std::inserter(map_properties_.value, map_properties_.value.end()));
    }

    // IMOMD parameters
    bipedlab::ros_utils::checkROSParam(nh_, "rrt_params/goal_bias", setting_.goal_bias,
             getNameOf(setting_) + "." + getNameOf(setting_.goal_bias),
             title_name, received_all);
    
    bipedlab::ros_utils::checkROSParam(nh_, "rrt_params/random_seed", setting_.random_seed,
             getNameOf(setting_) + "." + getNameOf(setting_.random_seed),
             title_name, received_all);

    // write default values here
    if (!received_all)
    {
        driver_mode_ = 1;
        setting_.goal_bias = 0.2;
        setting_.max_iter = 100000;
    }

    return received_all;
}

Driver::~Driver() { }