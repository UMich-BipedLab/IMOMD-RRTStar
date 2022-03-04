#include <map_visualizer/map_visualizer.h>

    // std::string osm_file = "/home/dongmyeong/catkin_ws/src/imomt_rrt_star/osm_data/sanf.osm";
    // auto osmparser = OSMParser(osm_file);
    // osmparser.parse();

    // ImomtRRT imomt(0, 6355, {2163}, osmparser.getMap(), osmparser.getConnection());
    
    // pthread_t tid[2];

    // bipedlab::debugger::debugTitleTextOutput("[main]", "IMOMT running", 10);
    // pthread_create(&tid[0], NULL, ImomtRRT::findShortestPath, &imomt);
    // // pthread_create(&tid[1], NULL, ImomtRRT::printPath, &imomt);

MapVisualizer::MapVisualizer(ros::NodeHandle& nh)
{
    nh_ = nh;

    global_map_pub_ = nh_.advertise<visualization_msgs::Marker>("world_map", 1);

    ros::Rate r(10);
    while(ros::ok())
    {

        while (global_map_pub_.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

    
        publishToRviz_();
        r.sleep();
    }
}

void MapVisualizer::publishToRviz_()
{
    visualization_msgs::Marker line_marker;
    bipedlab::plotting::addMarkerWithTwoPoints(line_marker, 
                visualization_msgs::Marker::LINE_STRIP,
                "subgoal_to_goal",
                1, 1, 1, 1, // color
                0, // x1
                0, // y1
                0, // z1
                1, // x2
                1, // y2
                0, // z2
                0, 0, "",// count, time
                0.1, 0.01, 0.01
                );
    
    global_map_pub_.publish(line_marker);
}

MapVisualizer::~MapVisualizer() { }