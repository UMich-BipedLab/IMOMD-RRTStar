#include <ros/ros.h>

#include <map_visualizer/map_visualizer.h>
#include <osm_parser/osm_parser.h>
#include <imomt_rrt_star/imomt_rrt_star.h>

int DEBUG_LEVEL = 10;

int main(int argc, char** argv)
{
    // ros::init(argc, argv, "map_visualizer");
    // ros::NodeHandle nh("~");

    // MapVisualizer map_visualizer(nh);

    // return 0;

    std::string osm_file = "/home/dongmyeong/catkin_ws/src/imomt_rrt_star/osm_data/sanf.osm";
    auto osmparser = OSMParser(osm_file);
    osmparser.parse();

    ImomtRRT imomt(0, 6355, {2163}, osmparser.getMap(), osmparser.getConnection());
    
    pthread_t tid[2];

    bipedlab::debugger::debugTitleTextOutput("[main]", "IMOMT running", 10);
    pthread_create(&tid[0], NULL, ImomtRRT::findShortestPath, &imomt);
    pthread_create(&tid[1], NULL, ImomtRRT::printPath, &imomt);
}