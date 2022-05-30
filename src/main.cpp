#include <ros/ros.h>

#include "driver/driver.h"

int DEBUG_LEVEL = 10;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imomd_rrt_star");
    ros::NodeHandle nh("~");

    Driver driver(nh);
    
    return 0;
}
