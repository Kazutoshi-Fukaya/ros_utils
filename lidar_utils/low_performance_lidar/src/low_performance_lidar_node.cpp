#include "low_performance_lidar/low_performance_lidar.h"

using namespace lidar_utils;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"low_performance_lidar");
    LowPerformanceLidar low_performance_lidar;
    low_performance_lidar.process();
    return 0;
}
