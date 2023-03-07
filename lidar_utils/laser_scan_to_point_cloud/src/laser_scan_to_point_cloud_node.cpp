#include "laser_scan_to_point_cloud/laser_scan_to_point_cloud.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"laser_scan_to_point_cloud");
    LaserScanToPointCloud laser_scan_to_point_cloud;
    laser_scan_to_point_cloud.process();
    return 0;
}
