#include "laser_scan_to_point_cloud/laser_scan_to_point_cloud.h"

LaserScanToPointCloud::LaserScanToPointCloud() :
    private_nh_("~")
{
    lsr_sub_ = nh_.subscribe("scan",1,&LaserScanToPointCloud::lsr_callback,this);
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pc_out",1);
}

void LaserScanToPointCloud::lsr_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud_msg;
    projector_.projectLaser(*msg,cloud_msg);
    cloud_msg.header = msg->header;
    pc_pub_.publish(cloud_msg);
}

void LaserScanToPointCloud::process() { ros::spin(); }
