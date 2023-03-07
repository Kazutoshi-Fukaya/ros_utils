#ifndef LASER_SCAN_TO_POINT_CLOUD_H_
#define LASER_SCAN_TO_POINT_CLOUD_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

class LaserScanToPointCloud
{
public:
    LaserScanToPointCloud();
    void process();

private:
    void lsr_callback(const sensor_msgs::LaserScanConstPtr& msg);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber lsr_sub_;

    // publisher
    ros::Publisher pc_pub_;

    // proojector
    laser_geometry::LaserProjection projector_;
};

#endif  // LASER_SCAN_TO_POINT_CLOUD_H_
