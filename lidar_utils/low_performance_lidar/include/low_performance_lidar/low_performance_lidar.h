#ifndef LOW_PERFORMANCE_LIDAR_H_
#define LOW_PERFORMANCE_LIDAR_H_

// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// c++
#include <algorithm>

namespace lidar_utils
{
class LowPerformanceLidar
{
public:
    LowPerformanceLidar();
    void process();

private:
    void lsr_callback(const sensor_msgs::LaserScanConstPtr& msg);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber lsr_sub_;

    // publisher
    ros::Publisher lsr_pub_;

    // params
    bool USE_MSG_RANGE_LIMITS_;
    int RANGE_STEP_;
    float LOWER_REPLACEMENT_VALUE_;
    float UPPER_REPLACEMENT_VALUE_;
    double LOWER_THRESHOlLD_;
    double UPPER_THRESHOlLD_;
};
}

#endif  // LOW_PERFORMANCE_LIDAR_H_
