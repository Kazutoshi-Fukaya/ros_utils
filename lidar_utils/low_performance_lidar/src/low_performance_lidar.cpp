#include "low_performance_lidar/low_performance_lidar.h"

using namespace lidar_utils;

LowPerformanceLidar::LowPerformanceLidar() :
    private_nh_("~")
{
    private_nh_.param("USE_MSG_RANGE_LIMITS",USE_MSG_RANGE_LIMITS_,{false});
    private_nh_.param("RANGE_STEP",RANGE_STEP_,{2});

    double tmp_replacement_value = std::numeric_limits<double>::quiet_NaN();
    private_nh_.getParam("LOWER_REPLACEMENT_VALUE",tmp_replacement_value);
    LOWER_REPLACEMENT_VALUE_ = static_cast<float>(tmp_replacement_value);

    tmp_replacement_value = std::numeric_limits<double>::quiet_NaN();
    private_nh_.getParam("UPPER_REPLACEMENT_VALUE",tmp_replacement_value);
    UPPER_REPLACEMENT_VALUE_ = static_cast<float>(tmp_replacement_value);

    private_nh_.param("LOWER_THRESHOlLD",LOWER_THRESHOlLD_,{0.02});
    private_nh_.param("UPPER_THRESHOlLD",UPPER_THRESHOlLD_,{60.0});

    lsr_sub_ = nh_.subscribe("lsr_in",1,&LowPerformanceLidar::lsr_callback,this);
    lsr_pub_ = nh_.advertise<sensor_msgs::LaserScan>("lsr_out",1);
}

void LowPerformanceLidar::lsr_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    sensor_msgs::LaserScan lsr_msg;
    lsr_msg = *msg;

    if(USE_MSG_RANGE_LIMITS_){
        LOWER_THRESHOlLD_ = msg->range_min;
        UPPER_THRESHOlLD_ = msg->range_max;
    }
    lsr_msg.range_max = UPPER_THRESHOlLD_;
    lsr_msg.range_min = LOWER_THRESHOlLD_;

    // resample
    lsr_msg.ranges.resize((msg->ranges.size() - 1)/RANGE_STEP_ + 1);
    lsr_msg.intensities.resize(msg->ranges.size());
    int index = 0;
    for(size_t i = 0; i < msg->ranges.size(); i += RANGE_STEP_){
        lsr_msg.ranges.at(index) = msg->ranges.at(i);
        lsr_msg.intensities.at(index) = msg->intensities.at(i);
        index++;
    }

    for(auto &range : lsr_msg.ranges){
        if(range <= LOWER_THRESHOlLD_) range = LOWER_REPLACEMENT_VALUE_;
        if(range >= UPPER_THRESHOlLD_) range = UPPER_REPLACEMENT_VALUE_;
    }

    // angle
    lsr_msg.angle_increment = msg->angle_increment*RANGE_STEP_;
    lsr_msg.angle_min = msg->angle_min;
    lsr_msg.angle_max = lsr_msg.angle_min + lsr_msg.angle_increment*(lsr_msg.ranges.size() - 1);

    // std::cout << " input max_range: " << msg->range_max << std::endl;
    // std::cout << " input min_range: " << msg->range_min << std::endl;
    // std::cout << "output max_range: " << lsr_msg.range_max << std::endl;
    // std::cout << "output min_range: " << lsr_msg.range_min << std::endl;

    lsr_pub_.publish(lsr_msg);
}

void LowPerformanceLidar::process() { ros::spin(); }
