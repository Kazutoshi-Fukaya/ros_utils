#ifndef IMAGE_SAVER_H_
#define IMAGE_SAVER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace image_utils
{
class ImageSaver
{
public:
    ImageSaver();
    void process();

private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber img_sub_;

    // buffer
    int count_;

    // params
    std::string FILE_PATH_;
    int HZ_;
    int SAMPLE_INTERVAL_;
};
} // namespace image_utils

#endif  // IMAGE_SAVER_H_
