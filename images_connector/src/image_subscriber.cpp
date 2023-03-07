#include "images_connector/image_subscriber.h"

ImageSubscriber::ImageSubscriber(ros::NodeHandle _nh,std::string robot_name) :
    nh_(_nh)
{
    std::string topic_name = robot_name + "/detected_image";
    img_sub_ = nh_.subscribe(topic_name,1,&ImageSubscriber::img_callback,this);
}

void ImageSubscriber::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& ex){
        ROS_ERROR("Could not convert to color image");
        return;
    }
    img_ = cv_ptr->image;
}

cv::Mat ImageSubscriber::get_img() { return img_; }