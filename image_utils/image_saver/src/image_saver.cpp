#include "image_saver/image_saver.h"

using namespace image_utils;

ImageSaver::ImageSaver() :
    private_nh_("~"),
    count_(0)
{
    private_nh_.param("FILE_PATH",FILE_PATH_,{std::string("")});
    private_nh_.param("HZ",HZ_,{1});
    private_nh_.param("SAMPLE_INTERVAL",SAMPLE_INTERVAL_,{10});

    img_sub_ = nh_.subscribe("img_in",1,&ImageSaver::image_callback,this);
}

void ImageSaver::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& ex){
        ROS_ERROR("Could not convert to color image");
        return;
    }

    if(count_%SAMPLE_INTERVAL_ == 0){
        std::string file_name = FILE_PATH_ + "/image" + std::to_string(count_/SAMPLE_INTERVAL_) + ".jpg";
        cv::imwrite(file_name,cv_ptr->image);
    }
    count_++;
}

void ImageSaver::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}
