#include "images_connector/images_connector.h"

ImagesConnector::ImagesConnector() :
    private_nh_("~")
{
    private_nh_.param("HZ",HZ_,{10});
    img_pub_ = nh_.advertise<sensor_msgs::Image>("image",1);
    init();
}

void ImagesConnector::init()
{
    this->clear();
    std::string robot_element_list_name;
    private_nh_.param("ROBOT_ELEMENT_LIST",robot_element_list_name,{std::string("robot_element_list")});
    XmlRpc::XmlRpcValue robot_element_list;
    if(!private_nh_.getParam(robot_element_list_name.c_str(),robot_element_list)){
        ROS_ERROR("Cloud not load %s", robot_element_list_name.c_str());
        return;
    }

    ROS_ASSERT(robot_element_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    this->resize(robot_element_list.size());
    for(int i = 0; i < (int)robot_element_list.size(); i++){
        if(!robot_element_list[i]["robot_name"].valid() ||
           !robot_element_list[i]["color"].valid()){
            ROS_ERROR("%s is valid", robot_element_list_name.c_str());
            return;
        }
        if(robot_element_list[i]["robot_name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
           robot_element_list[i]["color"].getType() == XmlRpc::XmlRpcValue::TypeString){
            std::string robot_name = static_cast<std::string>(robot_element_list[i]["robot_name"]);
            std::string color = static_cast<std::string>(robot_element_list[i]["color"]);
            this->at(i) = new ImageSubscriber(nh_,robot_name);
        }
    }
}

void ImagesConnector::publish_img()
{
    if(this->at(0)->get_img().empty()) return;

    int rows = this->at(0)->get_img().rows;
    int cols = this->at(0)->get_img().cols;

    cv::Mat space(cv::Mat::zeros(rows,50,CV_8UC3));
    cv::Mat base(rows,cols*this->size() + space.cols*((int)this->size() - 1),CV_8UC3);
    for(size_t i = 0; i < this->size(); i++){
        cv::Mat img = this->at(i)->get_img();
        if(img.empty()){
            img = cv::Mat::zeros(rows,cols,CV_8UC3);
        }
        cv::Mat roi_1(base,cv::Rect(i*(img.cols + space.cols),0,img.cols,img.rows));
        img.copyTo(roi_1);

        if((int)i != (int)this->size() - 1){
            cv::Mat roi_2(base,cv::Rect((i + 1)*img.cols + i*space.cols,0,space.cols,space.rows));
            space.copyTo(roi_2);
        }
    }

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",base).toImageMsg();
    img_pub_.publish(img_msg);
}

void ImagesConnector::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        publish_img();
        ros::spinOnce();
        rate.sleep();
    }
}
