#ifndef IMAGES_CONNECTOR_H_
#define IMAGES_CONNECTOR_H_

#include "images_connector/image_subscriber.h"

class ImagesConnector : public std::vector<ImageSubscriber*>
{
public:
    ImagesConnector();
    void process();

private:
    void init();
    void publish_img();

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // publisher
    ros::Publisher img_pub_;

    // param
    int HZ_;
};

#endif  // IMAGES_CONNECTOR_H_