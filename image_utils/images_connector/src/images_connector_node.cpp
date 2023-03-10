#include "images_connector/images_connector.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"images_connector");
    ImagesConnector images_connector;
    images_connector.process();
    return 0;
}