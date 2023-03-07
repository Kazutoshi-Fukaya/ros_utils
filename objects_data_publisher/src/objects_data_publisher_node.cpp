#include "objects_data_publisher/objects_data_publisher.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"objects_data_publisher");
    ObjectsDataPublisher objects_data_publisher;
    objects_data_publisher.process();
    return 0;
}
