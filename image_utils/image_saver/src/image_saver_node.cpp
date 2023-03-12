#include "image_saver/image_saver.h"

using namespace image_utils;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"image_saver");
    ImageSaver image_saver;
    image_saver.process();
    return 0;
}
