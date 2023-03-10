#include "pose_recorder/pose_recorder.h"

using namespace recorder;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"pose_recorder");
    PoseRecorder pose_recorder;
    pose_recorder.process();
    return 0;
}
