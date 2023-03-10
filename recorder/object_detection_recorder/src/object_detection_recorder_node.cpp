#include "object_detection_recorder/object_detection_recorder.h"

using namespace recorder;

int main(int argc,char** argv)
{
	ros::init(argc,argv,"object_detection_recorder");
	ObjectDetectionRecorder object_detection_recorder;
	object_detection_recorder.process();
	return 0;
}