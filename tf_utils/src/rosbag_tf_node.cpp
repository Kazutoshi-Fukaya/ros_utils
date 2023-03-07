#include "rosbag_tf/rosbag_tf.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"rosbag_tf");
	RosbagTF rosbag_tf;
	rosbag_tf.process();
	return 0;
}