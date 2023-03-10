#ifndef OBJECT_DETECTION_RECORDER_H_
#define OBJECT_DETECTION_RECORDER_H_

// ros
#include <ros/ros.h>

// Custom msg
#include "object_detector_msgs/ObjectPositions.h"

// c++
#include <iomanip>
#include <fstream>
#include <sstream>

// utiils
#include "object_detection_recorder/observation.h"
#include "recorded_objects/recorded_objects.h"

namespace recorder
{
class ObjectDetectionRecorder
{
public:
	ObjectDetectionRecorder();
	~ObjectDetectionRecorder();
	void process();

private:
	void od_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);
	void filter_od(object_detector_msgs::ObjectPositions input_od,object_detector_msgs::ObjectPositions& output_od);

	bool is_visible_range(object_detector_msgs::ObjectPosition od);
	std::string get_date();

	// node handler
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	// subscriber
	ros::Subscriber od_sub_;

	// buffer
	RecordedObjects* recorded_objects_;
	std::vector<Observation> observations_;
	ros::Time start_time_;
	bool is_first_;

	// params
	bool IS_RECORD_;
	int HZ_;
	double PROBABILITY_TH_;
	double ANGLE_OF_VIEW_;
	double VISIBLE_LOWER_DISTANCE_;
	double VISIBLE_UPPER_DISTANCE_;


};
} // namespace recorder

#endif  // OBJECT_DETECTION_RECORDER_H_