#ifndef OBJECTS_DATA_PUBLISHER_H_
#define OBJECTS_DATA_PUBLISHER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

// Custom msg
#include "object_detector_msgs/ObjectPositions.h"
#include "multi_localizer_msgs/ObjectsData.h"

class ObjectsDataPublisher
{
public:
    ObjectsDataPublisher();
	~ObjectsDataPublisher();
    void process();

private:
    void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void op_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);

    void filter_op(object_detector_msgs::ObjectPositions input_op,object_detector_msgs::ObjectPositions& output_op);
    bool is_visible_range(object_detector_msgs::ObjectPosition op);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber pose_sub_;
    ros::Subscriber op_sub_;

    // publisher
    ros::Publisher od_pub_;

    // buffer
    ros::Time start_time_;
    geometry_msgs::PoseStamped pose_;
    bool is_first_;

    // params
    std::string ROBOT_NAME_;
    std::string MAP_FRAME_ID_;
    bool PUBLISH_OBJ_MSG_;
    double PROBABILITY_TH_;
    double ANGLE_OF_VIEW_;
    double VISIBLE_LOWER_DISTANCE_;
    double VISIBLE_UPPER_DISTANCE_;
};

#endif	// OBJECTS_DATA_PUBLISHER_H_