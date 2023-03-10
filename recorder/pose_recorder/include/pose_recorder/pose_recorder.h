#ifndef POSE_RECORDER_H_
#define POSE_RECORDER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

#include <sstream>
#include <fstream>

#include "pose_recorder/trajectory.h"

namespace recorder
{
class PoseRecorder
{
public:
    PoseRecorder();
    ~PoseRecorder();
    void process();

private:
    void est_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void ref_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    void record_pose();
    Pose get_pose_from_msg(geometry_msgs::PoseStamped msg);
    std::string get_date();

    // node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber est_pose_sub_;
    ros::Subscriber ref_pose_sub_;

    // buffer
    std::vector<Trajectory> trajectories_;
    ros::Time start_time_;
    geometry_msgs::PoseStamped est_pose_;
    geometry_msgs::PoseStamped ref_pose_;
    bool is_first_;

    // param
    bool IS_RECORD_;
    int HZ_;
};
} // namespace recorder

#endif  // POSE_RECORDER_H_
