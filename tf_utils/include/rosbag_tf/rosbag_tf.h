#ifndef ROSBAG_TF_H_
#define ROSBAG_TF_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/utils.h>

#include "rosbag_tf/tf_pose.h"

class RosbagTF
{
public:
    RosbagTF();
    ~RosbagTF();
    void process();

private:
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    geometry_msgs::TransformStamped get_transform_stamped(std::string frame_id,std::string child_frame_id,TFPose pose);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;

    // publisher
    ros::Publisher pose_pub_;

    // tf
    boost::shared_ptr<tf2_ros::Buffer> buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> listener_;
    boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    boost::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    // buffer
    geometry_msgs::PoseStamped pose_;
    nav_msgs::Odometry odom_;
    geometry_msgs::TransformStamped laser_transform_stamped_;
    geometry_msgs::TransformStamped camera_transform_stamped_;

    // params
    bool PUBLISH_POSE_;
    std::string MAP_FRAME_ID_;
    std::string BASE_LINK_FRAME_ID_;
    std::string LASER_FRAME_ID_;
    std::string CAMERA_FRAME_ID_;
};

#endif  // ROSBAG_TF_H_
