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
#include <tf2/utils.h>

class RosbagTF
{
public:
	RosbagTF();
	~RosbagTF();
	void process();

private:
	void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

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

	// buffer
	geometry_msgs::PoseStamped pose_;
	nav_msgs::Odometry odom_;

	// params
    std::string MAP_FRAME_ID_;
    std::string BASE_LINK_FRAME_ID_;
};

#endif	// ROSBAG_TF_H_