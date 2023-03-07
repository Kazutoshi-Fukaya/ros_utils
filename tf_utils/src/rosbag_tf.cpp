#include "rosbag_tf/rosbag_tf.h"

RosbagTF::RosbagTF() :
	private_nh_("~")
{
	private_nh_.param("MAP_FRAME_ID",MAP_FRAME_ID_,{std::string("map")});
    private_nh_.param("BASE_LINK_FRAME_ID",BASE_LINK_FRAME_ID_,{std::string("base_link")});

    pose_sub_ = nh_.subscribe("pose_in",1,&RosbagTF::pose_callback,this);
    odom_sub_ = nh_.subscribe("odom_in",1,&RosbagTF::odom_callback,this);
   
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_out",1);

    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));
    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
}

RosbagTF::~RosbagTF() {}

void RosbagTF::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
	odom_ = *msg;
    geometry_msgs::TransformStamped odom_transform;
    odom_transform.header = msg->header;
    odom_transform.child_frame_id = msg->child_frame_id;
    odom_transform.transform.translation.x = msg->pose.pose.position.x;
    odom_transform.transform.translation.y = msg->pose.pose.position.y;
    odom_transform.transform.translation.z = msg->pose.pose.position.z;
    odom_transform.transform.rotation = msg->pose.pose.orientation;
    broadcaster_->sendTransform(odom_transform);
}

void RosbagTF::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	pose_.header = msg->header;
    pose_.pose = msg->pose.pose;
    pose_pub_.publish(pose_);

    tf2::Quaternion q;
    q.setRPY(0.0,0.0,tf2::getYaw(pose_.pose.orientation));
    tf2::Transform map_to_robot(q,tf2::Vector3(pose_.pose.position.x,pose_.pose.position.y,0.0));

    geometry_msgs::PoseStamped robot_to_map_pose;
    robot_to_map_pose.header.frame_id = BASE_LINK_FRAME_ID_;
    robot_to_map_pose.header.stamp = odom_.header.stamp;
    tf2::toMsg(map_to_robot.inverse(),robot_to_map_pose.pose);

    geometry_msgs::PoseStamped odom_to_map_pose;
    try{
        buffer_->transform(robot_to_map_pose,odom_to_map_pose,odom_.header.frame_id);
    }
    catch(tf2::TransformException& ex){
        ROS_WARN("%s", ex.what());
        return;
    }
    tf2::Transform odom_to_map;
    tf2::convert(odom_to_map_pose.pose,odom_to_map);
    geometry_msgs::TransformStamped map_to_odom_transform;
    map_to_odom_transform.header.stamp = odom_.header.stamp;
    map_to_odom_transform.header.frame_id = MAP_FRAME_ID_;
    map_to_odom_transform.child_frame_id = odom_.header.frame_id;
    tf2::convert(odom_to_map.inverse(),map_to_odom_transform.transform);
    broadcaster_->sendTransform(map_to_odom_transform);
}

void RosbagTF::process() { ros::spin(); }