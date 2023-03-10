#include "rosbag_tf/rosbag_tf.h"

RosbagTF::RosbagTF() :
	private_nh_("~")
{
	private_nh_.param("MAP_FRAME_ID",MAP_FRAME_ID_,{std::string("map")});
    private_nh_.param("BASE_LINK_FRAME_ID",BASE_LINK_FRAME_ID_,{std::string("base_link")});
    private_nh_.param("LASER_FRAME_ID",LASER_FRAME_ID_,{std::string("laser")});
    private_nh_.param("CAMERA_FRAME_ID",CAMERA_FRAME_ID_,{std::string("camera")});

    TFPose LASER_POSE;
    private_nh_.param("LASER_X",LASER_POSE.x,{0.0});
    private_nh_.param("LASER_Y",LASER_POSE.y,{0.0});
    private_nh_.param("LASER_Z",LASER_POSE.z,{0.0});
    private_nh_.param("LASER_ROLL",LASER_POSE.roll,{0.0});
    private_nh_.param("LASER_PITCH",LASER_POSE.pitch,{0.0});
    private_nh_.param("LASER_YAW",LASER_POSE.yaw,{0.0});
    laser_transform_stamped_ = get_transform_stamped(BASE_LINK_FRAME_ID_,LASER_FRAME_ID_,LASER_POSE);

    TFPose CAMERA_POSE;
    private_nh_.param("CAMERA_X",CAMERA_POSE.x,{0.0});
    private_nh_.param("CAMERA_Y",CAMERA_POSE.y,{0.0});
    private_nh_.param("CAMERA_Z",CAMERA_POSE.z,{0.0});
    private_nh_.param("CAMERA_ROLL",CAMERA_POSE.roll,{0.0});
    private_nh_.param("CAMERA_PITCH",CAMERA_POSE.pitch,{0.0});
    private_nh_.param("CAMERA_YAW",CAMERA_POSE.yaw,{0.0});
    camera_transform_stamped_ = get_transform_stamped(BASE_LINK_FRAME_ID_,CAMERA_FRAME_ID_,CAMERA_POSE);
   
    bool publish_pose;
    private_nh_.param("PUBLISH_POSE",publish_pose,{false});
    if(publish_pose){
        pose_sub_ = nh_.subscribe("pose_in",1,&RosbagTF::pose_callback,this);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_out",1);
    }

    odom_sub_ = nh_.subscribe("odom_in",1,&RosbagTF::odom_callback,this);

    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));
    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
    static_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster);
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

geometry_msgs::TransformStamped RosbagTF::get_transform_stamped(std::string frame_id,std::string child_frame_id,TFPose pose)
{
    geometry_msgs::TransformStamped static_transform;
    static_transform.header.stamp = ros::Time::now();
    static_transform.header.frame_id = frame_id;
    static_transform.child_frame_id = child_frame_id;
    static_transform.transform.translation.x = pose.x;
    static_transform.transform.translation.y = pose.y;
    static_transform.transform.translation.z = pose.z;
    tf2::Quaternion tf_q;
    tf_q.setRPY(pose.roll,pose.pitch,pose.yaw);
    static_transform.transform.rotation.x = tf_q.x();
    static_transform.transform.rotation.y = tf_q.y();
    static_transform.transform.rotation.z = tf_q.z();
    static_transform.transform.rotation.w = tf_q.w();

    return static_transform;
}

void RosbagTF::process() 
{
    static_broadcaster_->sendTransform({laser_transform_stamped_, camera_transform_stamped_});
    ros::spin(); 
}