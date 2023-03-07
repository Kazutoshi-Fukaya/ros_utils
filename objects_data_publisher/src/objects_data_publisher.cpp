#include "objects_data_publisher/objects_data_publisher.h"

ObjectsDataPublisher::ObjectsDataPublisher() :
    private_nh_("~"),
    is_first_(true)
{
    private_nh_.param("ROBOT_NAME",ROBOT_NAME_,{std::string("")});
    private_nh_.param("MAP_FRAME_ID",MAP_FRAME_ID_,{std::string("map")});
    private_nh_.param("PROBABILITY_TH",PROBABILITY_TH_,{0.8});
    private_nh_.param("ANGLE_OF_VIEW",ANGLE_OF_VIEW_,{86.0/180.0*M_PI});
    private_nh_.param("VISIBLE_LOWER_DISTANCE",VISIBLE_LOWER_DISTANCE_,{0.1});
    private_nh_.param("VISIBLE_UPPER_DISTANCE",VISIBLE_UPPER_DISTANCE_,{5.0});

    pose_sub_ = nh_.subscribe("pose_in",1,&ObjectsDataPublisher::pose_callback,this);
    op_sub_ = nh_.subscribe("op_in",1,&ObjectsDataPublisher::op_callback,this);

    od_pub_ = nh_.advertise<multi_localizer_msgs::ObjectsData>("od_out",1);
}

ObjectsDataPublisher::~ObjectsDataPublisher() {}

void ObjectsDataPublisher::pose_callback(const geometry_msgs::PoseStampedConstPtr& msg) { pose_ = *msg; }

void ObjectsDataPublisher::op_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{
    if(is_first_){
        start_time_= msg->header.stamp;
        is_first_ = false;
    }

    multi_localizer_msgs::ObjectsData objects;
    objects.header.frame_id = MAP_FRAME_ID_;
    objects.header.stamp = msg->header.stamp;
    objects.pose.name = ROBOT_NAME_;
    objects.pose.weight = 1.0;
    objects.pose.x = pose_.pose.position.x;
    objects.pose.y = pose_.pose.position.y;
    objects.pose.theta = tf2::getYaw(pose_.pose.orientation);

    object_detector_msgs::ObjectPositions filtered_op;
    filter_op(*msg,filtered_op);
    if(filtered_op.object_position.empty()) return;

    // objects
    for(const auto & p : filtered_op.object_position){
        double dist = std::sqrt(p.x*p.x + p.z*p.z);
        double angle = std::atan2(p.z,p.x) - 0.5*M_PI;

        multi_localizer_msgs::ObjectData object;
        object.name = p.Class;
        object.time = (msg->header.stamp - start_time_).toSec();
        object.credibility = 1.0;
        double yaw = tf2::getYaw(pose_.pose.orientation);
        object.x = pose_.pose.position.x + dist*std::cos(yaw + angle);
        object.y = pose_.pose.position.y + dist*std::sin(yaw + angle);
        objects.data.emplace_back(object);
    }
    od_pub_.publish(objects);
}

void ObjectsDataPublisher::filter_op(object_detector_msgs::ObjectPositions input_op,object_detector_msgs::ObjectPositions& output_op)
{
    output_op.header = input_op.header;
    output_op.object_position.clear();
    for(const auto & p : input_op.object_position){
        if(is_visible_range(p)) output_op.object_position.emplace_back(p);
    }
}

bool ObjectsDataPublisher::is_visible_range(object_detector_msgs::ObjectPosition op)
{
    if(op.Class == "roomba") return false;
    if(op.probability < PROBABILITY_TH_) return false;

    double r_vertex_x = std::cos(0.5*(M_PI - ANGLE_OF_VIEW_));
    double r_vertex_y = std::sin(0.5*(M_PI - ANGLE_OF_VIEW_));
    double l_vertex_x = std::cos(0.5*(M_PI + ANGLE_OF_VIEW_));
    double l_vertex_y = std::sin(0.5*(M_PI + ANGLE_OF_VIEW_));

    double dist = std::sqrt(op.x*op.x + op.z*op.z);
    if(VISIBLE_LOWER_DISTANCE_ < dist && dist < VISIBLE_UPPER_DISTANCE_){
        double x = op.x;
        double y = op.z;
        if(r_vertex_x*y - x*r_vertex_y >= 0 && l_vertex_x*y - x*l_vertex_y <= 0) return true;
    }
    return false;
}

void ObjectsDataPublisher::process() { ros::spin(); }
