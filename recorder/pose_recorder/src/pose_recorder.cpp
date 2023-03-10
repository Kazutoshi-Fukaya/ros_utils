#include "pose_recorder/pose_recorder.h"

using namespace recorder;

PoseRecorder::PoseRecorder() :
    private_nh_("~"),
    is_first_(true)
{
    private_nh_.param("HZ",HZ_,{1});

    private_nh_.param("IS_RECORD",IS_RECORD_,{false});
    if(IS_RECORD_){
        est_pose_sub_ = nh_.subscribe("est_pose",1,&PoseRecorder::est_pose_callback,this);
        ref_pose_sub_ = nh_.subscribe("ref_pose",1,&PoseRecorder::ref_pose_callback,this);
    }
}

PoseRecorder::~PoseRecorder()
{
    if(IS_RECORD_){
        std::string file_path;
        private_nh_.param("RECORD_FILE_PATH",file_path,{std::string("")});
        std::string file_name = file_path + "_" + get_date() +"_traj.txt";
        static std::ofstream ofs(file_name);
        for(const auto &t : trajectories_){
            ofs << t.time << ","
                << t.est_pose.x << "," << t.est_pose.y << "," << t.est_pose.yaw << ","
                << t.ref_pose.x << "," << t.ref_pose.y << "," << t.ref_pose.yaw << std::endl;
        }
        ofs.close();
    }
}

void PoseRecorder::est_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    est_pose_ = *msg;
}

void PoseRecorder::ref_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    ref_pose_.header = msg->header;
    ref_pose_.pose = msg->pose.pose;
}

void PoseRecorder::record_pose()
{
    ros::Time now_time = ros::Time::now();
    if(is_first_){
        start_time_ = now_time;
        is_first_ = false;
    }

    Trajectory trajectory;
    trajectory.time = (now_time - start_time_).toSec();
    trajectory.est_pose = get_pose_from_msg(est_pose_);
    trajectory.ref_pose = get_pose_from_msg(ref_pose_);
    trajectories_.emplace_back(trajectory);
}

Pose PoseRecorder::get_pose_from_msg(geometry_msgs::PoseStamped msg)
{
    Pose pose;
    pose.x = msg.pose.position.x;
    pose.y = msg.pose.position.y;
    pose.yaw = tf2::getYaw(msg.pose.orientation);
    return pose;
}

std::string PoseRecorder::get_date()
{
    time_t t = time(nullptr);
    const tm* localTime = localtime(&t);
    std::stringstream s;
    s << localTime->tm_year + 1900;
    s << std::setw(2) << std::setfill('0') << localTime->tm_mon + 1;
    s << std::setw(2) << std::setfill('0') << localTime->tm_mday;
    s << std::setw(2) << std::setfill('0') << localTime->tm_hour;
    s << std::setw(2) << std::setfill('0') << localTime->tm_min;
    s << std::setw(2) << std::setfill('0') << localTime->tm_sec;

    return s.str();
}

void PoseRecorder::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        record_pose();
        ros::spinOnce();
        rate.sleep();
    }
}
