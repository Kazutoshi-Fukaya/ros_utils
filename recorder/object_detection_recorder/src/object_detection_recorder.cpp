#include "object_detection_recorder/object_detection_recorder.h"

using namespace recorder;

ObjectDetectionRecorder::ObjectDetectionRecorder() :
	private_nh_("~"),
	recorded_objects_(new RecordedObjects(private_nh_)),
	is_first_(true)
{
	private_nh_.param("HZ",HZ_,{10});
	private_nh_.param("PROBABILITY_TH",PROBABILITY_TH_,{0.8});
	private_nh_.param("ANGLE_OF_VIEW",ANGLE_OF_VIEW_,{86.0/180.0*M_PI});
    private_nh_.param("VISIBLE_LOWER_DISTANCE",VISIBLE_LOWER_DISTANCE_,{0.3});
    private_nh_.param("VISIBLE_UPPER_DISTANCE",VISIBLE_UPPER_DISTANCE_,{5.0});

	private_nh_.param("IS_RECORD",IS_RECORD_,{false});
	if(IS_RECORD_){
		od_sub_ = nh_.subscribe("od_in",1,&ObjectDetectionRecorder::od_callback,this);
	}
}

ObjectDetectionRecorder::~ObjectDetectionRecorder()
{
	if(IS_RECORD_){
		if(observations_.empty()){
			ROS_INFO("'observations' are empty");
			return;
    	}
		std::string record_file_path;
		private_nh_.param("RECORD_FILE_PATH",record_file_path,{std::string("")});
		std::string file_name = record_file_path + "observation_" + get_date() + ".csv";
    	static std::ofstream ofs(file_name);
    	for(const auto &obs : observations_){
			ofs << obs.time << "," << obs.name << std::endl;
    	}
    	ofs.close();
		ROS_INFO("Save: %s", file_name.c_str());
	}
}

void ObjectDetectionRecorder::od_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{
	if(is_first_){
		start_time_ = msg->header.stamp;
		is_first_ = false;
	}
	
	object_detector_msgs::ObjectPositions filtered_od;
	filter_od(*msg,filtered_od);
	if(filtered_od.object_position.empty()) return;
	double time = (msg->header.stamp - start_time_).toSec();
	for(const auto &od : filtered_od.object_position){
		Observation observation;
		observation.time = time;
		observation.name = od.Class;
		observations_.emplace_back(observation);
	}
}

void ObjectDetectionRecorder::filter_od(object_detector_msgs::ObjectPositions input_od,object_detector_msgs::ObjectPositions& output_od)
{
	output_od.header = input_od.header;
    output_od.object_position.clear();    
    for(const auto &inp_od : input_od.object_position){
        if(is_visible_range(inp_od)) output_od.object_position.emplace_back(inp_od);
    }
}

bool ObjectDetectionRecorder::is_visible_range(object_detector_msgs::ObjectPosition od)
{
	if(!recorded_objects_->is_included(od.Class)) return false;
    if(od.probability <= PROBABILITY_TH_) return false;

    double r_vertex_x = std::cos(0.5*(M_PI - ANGLE_OF_VIEW_));
    double r_vertex_y = std::sin(0.5*(M_PI - ANGLE_OF_VIEW_));
    double l_vertex_x = std::cos(0.5*(M_PI + ANGLE_OF_VIEW_));
    double l_vertex_y = std::sin(0.5*(M_PI + ANGLE_OF_VIEW_));

    double dist = std::sqrt(od.x*od.x + od.z*od.z);
    if(VISIBLE_LOWER_DISTANCE_ < dist && dist < VISIBLE_UPPER_DISTANCE_){
        double x = od.x;
        double y = od.z;
        if(r_vertex_x*y - x*r_vertex_y >= 0 && l_vertex_x*y - x*l_vertex_y <= 0){
            return true;
        }
    }
    return false;
}

std::string ObjectDetectionRecorder::get_date()
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

void ObjectDetectionRecorder::process()
{
	ros::Rate rate(HZ_);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
}