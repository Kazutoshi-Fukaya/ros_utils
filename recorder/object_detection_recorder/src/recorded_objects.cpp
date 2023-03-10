#include "recorded_objects/recorded_objects.h"

using namespace recorder;

RecordedObjects::RecordedObjects()
{
	ROS_WARN("'recorded_objects' has not received 'private_nh'");
}

RecordedObjects::RecordedObjects(ros::NodeHandle private_nh)
{
	load_yaml(private_nh);
}

void RecordedObjects::load_yaml(ros::NodeHandle private_nh)
{
	std::string yaml_file_name;
	private_nh.param("YAML_FILE_NAME",yaml_file_name,{std::string("recorded_objects")});
    XmlRpc::XmlRpcValue recorded_objects;
    if(!private_nh.getParam(yaml_file_name.c_str(),recorded_objects)){
		ROS_ERROR("Cloud not load %s", yaml_file_name.c_str());
        return;
    }

    ROS_ASSERT(recorded_objects.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < recorded_objects.size(); i++){
		if(!recorded_objects[i]["name"].valid()){
			ROS_ERROR("%s is valid", yaml_file_name.c_str());
            return;
        }
        if(recorded_objects[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString){
			std::string name = static_cast<std::string>(recorded_objects[i]["name"]);
            this->emplace_back(name);
        }
    }
}

void RecordedObjects::print_elements()
{
	std::cout << "=== RECORDED OBJECTS ===" << std::endl;
	for(auto it = this->begin(); it != this->end(); it++){
		std::cout << "name: " << *it << std::endl;
    }
    std::cout << std::endl;
}

bool RecordedObjects::is_included(std::string name)
{
	for(auto it = this->begin(); it != this->end(); it++){
		if(name ==*it) return true;
    }
    return false;
}