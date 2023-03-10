#ifndef RECORDED_OBJECTS_H_
#define RECORDED_OBJECTS_H_

#include <ros/ros.h>

namespace recorder
{
class RecordedObjects : public std::vector<std::string>
{
public:
	RecordedObjects();
    RecordedObjects(ros::NodeHandle private_nh);

    bool is_included(std::string name);
    void print_elements();  // for debug

private:
    void load_yaml(ros::NodeHandle private_nh);
};
} // namespace recorder

#endif	// RECORDED_OBJECTS_H_