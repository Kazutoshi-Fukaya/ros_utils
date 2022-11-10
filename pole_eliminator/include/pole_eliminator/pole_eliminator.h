#ifndef POLE_ELIMINATOR_H_
#define POLE_ELIMINATOR_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class PoleEliminator
{
public:
    PoleEliminator();
    void process();

private:
    static constexpr size_t MAX_LASER_SIZE_ = 10000;

private:
    void scan_callback(const sensor_msgs::LaserScanConstPtr& msg);
    void laser_is_in_radius(const sensor_msgs::LaserScanConstPtr& msg,
                            std::array<bool,MAX_LASER_SIZE_>* is_in_radiuses_ptr);
    void extend_radius(int mergin,const size_t laser_size,
                       std::array<bool,MAX_LASER_SIZE_>* is_in_radiuses_ptr);
    void ranges_is_in_radius(const size_t laser_size,
                             const std::array<bool,MAX_LASER_SIZE_>& is_in_radiuses,
                             std::vector<std::pair<size_t,size_t>>* pole_ranges_ptr);
    void register_calculation_function(const sensor_msgs::LaserScanConstPtr &msg,
                                       const std::vector<std::pair<size_t, size_t>> &pole_ranges,
                                       std::vector<std::function<double(double)>> *linear_interpolation_funcs_ptr);
    void calc_constant_bc(const sensor_msgs::LaserScanConstPtr &msg,
                          size_t min_idx,size_t max_idx,double *b_ptr,double *c_ptr);

    bool is_linear_interpolated(const sensor_msgs::LaserScanConstPtr &msg,
                                size_t min_idx, size_t max_idx);
    double get_ranges_from_index_to_angle(size_t index,double angle_min,double angle_increase);
    double get_range(double b,double c,double angle);

    // node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber raw_scan_sub_;

    // publisher
    ros::Publisher corrected_scan_pub_;

    // parameters
    std::string LASER_FRAME_ID_;
    int MARGIN_;
    double ROBOT_RADIUS_;
    double LINEAR_INTERPOLATE_THRESHOLD_;
};

#endif  // POLE_ELIMINATOR_H_
