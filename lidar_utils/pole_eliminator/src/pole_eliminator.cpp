#include "pole_eliminator/pole_eliminator.h"

PoleEliminator::PoleEliminator() : 
    private_nh_("~")
{
    private_nh_.param("LASER_FRAME_ID",LASER_FRAME_ID_,{std::string("")});
    private_nh_.param("MARGIN_",MARGIN_,{15});
    private_nh_.param("ROBOT_RADIUS_",ROBOT_RADIUS_,{0.30});
    private_nh_.param("LINEAR_INTERPOLATE_THRESHOLD_",LINEAR_INTERPOLATE_THRESHOLD_,{M_PI/18.0});

    raw_scan_sub_ = nh_.subscribe("scan",10,&PoleEliminator::scan_callback,this);
    corrected_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("corrected_scan",1);
}

void PoleEliminator::scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    const size_t laser_size = msg->ranges.size();
    ROS_ASSERT(laser_size <= MAX_LASER_SIZE_);

    static std::array<bool,MAX_LASER_SIZE_> is_in_radiuses;
    laser_is_in_radius(msg,&is_in_radiuses);
    extend_radius(MARGIN_,laser_size,&is_in_radiuses);

    std::vector<std::pair<size_t,size_t>> pole_ranges;
    ranges_is_in_radius(laser_size,is_in_radiuses,&pole_ranges);
    ROS_DEBUG("Auto detector poles: ");
    for(const auto &range : pole_ranges)
        ROS_DEBUG_STREAM("[" << range.first << "," << range.second << ")");

    std::vector<std::function<double(double)>> linear_interpolation_funcs;
    register_calculation_function(msg,pole_ranges,&linear_interpolation_funcs);
    ROS_ASSERT(pole_ranges.size() == linear_interpolation_funcs.size());

    sensor_msgs::LaserScan corrected_laser = *msg;
    if(!LASER_FRAME_ID_.empty()) corrected_laser.header.frame_id = LASER_FRAME_ID_;
    for(size_t range_idx = 0; range_idx < pole_ranges.size(); range_idx++) {
        for(size_t i = pole_ranges[range_idx].first; i < pole_ranges[range_idx].second; i++){
            double angle = get_ranges_from_index_to_angle(i,corrected_laser.angle_min,corrected_laser.angle_increment);
            corrected_laser.ranges[i] = linear_interpolation_funcs[range_idx](angle);
            if (corrected_laser.ranges[i] < ROBOT_RADIUS_) corrected_laser.ranges[i] = 30.0;
        }
    }
    corrected_scan_pub_.publish(corrected_laser);
}

void PoleEliminator::laser_is_in_radius(const sensor_msgs::LaserScanConstPtr& msg,
                                        std::array<bool,MAX_LASER_SIZE_>* is_in_radiuses_ptr)
{
    for(size_t i = 0; i < msg->ranges.size(); i++){
        if(msg->ranges[i] < ROBOT_RADIUS_) is_in_radiuses_ptr->at(i) = true;
        else is_in_radiuses_ptr->at(i) = false;
    }
}

void PoleEliminator::extend_radius(int mergin,const size_t laser_size,
                                   std::array<bool,MAX_LASER_SIZE_> *is_in_radiuses_ptr)
{
    std::vector<int> extend_forward, extend_back;
    for(size_t i = 0; i < laser_size; i++){
        if(i > 0 && !(is_in_radiuses_ptr->at(i - 1)) && is_in_radiuses_ptr->at(i))
            extend_back.push_back(i);
        if(i < laser_size - 1 && is_in_radiuses_ptr->at(i) && !(is_in_radiuses_ptr->at(i + 1)))
            extend_forward.push_back(i);
    }

    for(auto idx : extend_back){
        for(size_t i = (size_t)std::max(0,idx - mergin); i < (size_t)idx; i++)
            is_in_radiuses_ptr->at(i) = true;
    }

    for(auto idx : extend_forward){
        for(size_t i = idx + 1; i <= (size_t)std::min(idx + mergin,static_cast<int>(laser_size) - 1); i++)
            is_in_radiuses_ptr->at(i) = true;
    }
}

void PoleEliminator::ranges_is_in_radius(const size_t laser_size,
                                         const std::array<bool,MAX_LASER_SIZE_>& is_in_radiuses,
                                         std::vector<std::pair<size_t,size_t>>* pole_ranges_ptr)
{
    bool is_start = false;
    size_t start_idx, end_idx;
    for(size_t i = 0; i < laser_size; i++){
        if(!is_start && is_in_radiuses[i]){
            start_idx = i;
            is_start = true;
        }
        if(is_start && !(is_in_radiuses[i])){
            end_idx = i;
            is_start = false;
            pole_ranges_ptr->emplace_back(start_idx,end_idx);
        }
    }
    if(is_start) pole_ranges_ptr->emplace_back(start_idx,laser_size);
}

void PoleEliminator::register_calculation_function(const sensor_msgs::LaserScanConstPtr &msg,
                                                   const std::vector<std::pair<size_t, size_t>> &pole_ranges,
                                                   std::vector<std::function<double(double)>> *linear_interpolation_funcs_ptr)
{
    for(const auto &range : pole_ranges) {
        ROS_ASSERT(!(range.first == 0 && range.second == msg->ranges.size()));
        if(range.first == 0)
            linear_interpolation_funcs_ptr->push_back([&](double angle) { return msg->ranges[range.second]; });
        else if(range.second == msg->ranges.size())
            linear_interpolation_funcs_ptr->push_back([&](double angle) { return msg->ranges[range.first]; });
        else if(is_linear_interpolated(msg,range.first,range.second)) {
            double b, c;
            calc_constant_bc(msg,range.first,range.second - 1,&b,&c);
            linear_interpolation_funcs_ptr->push_back(std::bind(&PoleEliminator::get_range,this,b,c,std::placeholders::_1));
        }else{
            linear_interpolation_funcs_ptr->push_back([&](double angle) { return std::min(msg->ranges[range.first],msg->ranges[range.second]); });
        }
    }
}

void PoleEliminator::calc_constant_bc(const sensor_msgs::LaserScanConstPtr &msg,
                                      size_t min_idx,size_t max_idx,double *b_ptr,double *c_ptr)
{
    double min_theta = get_ranges_from_index_to_angle(min_idx,msg->angle_min,msg->angle_increment);
    double max_theta = get_ranges_from_index_to_angle(max_idx,msg->angle_min,msg->angle_increment);

    double min_point_x = msg->ranges[min_idx]*std::cos(min_theta);
    double min_point_y = msg->ranges[min_idx]*std::sin(min_theta);
    double max_point_x = msg->ranges[max_idx]*std::cos(max_theta);
    double max_point_y = msg->ranges[max_idx]*std::sin(max_theta);

    if (min_point_x == max_point_x) ROS_FATAL_STREAM("PoleEliminator::calc_constant_bc contains a division by zero");

    // y = bx + c
    *b_ptr = (min_point_y - max_point_y) / (min_point_x - max_point_x);
    *c_ptr = (max_point_x * min_point_y - min_point_x * max_point_y) / (max_point_x - min_point_x);
}


bool PoleEliminator::is_linear_interpolated(const sensor_msgs::LaserScanConstPtr &msg,
                                            size_t min_idx, size_t max_idx)
{
    double pre_b, b, next_b, c;
    if(min_idx - MARGIN_ < 0 || msg->ranges.size() <= max_idx + MARGIN_) return false;
    calc_constant_bc(msg,min_idx - MARGIN_,min_idx,&pre_b,&c);
    double pre_theta = std::atan(pre_b);
    calc_constant_bc(msg,min_idx,max_idx,&b,&c);
    double theta = std::atan(b);
    calc_constant_bc(msg,max_idx,max_idx + MARGIN_,&next_b,&c);
    double next_theta = std::atan(next_b);
    if(std::abs(pre_theta - theta) < LINEAR_INTERPOLATE_THRESHOLD_ && std::abs(theta - next_theta) < LINEAR_INTERPOLATE_THRESHOLD_)
        return true;
    return false;
}

double PoleEliminator::get_ranges_from_index_to_angle(size_t index,double angle_min,double angle_increase)
{
    return angle_min + index * angle_increase;
}

double PoleEliminator::get_range(double b,double c,double angle)
{
    static auto is_equal = [](double val1, double val2) -> bool {
        constexpr double eps = 0.0001;
        if (std::abs(val1 - val2) < eps) return true;
        return false;
    };

    double cross_x, cross_y;
    if(is_equal(0.0, angle) || is_equal(M_PI, angle) || is_equal(-M_PI,angle)){
        cross_x = -c / b;
        cross_y = 0.0;
    }else if(is_equal(M_PI/2.0,angle) || is_equal(-M_PI/2.0,angle)){
        cross_x = 0.0;
        cross_y = c;
    }else {                             // cross_y = b * cross_x + c
        double a = std::tan(angle);     // a = cross_y / cross_x
        if(a == b) ROS_FATAL_STREAM("PoleEliminator::calc_range contains a division by zero");
        cross_x = c / (a - b);
        cross_y = a * c / (a - b);
    }

    return std::sqrt(cross_x * cross_x + cross_y * cross_y);
}

void PoleEliminator::process() { ros::spin(); }