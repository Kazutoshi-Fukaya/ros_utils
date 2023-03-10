#ifndef TF_POSE_H_
#define TF_POSE_H_

class TFPose
{
public:
	TFPose() :
		x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0) {}

	TFPose(double _x,double _y,double _z,double _roll,double _pitch,double _yaw) :
		x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw) {}

	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;

private:
};

#endif	// TF_POSE_H_