#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "pose_recorder/pose.h"

namespace recorder
{
class Trajectory
{
public:
    Trajectory() :
        time(0.0), est_pose(Pose()), ref_pose(Pose()) {}

    double time;
    Pose est_pose;
    Pose ref_pose;

private:
};
} // namespace recorder

#endif  // TRAJECTORY_H_
