#ifndef SONAR_DETECTION_TASK_TYPES_HPP_
#define SONAR_DETECTION_TASK_TYPES_HPP_

#include <vector>
#include <base/eigen.h>
#include <base/time.h>
#include <base/samples/pointcloud.h>

namespace sonar_detectors
{

struct wallServoingDebugData
{
    base::samples::Pointcloud pointCloud;
    base::Time time;
    std::vector<base::Vector3d> wall;
    base::Vector3d relative_wall_position;
    double wall_angle;
    double wall_distance;
    wallServoingDebugData()
    : time(base::Time::now()), wall_angle(0), wall_distance(0){}
};

}

#endif