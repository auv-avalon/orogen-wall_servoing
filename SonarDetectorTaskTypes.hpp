#ifndef SONAR_DETECTION_TASK_TYPES_HPP_
#define SONAR_DETECTION_TASK_TYPES_HPP_

#include <vector>
#include <base/eigen.h>
#include <base/time.h>
#include <base/samples/pointcloud.h>
#include <base/float.h>

namespace sonar_detectors
{

struct WallServoingDebugData
{
    base::samples::Pointcloud pointCloud;
    base::Time time;
    std::vector<base::Vector3d> wall;
    base::Vector3d relative_wall_position;
    double wall_angle;
    double wall_distance;
    WallServoingDebugData()
    : time(base::Time::now()), wall_angle(base::unknown<double>()), wall_distance(base::unknown<double>()){}
};

struct DualWallServoingDebugData
{
    base::Time time;
    double wall_distance_front;
    double wall_distance_right;
    double wall_angle;
    DualWallServoingDebugData()
    : time(base::Time::now()) , wall_distance_front(base::unknown<double>()), wall_distance_right(base::unknown<double>()), wall_angle(base::unknown<double>()){}
};

}

#endif
