#ifndef SONAR_DETECTION_TASK_TYPES_HPP_
#define SONAR_DETECTION_TASK_TYPES_HPP_

#include <vector>
#include <base/eigen.h>
#include <base/time.h>
#include <base/samples/pointcloud.h>

namespace avalon
{

struct wallDetectionData
{
    base::samples::Pointcloud pointCloud;
    base::Time time;
    std::vector<base::Vector3d> wall;
    base::Vector3d relative_wall_position;
    double distance;
    wallDetectionData()
    : distance(0), time(base::Time::now()){}
};

}

#endif