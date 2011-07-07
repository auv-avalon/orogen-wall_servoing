#ifndef SONAR_DETECTION_TASK_TYPES_HPP_
#define SONAR_DETECTION_TASK_TYPES_HPP_

#include <vector>
#include <base/eigen.h>
#include <base/time.h>

namespace avalon
{

struct wallDetectionData
{
    std::vector<base::Vector3d> pointCloud;
    base::Time time;
    base::Vector3d pose_vector;
    base::Vector3d direction_vector;
    base::Vector3d relative_wall_position;
    double distance;
    wallDetectionData()
    : distance(0){}
};

}

#endif