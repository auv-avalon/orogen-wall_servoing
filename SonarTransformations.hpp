#ifndef SONAR_TRASFORMATIONS_HPP_
#define SONAR_TRASFORMATIONS_HPP_

#include <Eigen/Geometry>

namespace avalon
{
    static const Eigen::Affine3d sonarFront2Center = Eigen::Translation3d(Eigen::Vector3d(0.3295, 0.0000, 0.2765)) * Eigen::AngleAxisd::Identity();
    static const Eigen::Affine3d sonarRear2Center = Eigen::Translation3d(Eigen::Vector3d(-0.8420, -0.0500, -0.0985)) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    static const Eigen::Affine3d sonarRear2sonarFront = sonarFront2Center.inverse() * sonarRear2Center;
}

#endif
