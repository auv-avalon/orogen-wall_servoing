/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <sonar_detectors/SonarDetectorTypes.hpp>
#include <SonarDetectorTaskTypes.hpp>
#include <sonar_detectors/SonarDetectorMath.hpp>
#include <base/samples/pointcloud.h>

using namespace wall_servoing;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
    , wallEstimation(0), distanceEstimation(0)
{
    
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

// bool Task::configureHook()
// {
//     return true;
// }

bool Task::startHook()
{
    last_state = PRE_OPERATIONAL;
    checking_count = 0;
    last_distance_to_wall = -1;
    last_angle_to_wall = 2.0 * M_PI;
    wall_checking_done = false;
    current_orientation.invalidate();
    // check if input ports are connected
    if (!_sonarbeam_feature.connected())
    {
        std::cerr << TaskContext::getName() << ": " 
                    << "Input port 'sonarbeam_feature' is not connected." << std::endl;
        return false;
    }
    if (!_orientation_sample.connected())
    {
        std::cerr << TaskContext::getName() << ": "
                    << "Input port 'orientation_sample' is not connected." << std::endl;
        return false;
    }

    double wall_estimation_start_angle = _wall_estimation_start_angle.get();
    double wall_estimation_end_angle = _wall_estimation_end_angle.get();
    bool bounded_input = false;
    double ransac_threshold = _wall_estimation_ransac_threshold.get();
    double ransac_min_inliers = _wall_estimation_ransac_min_inliers.get();
    if(wall_estimation_end_angle != wall_estimation_start_angle)
    {
        bounded_input = true;
    }
    if (ransac_threshold < 0.0)
    {
        std::cerr << "The ransac threshold has to be greater than 0 for the wall estimation." << std::endl;
        return false;
    }
    if (ransac_min_inliers > 1.0 || ransac_min_inliers < 0.0)
    {
        std::cerr << "The ransac minimum inliers have to be between 0 and 1 for the wall estimation," 
                     << "because this value is in percent." << std::endl;
        return false;
    }
    
    // set up wall estimation
    delete wallEstimation;
    wallEstimation = new sonar_detectors::WallEstimation();
    sonar_detectors::estimationSettings settings;
    settings.startAngle = base::Angle::fromRad(wall_estimation_start_angle);
    settings.endAngle = base::Angle::fromRad(wall_estimation_end_angle);
    settings.boundedInput = bounded_input;
    wallEstimation->setSettings(settings);
    wallEstimation->setRansacParameters(ransac_threshold, ransac_min_inliers);
    
    // set up distance estimation
    delete distanceEstimation;
    distanceEstimation = new sonar_detectors::DistanceEstimation();
    sonar_detectors::estimationSettings dist_settings;
    dist_settings.startAngle = base::Angle::fromRad(wall_estimation_start_angle);
    dist_settings.endAngle = base::Angle::fromRad(wall_estimation_end_angle);
    dist_settings.boundedInput = bounded_input;
    distanceEstimation->setSettings(dist_settings);

    return true;
}

void Task::updateHook()
{
    States actual_state = RUNNING;
    
    _orientation_sample.readNewest(current_orientation);
    
    base::samples::LaserScan feature;
    while (_sonarbeam_feature.read(feature) == RTT::NewData) 
    {
        wallEstimation->updateFeature(feature);
        distanceEstimation->updateFeature(feature);
    }
    
    base::Vector3d relativeWallPos = wallEstimation->getRelativeVirtualPoint();
    double distance_to_wall = sonar_detectors::length(relativeWallPos);//distanceEstimation->getActualDistance();
    Eigen::Vector3d relPos(0,0,0);
    base::AUVPositionCommand positionCommand;
    positionCommand.z = _fixed_depth.get();
    if (relativeWallPos.x() == 0 && relativeWallPos.y() == 0)
    {
        actual_state = SEARCHING_WALL;
        positionCommand.heading = 0;
        if (!wall_checking_done)
            checking_count = 0;
        relPos.x() = _exploration_speed.get();
        relPos.y() = 0;
    }
    else 
    {
        // calculate new relative heading
        double delta_rad = acos(relativeWallPos.x() / sqrt(pow(relativeWallPos.x(), 2) + pow(relativeWallPos.y(), 2)));
        if (relativeWallPos.y() < 0)
        {
            current_wall_angle = current_orientation.getYaw() - delta_rad;
        }
        else 
        {
            current_wall_angle = current_orientation.getYaw() + delta_rad;
        }
        // correct angles
        if (current_wall_angle > M_PI)
            current_wall_angle -= 2.0 * M_PI;
        else if (current_wall_angle < -M_PI)
            current_wall_angle += 2.0 * M_PI;
            
        // check for some samples if the wall distance seems to be stable
        if (checking_count < checking_wall_samples)
        {
            actual_state = CHECKING_WALL;
            origin_wall_angle = 2.0 * M_PI;
            if (last_distance_to_wall < 0)
            {
                last_distance_to_wall = distance_to_wall;
            }
            if (last_angle_to_wall > M_PI)
            {
                last_angle_to_wall = current_wall_angle;
            }
            
            if (last_distance_to_wall < distance_to_wall + check_distance_threshold &&
                last_distance_to_wall > distance_to_wall - check_distance_threshold && 
                last_angle_to_wall < current_wall_angle + check_angle_threshold &&
                last_angle_to_wall > current_wall_angle - check_angle_threshold)
            {
                checking_count++;
                last_distance_to_wall = distance_to_wall;
                last_angle_to_wall = current_wall_angle;
            }
            else
            {
                checking_count = 0;
                last_distance_to_wall = -1;
                last_angle_to_wall = 2.0 * M_PI;
            }
        }
        else
        {
            wall_checking_done = true;
            actual_state = WALL_FOUND;
            
            if (relativeWallPos.y() < 0)
            {
                //values outside of -PI..PI will handled by the auv_rel_pos_controller
                positionCommand.heading = _heading_modulation.get() - delta_rad;
            }
            else 
            {
                positionCommand.heading = _heading_modulation.get() + delta_rad;
            }
            
            // save inital heading
            if (origin_wall_angle > M_PI)
            {
                // save inital wall position
                origin_wall_angle = current_wall_angle;
                std::cerr << origin_wall_angle << std::endl;
            }
            else
            {
                // check for corner
                if (std::abs(origin_wall_angle - current_wall_angle) > M_PI * 0.4)
                {
                    std::cerr << "found corner" << std::endl;
                    actual_state = DETECTED_CORNER;
                }
                // check for lost wall position
                if (std::abs(origin_wall_angle - current_wall_angle) > M_PI * 0.8)
                {
                    std::cerr << "lost position" << std::endl;
                    std::cerr << "drive back " << std::endl;
                }
            }
            
            // do servoing if wall is near enough
            if (relativeWallPos.x() < 2.0 * _wall_distance.get())
            {
                relPos.y() = _servoing_speed.get();
            }
            
            // calculate new x
            // (maybe use relativeWallPos.x() for average here)
            if (distance_to_wall > 0)
            {
                relPos.x() = distance_to_wall - _wall_distance.get();
            }
            else
            {
                relPos.x() = 0;
                actual_state = DISTANCE_ESTIMATOR_TIMEOUT;
            }
        }
    }
    
    relPos = Eigen::AngleAxisd(-_heading_modulation.get(), Eigen::Vector3d::UnitZ()) * relPos;
    positionCommand.x = relPos.x();
    positionCommand.y = relPos.y();
    
    // write detection data
    sonar_detectors::wallDetectionData wallData;
    wallData.time = base::Time::now();
    const std::vector< std::pair< base::Vector3d, base::Vector3d > > walls = wallEstimation->getWalls();
    if (walls.size() >= 1)
    {
        wallData.wall.push_back(walls[0].first);
        wallData.wall.push_back(walls[0].second);
    }
    wallData.distance = distance_to_wall;
    wallData.relative_wall_position = relativeWallPos;
    std::list<base::Vector3d> feature_list = wallEstimation->getPointCloud();
    wallData.pointCloud.points.resize(feature_list.size());
    std::copy(feature_list.begin(), feature_list.end(), wallData.pointCloud.points.begin());
    wallData.pointCloud.time = base::Time::now();
    _wall_data.write(wallData);
    
    // write state if it has changed
    if(last_state != actual_state)
    {
        last_state = actual_state;
        state(actual_state);
    }
    
    // write relative position command
    if (_position_command.connected())
        _position_command.write(positionCommand);
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    if (wallEstimation)
    {
        delete wallEstimation;
        wallEstimation = 0;
    }
    if (distanceEstimation)
    {
        delete distanceEstimation;
        distanceEstimation = 0;
    }
}

