/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SingleSonarServoing.hpp"
#include <sonar_detectors/SonarDetectorTypes.hpp>
#include <SonarDetectorTaskTypes.hpp>
#include <sonar_detectors/SonarDetectorMath.hpp>
#include <base/samples/pointcloud.h>

using namespace wall_servoing;

SingleSonarServoing::SingleSonarServoing(std::string const& name, TaskCore::TaskState initial_state)
    : SingleSonarServoingBase(name, initial_state)
    , centerWallEstimation(0), mWallEstimation(0)
{
    
}

SingleSonarServoing::~SingleSonarServoing()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

// bool Task::configureHook()
// {
//     return true;
// }

bool SingleSonarServoing::startHook()
{
    last_state = PRE_OPERATIONAL;
    wall_state = NO_WALL_FOUND;
    checking_count = 0;
    exploration_checking_count = 0;
    last_distance_to_wall = -1.0;
    last_angle_to_wall.rad = 2.0 * M_PI;
    origin_wall_angle = 2.0 * M_PI;
    do_wall_servoing = false;
    wall_servoing = false;
    align_origin_position = false;
    align_origin_heading = false;
    do_heading_modulation = false;
    current_orientation.invalidate();
    wall_map.setResolution(24);
    wall_servoing_direction = 0.0;
    detected_corner_msg = false;
    start_corner_msg = base::Time::now();
    // check if input ports are connected
    if (!_sonarbeam_feature.connected())
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": " 
                    << "Input port 'sonarbeam_feature' is not connected." << RTT::endlog();
        return false;
    }
    if (!_orientation_sample.connected())
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": "
                    << "Input port 'orientation_sample' is not connected." << RTT::endlog();
        return false;
    }

    double wall_estimation_start_angle = _wall_estimation_start_angle.get();
    double wall_estimation_end_angle = _wall_estimation_end_angle.get();
    double ransac_threshold = _wall_estimation_ransac_threshold.get();
    double ransac_min_inliers = _wall_estimation_ransac_min_inliers.get();
    if (ransac_threshold < 0.0)
    {
        RTT::log(RTT::Error) << "The ransac threshold has to be greater than 0 for the wall estimation." << RTT::endlog();
        return false;
    }
    if (ransac_min_inliers > 1.0 || ransac_min_inliers < 0.0)
    {
        RTT::log(RTT::Error) << "The ransac minimum inliers have to be between 0 and 1 for the wall estimation," 
                     << "because this value is in percent." << RTT::endlog();
        return false;
    }
    
    // set up wall estimation
    delete centerWallEstimation;
    centerWallEstimation = new sonar_detectors::CenterWallEstimation();
    centerWallEstimation->setEstimationZone(base::Angle::fromRad(wall_estimation_start_angle), base::Angle::fromRad(wall_estimation_end_angle));
    centerWallEstimation->setFadingOutFactor(_fading_out_factor.get());
    
    delete mWallEstimation;
    mWallEstimation = new sonar_detectors::MWallEstimation();
    mWallEstimation->setEstimationZone(base::Angle::fromRad(wall_estimation_start_angle), base::Angle::fromRad(wall_estimation_end_angle));
    mWallEstimation->setParameters(ransac_threshold, ransac_min_inliers, _dbscan_epsilon.get(), 0.08726646259971647);

    return true;
}

void SingleSonarServoing::updateHook()
{
    States actual_state = RUNNING;
    
    // read input ports
    _orientation_sample.readNewest(current_orientation);
    
    base::samples::LaserScan feature;
    while (_sonarbeam_feature.read(feature) == RTT::NewData) 
    {
        // feed estimators
        centerWallEstimation->updateFeature(feature, base::Angle::fromRad(current_orientation.getYaw()));
        mWallEstimation->updateFeature(feature, base::Angle::fromRad(current_orientation.getYaw()));
    }
    
    // analyze wall position
    base::Vector3d wallPos = centerWallEstimation->getWall().first;
    double distance_to_wall = 0.0;
    if (wallPos.x() == 0.0 && wallPos.y() == 0.0)
    {
        wall_state = NO_WALL_FOUND;
    }
    else
    {
        distance_to_wall = sonar_detectors::length(wallPos);
        current_wall_angle = base::Angle::fromRad(atan2(wallPos.y(), wallPos.x()));
        
        if (last_distance_to_wall < 0)
        {
            last_distance_to_wall = distance_to_wall;
        }
        if(last_angle_to_wall.rad > M_PI)
        {
            last_angle_to_wall = current_wall_angle;
        }
        
        if(distance_to_wall < _minimal_wall_distance.get())
        {
            wall_state = WALL_TO_NEAR;
        }   
        else if (distance_to_wall > last_distance_to_wall + check_distance_threshold ||
                 distance_to_wall < last_distance_to_wall - check_distance_threshold)
        {
            wall_state = DISTANCE_DIFF;
        }
        else if(check_angle_threshold < std::abs(base::Angle::fromRad(last_angle_to_wall.rad - current_wall_angle.rad).getRad()) )
        {
            wall_state = ANGLE_DIFF;
        }
        else
        {
            wall_state = WALL_FOUND;
        }
    }
    
    
    if(checking_count >= checking_wall_samples)
    {
        if(do_wall_servoing)
            wall_servoing = true;
        else
            align_origin_position = true;
            
    }
    else if(checking_count <= 0)
    {
        wall_servoing = false;
        align_origin_position = false;
    }
    
    
    Eigen::Vector3d relative_target_position(0,0,0);
    base::Angle relative_target_heading = base::Angle::fromRad(0.0);
    
    if(wall_servoing)
    {
        actual_state = WALL_SERVOING;
        exploration_checking_count = 0;
        switch(wall_state)
        {
            case NO_WALL_FOUND:
                checking_count--;
                break;
            case WALL_TO_NEAR:
            case DISTANCE_DIFF:
            case ANGLE_DIFF:
                checking_count = 0;
                actual_state = LOST_WALL;
                break;
            case WALL_FOUND:
            {
                wall_map.updateAngle(current_wall_angle.getRad());
                last_distance_to_wall = distance_to_wall;
                last_angle_to_wall = current_wall_angle;
                
                // calculate ralative heading correction
                base::Angle delta_rad = current_wall_angle - base::Angle::fromRad(current_orientation.getYaw() + wall_servoing_direction);
                
                    relative_target_position.y() = _servoing_speed.get();
                    relative_target_heading = base::Angle::fromRad(_heading_modulation.get()) + delta_rad;
                    do_heading_modulation = true;
                
                // calculate new x
                relative_target_position.x() = distance_to_wall - _wall_distance.get();
                
                
                // corner detection
                if (origin_wall_angle > M_PI)
                {
                    // save inital wall angle
                    origin_wall_angle = current_wall_angle.rad;
                }
                else if(std::abs(base::Angle::fromRad(origin_wall_angle - current_wall_angle.rad).getRad()) >= M_PI * 0.45)
                {
                    // save new wall angle
                    origin_wall_angle = base::Angle::fromRad(current_wall_angle.rad + copysign(0.05 * M_PI, base::Angle::fromRad(origin_wall_angle - current_wall_angle.rad).getRad())).getRad();
                    // show detected corner msg and state
                    detected_corner_msg = true;
                    start_corner_msg = base::Time::now();
                }
                break;
            }
            default:
                std::runtime_error("received unknown wall state!");
            }
    }
    else if(align_origin_position)
    {
        actual_state = ORIGIN_ALIGNMENT;
        exploration_checking_count = 0;
        
        // align heading
        if(align_origin_heading)
        {
            base::Angle angle_diff = last_angle_to_wall - base::Angle::fromRad(current_orientation.getYaw() + _servoing_wall_direction.get());
            if(std::abs(angle_diff.rad) > 0.1)
                relative_target_heading = angle_diff;
            else
            {
                align_origin_heading = false;
                align_origin_position = false;
                do_wall_servoing = true;
            }
        }
        else
        {
            switch(wall_state)
            {
                case NO_WALL_FOUND:
                    checking_count--;
                    break;
                case WALL_TO_NEAR:
                case DISTANCE_DIFF:
                case ANGLE_DIFF:
                    checking_count = 0;
                    actual_state = LOST_WALL;
                    break;
                case WALL_FOUND:
                {
                    last_distance_to_wall = distance_to_wall;
                    last_angle_to_wall = current_wall_angle;
                    
                    // align position
                    double distance_diff = distance_to_wall - _wall_distance.get();
                    if(distance_diff > 0.5)
                        relative_target_position.x() = distance_diff;
                    else
                        align_origin_heading = true;
                    
                    break;
                }
                default:
                    std::runtime_error("received unknown wall state!"); 
            }
        }
    }
    else
    {
        actual_state = LOST_WALL;
        // switch to exploration mode after some samples
        if(exploration_checking_count < exploration_mode_samples)
        {
            exploration_checking_count++;
        }
        
        switch(wall_state)
        {
            case NO_WALL_FOUND:
                checking_count = 0;
                last_distance_to_wall = -1;
                last_angle_to_wall.rad = 2.0 * M_PI;
                break;
            case WALL_TO_NEAR:
                // drive back
                exploration_checking_count--;
                relative_target_position.x() = -_exploration_speed.get() * 2.0;
                break;
            case DISTANCE_DIFF:
            case ANGLE_DIFF:
                checking_count = 0;
                last_distance_to_wall = distance_to_wall;
                last_angle_to_wall = current_wall_angle;
                break;
            case WALL_FOUND:
                checking_count++;
                exploration_checking_count--;
                last_distance_to_wall = distance_to_wall;
                last_angle_to_wall = current_wall_angle;
                actual_state = CHECKING_WALL;
                break;
            default:
                std::runtime_error("received unknown wall state!");
        }
        
        // exploration mode
        if(exploration_checking_count >= exploration_mode_samples)
        {
            exploration_checking_count = exploration_mode_samples;
            actual_state = SEARCHING_WALL;
            if(do_wall_servoing && wall_map.isHealthy())
            {
                relative_target_heading = wall_map.getAngleForBestWall() - base::Angle::fromRad(current_orientation.getYaw() + wall_servoing_direction);
                relative_target_heading = relative_target_heading + base::Angle::fromRad(_heading_modulation.get());
                relative_target_position.x() = -_exploration_speed.get() * 2.0;
                if(std::abs(relative_target_heading.rad) <= 0.1 * M_PI)
                {
                    relative_target_position.x() = _exploration_speed.get();
                }
            }
            else
            {
                relative_target_position.x() = _exploration_speed.get();
            }
        }
    }
    
    // apply wall servoing direction
    if(do_wall_servoing)
    relative_target_position = Eigen::AngleAxisd(wall_servoing_direction, Eigen::Vector3d::UnitZ()) * relative_target_position;

    // apply heading modulation
    if(do_heading_modulation)
        relative_target_position = Eigen::AngleAxisd(-_heading_modulation.get(), Eigen::Vector3d::UnitZ()) * relative_target_position;
    
     // create relative position command
    base::AUVPositionCommand positionCommand;
    positionCommand.x = relative_target_position.x();
    positionCommand.y = relative_target_position.y();
    positionCommand.z = _fixed_depth.get();
    positionCommand.heading = relative_target_heading.getRad();
    
    // print detected corner msg for 2 seconds
    if(detected_corner_msg)
    {
        RTT::log(RTT::Info) << "found corner" << RTT::endlog();
        actual_state = DETECTED_CORNER;
        if((base::Time::now() - start_corner_msg).toSeconds() > 2.0)
        {
            detected_corner_msg = false;
        }
    }

    // write state if it has changed
    if(last_state != actual_state)
    {
        last_state = actual_state;
        state(actual_state);
    }
    
    // write relative position command
    if (_position_command.connected())
        _position_command.write(positionCommand);
    
    // write detection debug data
    if(_enable_debug_output.get())
    {
        sonar_detectors::wallServoingDebugData debugData;
        debugData.time = base::Time::now();
        std::pair< base::Vector3d, base::Vector3d > wall = centerWallEstimation->getWall();
        debugData.wall.push_back(wall.first);
        debugData.wall.push_back(wall.second);
        debugData.wall_distance = distance_to_wall;
        debugData.wall_angle = current_wall_angle.getRad();
        debugData.relative_wall_position = current_orientation.orientation.conjugate() * wallPos;
        debugData.pointCloud.points = centerWallEstimation->getPointCloud();
        debugData.pointCloud.time = base::Time::now();
        
        _wall_servoing_debug.write(debugData);
    }
}

void SingleSonarServoing::errorHook()
{
    SingleSonarServoingBase::errorHook();
}

void SingleSonarServoing::stopHook()
{
    SingleSonarServoingBase::stopHook();
}

void SingleSonarServoing::cleanupHook()
{
    if (mWallEstimation)
    {
        delete mWallEstimation;
        mWallEstimation = 0;
    }
    if (centerWallEstimation)
    {
        delete centerWallEstimation;
        centerWallEstimation = 0;
    }
}

