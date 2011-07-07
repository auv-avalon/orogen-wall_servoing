/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <sonar_detectors/SonarDetectorTypes.hpp>
#include <SonarDetectorTaskTypes.hpp>

using namespace sonardetector;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
    , processing(0), wallEstimation(0), distanceEstimation(0)
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
    // check if input ports are connected
    if (!_sonar_input.connected())
    {
        std::cerr << TaskContext::getName() << ": " 
                    << "Input port 'sonar_input' is not connected." << std::endl;
        return false;
    }
    if (!_orientation_sample.connected())
    {
        std::cerr << TaskContext::getName() << ": "
                    << "Input port 'orientation_sample' is not connected." << std::endl;
        return false;
    }
    
    // set up sonar beam processing
    delete processing;
    processing = new avalon::SonarBeamProcessing(avalon::globalMaximum, avalon::persistNewScans);
    double beam_threshold_min = _beam_threshold_min.get();
    double beam_threshold_max = _beam_threshold_max.get();
    double min_response_value = _min_response_value.get();
    double wall_estimation_start_angle = _wall_estimation_start_angle.get();
    double wall_estimation_end_angle = _wall_estimation_end_angle.get();
    double ransac_threshold = _wall_estimation_ransac_threshold.get();
    double ransac_min_inliers = _wall_estimation_ransac_min_inliers.get();
    
    if (beam_threshold_min < 0.0 || beam_threshold_max < 0.0 || beam_threshold_min >= beam_threshold_max)
    {
        std::cerr << "The sonar beam thresholds shouldn't be smaller then 0 and the "
                     << "maximum threshold should be greater than the minimum one." << std::endl;
        return false;
    }
    if (min_response_value < 0.0) 
    {
        std::cerr << "The minimum response value has to be greater than 0!" << std::endl;
        return false;
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
    
    processing->setBeamThreshold(beam_threshold_min, beam_threshold_max);
    processing->enableBeamThreshold(_enable_beam_threshold.get());
    processing->setMinResponseValue(min_response_value);
    
    // set up wall estimation
    delete wallEstimation;
    wallEstimation = new avalon::WallEstimation();
    avalon::estimationSettings settings;
    settings.segMode = avalon::forEachBeam;
    settings.startAngle = wall_estimation_start_angle;
    settings.endAngle = wall_estimation_end_angle;
    wallEstimation->setSettings(settings);
    wallEstimation->setRansacParameters(ransac_threshold, ransac_min_inliers);
    processing->addSonarEstimation(wallEstimation);
    
    // set up distance estimation
    delete distanceEstimation;
    distanceEstimation = new avalon::DistanceEstimation();
    avalon::estimationSettings dist_settings;
    dist_settings.segMode = avalon::forEachBeam;
    dist_settings.startAngle = wall_estimation_start_angle;
    dist_settings.endAngle = wall_estimation_end_angle;
    distanceEstimation->setSettings(dist_settings);
    processing->addSonarEstimation(distanceEstimation);

    return true;
}

void Task::updateHook()
{
    States actual_state = RUNNING;
    
    base::samples::SonarScan sonarScan;
    while (_sonar_input.read(sonarScan) == RTT::NewData) 
    {
        processing->updateSonarData(sonarScan);
    }
    base::samples::RigidBodyState orientation;
    if (_orientation_sample.readNewest(orientation) == RTT::NewData) 
    {
        processing->updatePosition(base::Position(0,0,0));
        processing->updateOrientation(orientation.orientation);
    }
    
    base::Vector3d relativeWallPos = wallEstimation->getRelativeVirtualPoint();
    double distance_to_wall = distanceEstimation->getActualDistance();
    Eigen::Vector3d relPos(0,0,0);
    base::AUVPositionCommand positionCommand;
    positionCommand.z = _fixed_depth.get();
    if (relativeWallPos.x() == 0 && relativeWallPos.y() == 0)
    {
        actual_state = SEARCHING_WALL;
        
        positionCommand.heading = 0;
        relPos.x() = _exploration_speed.get();
        relPos.y() = 0;
    }
    else 
    {
        actual_state = WALL_FOUND;
        // calculate new relative heading
        double delta_rad = acos(relativeWallPos.x() / sqrt(pow(relativeWallPos.x(), 2) + pow(relativeWallPos.y(), 2)));
        if (relativeWallPos.y() < 0)
        {
            //values outside of -PI..PI will handled by the auv_rel_pos_controller
            positionCommand.heading = _heading_modulation.get() - delta_rad;
        }
        else 
        {
            positionCommand.heading = _heading_modulation.get() + delta_rad;
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
            relPos.x() = ((distance_to_wall + relativeWallPos.x()) * 0.5) - _wall_distance.get();
        }
        else
        {
            relPos.x() = 0;
            actual_state = DISTANCE_ESTIMATOR_TIMEOUT;
        }
    }
    
    relPos = Eigen::AngleAxisd(-_heading_modulation.get(), Eigen::Vector3d::UnitZ()) * relPos;
    positionCommand.x = relPos.x();
    positionCommand.y = relPos.y();
    
    // write detection data
    avalon::wallDetectionData wallData;
    wallData.time = base::Time::now();
    const std::vector< std::pair< base::Vector3d, base::Vector3d > > walls = wallEstimation->getWalls();
    if (walls.size() >= 1)
    {
        wallData.pose_vector = walls[0].first;
        wallData.direction_vector = walls[0].second;
    }
    wallData.distance = distance_to_wall;
    wallData.relative_wall_position = relativeWallPos;
    wallData.pointCloud = wallEstimation->getPointCloud();
    // _wall_data.write(wallData);
    
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
    if (processing)
    {
        delete processing;
        processing = 0;
    }
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

