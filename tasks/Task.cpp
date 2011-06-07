/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <sonar_detectors/SonarDetectorTypes.hpp>

using namespace sonardetector;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
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
    validBodyState = false;
    // check if input ports are connected
    if (!_sonar_input.connected())
    {
        std::cerr << TaskContext::getName() << ": " 
                    << "Input port 'sonar_input' is not connected." << std::endl;
        return false;
    }
    if (!_body_state.connected())
    {
        std::cerr << TaskContext::getName() << ": "
                    << "Input port 'body_state' is not connected." << std::endl;
        return false;
    }
    
    // check property values
    processing = new avalon::SonarBeamProcessing(avalon::globalMaximum, avalon::persistNewScans);
    double beam_threshold_min = _beam_threshold_min.get();
    double beam_threshold_max = _beam_threshold_max.get();
    double min_response_value = _min_response_value.get();
    double wall_estimation_start_angle = _wall_estimation_start_angle.get();
    double wall_estimation_end_angle = _wall_estimation_end_angle.get();
    
    if (beam_threshold_min < 0 || beam_threshold_max < 0 || beam_threshold_min >= beam_threshold_max)
    {
        std::cerr << "The sonar beam thresholds shouldn't be smaller then 0 and the "
                     << "maximum threshold should be greater than the minimum one." << std::endl;
        return false;
    }
    if (min_response_value < 0) 
    {
        std::cerr << "The minimum response value has to be greater than 0!" << std::endl;
        return false;
    }
    if (wall_estimation_start_angle < M_PI_2 || wall_estimation_start_angle > M_PI + M_PI_2 ||
        wall_estimation_end_angle < M_PI_2 || wall_estimation_end_angle > M_PI + M_PI_2)
    {
        std::cerr << "The wall estimation angles have to be between 1/2 PI and 3/2 PI for this task." << std::endl;
        return false;
    }
    
    processing->setBeamThreshold(beam_threshold_min, beam_threshold_max);
    processing->enableBeamThreshold(_enable_beam_threshold.get());
    processing->setMinResponseValue(min_response_value);
    
    wallEstimation = new avalon::WallEstimation();
    avalon::estimationSettings settings;
    settings.segMode = avalon::forEachEdge;
    settings.startAngle = wall_estimation_start_angle;
    settings.endAngle = wall_estimation_end_angle;
    wallEstimation->setSettings(settings);
    processing->addSonarEstimation(wallEstimation);
    
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
    base::samples::SonarScan sonarScan;
    while (_sonar_input.read(sonarScan) == RTT::NewData) 
    {
        processing->updateSonarData(sonarScan);
    }
    base::samples::RigidBodyState bodyState;
    if (_body_state.readNewest(bodyState) == RTT::NewData) 
    {
        validBodyState = true;
        processing->updatePosition(bodyState.position);
        processing->updateOrientation(bodyState.orientation);
        actualBodyState = bodyState;
    }
    
    if(!validBodyState) 
    {
        std::cerr << TaskContext::getName() << ": " 
            << "Waiting for a valid RigidBodyState." << std::endl;
        return;
    }
    
    base::Vector3d relativeWallPos = wallEstimation->getRelativeVirtualPoint();
    base::AUVPositionCommand positionCommand;
    positionCommand.z = _fixed_depth.get();
    if (relativeWallPos.x() == 0 && relativeWallPos.y() == 0)
    {
        positionCommand.heading = 0;
        positionCommand.x = 0;
        positionCommand.y = 0;
    }
    else 
    {
        // calculate new heading
        double heading = base::getYaw(actualBodyState.orientation);
        double delta_rad = acos(relativeWallPos.x() / sqrt(pow(relativeWallPos.x(), 2) + pow(relativeWallPos.y(), 2)));
        if (relativeWallPos.y() < 0)
        {
            //values outside of -PI..PI will handled by the auv_rel_pos_controller
            positionCommand.heading = heading - delta_rad;
        }
        else 
        {
            positionCommand.heading = heading + delta_rad;
        }
        
        // calculate new x
        double distance_to_wall = distanceEstimation->getActualDistance();
        // (maybe use relativeWallPos.x() for average here)
        if (distance_to_wall > 0)
            positionCommand.x = distance_to_wall - _wall_distance.get();
        else
            positionCommand.x = 0;
        
        positionCommand.y = 0;
    }
    
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
        delete processing;
    if (wallEstimation)
        delete wallEstimation;
    if (distanceEstimation)
        delete wallEstimation;
}

