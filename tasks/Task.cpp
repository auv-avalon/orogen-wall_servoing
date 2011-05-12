/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

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

bool Task::configureHook()
{
    return true;
}
bool Task::startHook()
{
    processing = new avalon::SonarBeamProcessing(avalon::globalMaximum, avalon::persistNewScans);
    double beam_threshold_min = _beam_threshold_min.get();
    double beam_threshold_max = _beam_threshold_max.get();
    double min_response_value = _min_response_value.get();
    
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
    
    processing->setBeamThreshold(beam_threshold_min, beam_threshold_max);
    processing->enableBeamThreshold(_enable_beam_threshold.get());
    processing->enableWallEstimation(_enable_wall_estimation.get());
    processing->setMinResponseValue((unsigned int)min_response_value);
    return true;
}
void Task::updateHook()
{
    base::samples::SonarScan sonarScan;
    while (_sonar_input.connected() && _sonar_input.read(sonarScan) == RTT::NewData) 
    {
        processing->updateSonarData(sonarScan);
    }
    base::samples::RigidBodyState bodyState;
    if (_body_state.connected() && _body_state.readNewest(bodyState) == RTT::NewData) 
    {
        processing->updatePosition(bodyState.position);
        processing->updateOrientation(bodyState.orientation);
    }
    base::samples::LaserScan laserScan;
    if (_ground_distance_input.connected() && _ground_distance_input.read(laserScan) == RTT::NewData) 
    {
        
    }
    
    base::Vector3d vec = processing->getVirtualPoint();
    if (!(vec.x() == 0 && vec.y() == 0 && vec.z() == 0))
    {
        _virtual_point.write(vec);
    }
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
}

