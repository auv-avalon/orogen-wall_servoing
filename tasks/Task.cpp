/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace sonarvizkit;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
    app.start();
    vizkitWidget = app.getWidget();
    sonarPlugin = new vizkit::SonarBeamVisualization();
    vizkitWidget->addDataHandler(sonarPlugin);
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    base::samples::RigidBodyState state;
    state.invalidate();
    state.position(2,0) = 2;
    state.cov_position = Eigen::Matrix3d::Identity();
    sonarPlugin->updateData(state);
    vizkitWidget->changeCameraView(osg::Vec3d(0,0,2), osg::Vec3d(-5,0,4));
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    base::samples::SonarScan sonarScan;
    if (_sonar_input.connected() && _sonar_input.read(sonarScan) == RTT::NewData) 
    {
        sonarPlugin->updateData(sonarScan);
    }
    base::samples::RigidBodyState bodyState;
    if (_body_state.connected() && _body_state.read(bodyState) == RTT::NewData) 
    {
        sonarPlugin->updateData(bodyState);
    }
    base::samples::LaserScan laserScan;
    if (_ground_distance_input.connected() && _ground_distance_input.read(laserScan) == RTT::NewData) 
    {
        
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
    TaskBase::cleanupHook();
}

