/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace sonardetector;

Task::Task(std::string const& name)
    : TaskBase(name)
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
    if(_use_visualization.get())
    {
        use_visualization = true;
        //app.start();
        //vizkitWidget = app.getWidget();
        //sonarPlugin = new vizkit::SonarBeamVisualization();
        //vizkitWidget->addDataHandler(sonarPlugin);
        //vizkitWidget->changeCameraView(osg::Vec3d(0,0,0), osg::Vec3d(-5,0,2));
    }
    else
    {
        use_visualization = false;
        processing = new avalon::SonarBeamProcessing(avalon::globalMaximum, avalon::persistNewScans);
        processing->setBeamThreshold(0.5, 5);
        processing->enableBeamThreshold(true);
        processing->enableWallEstimation(true);
        processing->setMinResponseValue(10);
    }
    
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
    while (_sonar_input.connected() && _sonar_input.read(sonarScan) == RTT::NewData) 
    {
        if(use_visualization);
            //sonarPlugin->updateData(sonarScan);
        else
            processing->updateSonarData(sonarScan);
    }
    base::samples::RigidBodyState bodyState;
    if (_body_state.connected() && _body_state.readNewest(bodyState) == RTT::NewData) 
    {
        if(use_visualization);
            //sonarPlugin->updateData(bodyState);
        else
        {
            processing->updatePosition(bodyState.position);
            processing->updateOrientation(bodyState.orientation);
        }
    }
    base::samples::LaserScan laserScan;
    if (_ground_distance_input.connected() && _ground_distance_input.read(laserScan) == RTT::NewData) 
    {
        
    }
    
    base::Vector3d vec;
    if (use_visualization);
        //vec = sonarPlugin->getVirtualPoint();
    else
        vec = processing->getVirtualPoint();
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
    //if (sonarPlugin)
        //delete sonarPlugin;
    if (processing)
        delete processing;
}

