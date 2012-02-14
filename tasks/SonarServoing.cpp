/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SonarServoing.hpp"

using namespace wall_servoing;

SonarServoing::SonarServoing(std::string const& name, TaskCore::TaskState initial_state)
    : SonarServoingBase(name, initial_state)
{
}

SonarServoing::SonarServoing(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : SonarServoingBase(name, engine, initial_state)
{
}

SonarServoing::~SonarServoing()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SonarServoing.hpp for more detailed
// documentation about them.

// bool SonarServoing::configureHook()
// {
//     if (! SonarServoingBase::configureHook())
//         return false;
//     return true;
// }
// bool SonarServoing::startHook()
// {
//     if (! SonarServoingBase::startHook())
//         return false;
//     return true;
// }
// void SonarServoing::updateHook()
// {
//     SonarServoingBase::updateHook();
// }
// void SonarServoing::errorHook()
// {
//     SonarServoingBase::errorHook();
// }
// void SonarServoing::stopHook()
// {
//     SonarServoingBase::stopHook();
// }
// void SonarServoing::cleanupHook()
// {
//     SonarServoingBase::cleanupHook();
// }

