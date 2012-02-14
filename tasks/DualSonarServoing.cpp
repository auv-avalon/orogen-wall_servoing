/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DualSonarServoing.hpp"

using namespace wall_servoing;

DualSonarServoing::DualSonarServoing(std::string const& name, TaskCore::TaskState initial_state)
    : DualSonarServoingBase(name, initial_state)
{
}

DualSonarServoing::DualSonarServoing(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : DualSonarServoingBase(name, engine, initial_state)
{
}

DualSonarServoing::~DualSonarServoing()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DualSonarServoing.hpp for more detailed
// documentation about them.

// bool DualSonarServoing::configureHook()
// {
//     if (! DualSonarServoingBase::configureHook())
//         return false;
//     return true;
// }
// bool DualSonarServoing::startHook()
// {
//     if (! DualSonarServoingBase::startHook())
//         return false;
//     return true;
// }
// void DualSonarServoing::updateHook()
// {
//     DualSonarServoingBase::updateHook();
// }
// void DualSonarServoing::errorHook()
// {
//     DualSonarServoingBase::errorHook();
// }
// void DualSonarServoing::stopHook()
// {
//     DualSonarServoingBase::stopHook();
// }
// void DualSonarServoing::cleanupHook()
// {
//     DualSonarServoingBase::cleanupHook();
// }

