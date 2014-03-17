/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WallServoing.hpp"

using namespace wall_servoing;

WallServoing::WallServoing(std::string const& name, TaskCore::TaskState initial_state)
    : WallServoingBase(name, initial_state)
{
}

WallServoing::WallServoing(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : WallServoingBase(name, engine, initial_state)
{
}

WallServoing::~WallServoing()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See WallServoing.hpp for more detailed
// documentation about them.

bool WallServoing::configureHook()
{
    if (! WallServoingBase::configureHook())
        return false;
    return true;
}
bool WallServoing::startHook()
{
    if (! WallServoingBase::startHook())
        return false;

    start = true;

    return true;
}
void WallServoing::updateHook()
{
    WallServoingBase::updateHook();

    if(start){
        state(INITIAL_WALL_SEARCH);
        start = false;
    }

    base::LinearAngular6DCommand world_cmd;
    world_cmd.z() = _servoing_depth;
    world_cmd.roll() = 0;
    world_cmd.pitch() = 0;
    base::LinearAngular6DCommand aligned_velocity_cmd;

    base::samples::RigidBodyState orientation_sample;

    sonar_detectors::Wall obstacle_wall;
    sonar_detectors::Wall servoing_wall;

    if(_obstacle_wall.readNewest(obstacle_wall) == RTT::NoData){
        return;
    }
    
    if(_servoing_wall.readNewest(servoing_wall) == RTT::NoData){
        return;
    }
    
    if(_orientation_sample.readNewest(orientation_sample) == RTT::NoData){
        return;
    }

    switch(state()){
        case INITIAL_WALL_SEARCH:
            world_cmd.yaw() = _search_direction.get();
            aligned_velocity_cmd.x() = (obstacle_wall.wall_distance - _servoing_distance.get()) * _servoing_factor.get();
            if(aligned_velocity_cmd.x() > _servoing_speed.get()){
                aligned_velocity_cmd.x() = _servoing_speed.get();
            }
            aligned_velocity_cmd.y() = 0;

            if(fabs(obstacle_wall.wall_distance - _servoing_distance.get()) < 0.2){
                obstacle_angle = obstacle_wall.wall_angle;
                state(CORNER);
            }
            break;
        case CORNER:
            world_cmd.yaw() = obstacle_angle;
            aligned_velocity_cmd.x() = 0;
            aligned_velocity_cmd.y() = 0;

            if(fabs(base::getYaw(orientation_sample.orientation) - obstacle_angle) < 0.1 &&
                    obstacle_wall.last_detection > 0.5){
                state(WALL_SERVOING);
            }
            break;
        case WALL_SERVOING:
            world_cmd.yaw() = servoing_wall.wall_angle;
            aligned_velocity_cmd.x() = (obstacle_wall.wall_distance - _servoing_distance.get()) * _servoing_factor.get();
            if(aligned_velocity_cmd.x() > _servoing_speed.get()){    
               aligned_velocity_cmd.x() = _servoing_speed.get();
            }
            aligned_velocity_cmd.y() = (servoing_wall.wall_distance - _servoing_distance.get()) * _correction_factor.get();
            
            if(fabs(obstacle_wall.wall_distance - _servoing_distance.get()) < 0.2){
                obstacle_angle = obstacle_wall.wall_angle;
                state(CORNER);
            }
            break;
    }

    _world_command.write(world_cmd);
    _aligned_velocity_command.write(aligned_velocity_cmd);

}
void WallServoing::errorHook()
{
    WallServoingBase::errorHook();
}
void WallServoing::stopHook()
{
    WallServoingBase::stopHook();
}
void WallServoing::cleanupHook()
{
    WallServoingBase::cleanupHook();
}
