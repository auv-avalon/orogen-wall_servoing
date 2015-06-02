/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WallServoing.hpp"
#include <base/commands/AUVMotion.hpp>

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
}
void WallServoing::updateHook()
{
    WallServoingBase::updateHook();

    if(start){ 
        start = false;
        state(GET_INITIAL_WALL_SEARCH_DIRECTION);
    }

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

    double direction;
    double servoing_speed;
    double correction_speed;
    double max_servoing_speed = _servoing_speed.get();
    double max_correction_speed = _correction_speed.get();
    double direction_offset = _servoing_direction.get();
    bool direction_clockwise = _direction_clockwise.get();

    switch(state()){
        case GET_INITIAL_WALL_SEARCH_DIRECTION:
            {
            direction = _search_direction.get();
            servoing_speed = 0;
            correction_speed = 0;

            double offset_to_search_angle = fabs(base::Angle::normalizeRad(base::getYaw(orientation_sample.orientation) -_search_direction.get() - direction_offset));
            //std::cout << "OFFSET_TO_SEARCH: " << offset_to_search_angle << std::endl;

            if(offset_to_search_angle < 0.02){
                obstacle_detections = 0;
                last_obstacle_detection = obstacle_wall.last_detection;
                state(LEAVING_GET_INITIAL_WALL_SEARCH_DIRECTION);
            }
            }
            break;

        case LEAVING_GET_INITIAL_WALL_SEARCH_DIRECTION: 
            direction = _search_direction.get();
            servoing_speed = 0;
            correction_speed = 0;

            if(obstacle_wall.last_detection < last_obstacle_detection){
                obstacle_detections++;
            }
            last_obstacle_detection = obstacle_wall.last_detection;

            if(obstacle_detections >= 2){
                state(INITIAL_WALL_SEARCH);
            }
            break;


        case INITIAL_WALL_SEARCH:
            direction = _search_direction.get();
            servoing_speed = this->limitSpeed((obstacle_wall.wall_distance - _servoing_distance.get()) * _servoing_factor.get(), max_servoing_speed);
            correction_speed = 0;

            if(fabs(obstacle_wall.wall_distance - _servoing_distance.get()) < 0.2){
                obstacle_angle = this->directionCorrection(obstacle_wall.wall_angle, direction_clockwise);
                state(CORNER);
            }
            break;
        case CORNER:
            {
            direction = obstacle_angle;
            servoing_speed = 0;
            correction_speed = 0;
            
            double offset_to_obstacle_angle = fabs(base::getYaw(orientation_sample.orientation) - obstacle_angle - direction_offset);
            if(offset_to_obstacle_angle > M_PI){
                offset_to_obstacle_angle = 2*M_PI - offset_to_obstacle_angle;
            }

//          std::cout << "OFFSET_TO_OBSTACLE_ANGLE: " << offset_to_obstacle_angle   << std::endl;
            
            if(offset_to_obstacle_angle < 0.02){
                servoing_detections = 0;
                obstacle_detections = 0;
                last_servoing_detection = servoing_wall.last_detection;
                last_obstacle_detection = obstacle_wall.last_detection;
                state(LEAVING_CORNER);
            }
            }
            break;

        case LEAVING_CORNER: 
            direction = obstacle_angle;
            servoing_speed = 0;
            correction_speed = 0;

            if(obstacle_wall.last_detection < last_obstacle_detection){
                obstacle_detections++;
            }
            if(servoing_wall.last_detection < last_servoing_detection){
                servoing_detections++;
            }
            last_servoing_detection = servoing_wall.last_detection;
            last_obstacle_detection = obstacle_wall.last_detection;

            if(obstacle_detections >= 2 && servoing_detections >=2){
                state(WALL_SERVOING);
            }
            break;
        case WALL_SERVOING:
            direction = this->directionCorrection(servoing_wall.wall_angle, direction_clockwise);
            servoing_speed = this->limitSpeed((obstacle_wall.wall_distance - _servoing_distance.get()) * _servoing_factor.get(), max_servoing_speed);
            correction_speed = this->limitSpeed((servoing_wall.wall_distance - _servoing_distance.get()) * _correction_factor.get(), max_correction_speed);
            if(!direction_clockwise){
                correction_speed = correction_speed * -1;
            }

            if(fabs(obstacle_wall.wall_distance - _servoing_distance.get()) < 0.2){
                obstacle_angle = this->directionCorrection(obstacle_wall.wall_angle, direction_clockwise);
                state(CORNER);
            }
            break;
    }

    base::LinearAngular6DCommand world_cmd;
    world_cmd.time = base::Time::now();
    world_cmd.z() = _servoing_depth;
    world_cmd.roll() = 0;
    world_cmd.pitch() = 0;
    world_cmd.yaw() = base::Angle::normalizeRad(direction + direction_offset); 
    base::LinearAngular6DCommand aligned_velocity_cmd;
    aligned_velocity_cmd.time = base::Time::now();
    aligned_velocity_cmd.x() = (cos(_servoing_direction.get())*servoing_speed) + (sin(_servoing_direction.get())*correction_speed); 
    aligned_velocity_cmd.y() = -(sin(_servoing_direction.get())*servoing_speed) + (cos(_servoing_direction.get())*correction_speed); 
    //aligned_velocity_cmd.y() = (cos(_servoing_direction.get())*correction_speed); 

    _world_command.write(world_cmd);
    _aligned_velocity_command.write(aligned_velocity_cmd);

    //For the od controlchain

    base::commands::AUVMotion motion_command;
    motion_command.x_speed = aligned_velocity_cmd.x();
    motion_command.y_speed = aligned_velocity_cmd.y();
    motion_command.z = world_cmd.z();
    motion_command.heading = world_cmd.yaw();

    _motion_command.write(motion_command);
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

double WallServoing::limitSpeed(double speed, double limit){
    if(speed > limit){
        return limit;
    } else if(speed < -limit){
        return -limit;
    }
    return speed;
    
}

double WallServoing::directionCorrection(double direction, bool clockwise){
    if(clockwise){
        return direction;
    }
    return base::Angle::normalizeRad(direction + M_PI);
}
