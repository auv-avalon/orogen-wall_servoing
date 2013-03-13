/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DualSonarServoing.hpp"
#include "SonarDetectorTaskTypes.hpp"

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

bool DualSonarServoing::isInRange(const double& angle, const double& left_limit, const double& right_limit) const
{
    bool range_switch = false;
    if(right_limit - left_limit >= 0)
        range_switch = true;
    if((!range_switch && (angle <= left_limit && angle >= right_limit)) ||
        (range_switch && (angle <= left_limit || angle >= right_limit)))
        return true;
    else
        return false;
}

void DualSonarServoing::receiveDistanceFromFeature(const base::samples::LaserScan& feature, double& distance) const
{
    try
    {
        if(feature.isValidBeam(0))
        {
            distance = (double)feature.ranges.front() * 0.001;
        }
        else
        {
            distance = base::unknown<double>();
        }
    }
    catch (std::out_of_range e)
    {
        RTT::log(RTT::Info) << "Received empty laser scan." << RTT::endlog();
        distance = base::unknown<double>();
    }
}

void ObstacleDistance::addNewDistanceSample(double distance)
{
    if(distance_collector.size() > 10)
    {
        distance_collector.erase(distance_collector.begin());
    }
    distance_collector.push_back(distance);
    last_distance = 0.0;
    for(unsigned i = 0; i < distance_collector.size(); i++)
    {
        last_distance += distance_collector[i];
    }
    last_distance /= (double)distance_collector.size();
    time_last_distance = base::Time::now();
    dirty = true;
}

void ObstacleDistance::checkForTimeout(double seconds)
{
    if(!base::isUnknown(last_distance) && (base::Time::now() - time_last_distance).toSeconds() >= seconds)
    {
        last_distance = base::unknown<double>();
        dirty = false;
    }
}

bool ObstacleDistance::isValid()
{
    return !base::isUnknown(last_distance);
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
bool DualSonarServoing::startHook()
{
    if (!DualSonarServoingBase::startHook())
        return false;
    
    if (!_sonarbeam_feature_front.connected())
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": " 
                    << "Input port 'sonarbeam_feature_front' is not connected." << RTT::endlog();
        return false;
    }
    if (!_sonarbeam_feature_rear.connected())
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": " 
                    << "Input port 'sonarbeam_feature_rear' is not connected." << RTT::endlog();
        return false;
    }
    if (!_orientation_sample.connected())
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": "
                    << "Input port 'orientation_sample' is not connected." << RTT::endlog();
        return false;
    }
    
    double half_opening_angle =  _opening_angle.get() * 0.5;
    if (half_opening_angle <= 0.0)
    {
        RTT::log(RTT::Error) << "The opening angle has to be greater than 0." << RTT::endlog();
        return false;
    }
    front_sonar_front_left_angle = 0.0 + half_opening_angle;
    front_sonar_front_right_angle = 0.0 - half_opening_angle;
    front_sonar_right_left_angle = -M_PI_2 + half_opening_angle;
    front_sonar_right_right_angle = -M_PI_2 - half_opening_angle;
    rear_sonar_left_angle = 0.0 + half_opening_angle;
    rear_sonar_right_angle = 0.0 - half_opening_angle;
    
    sonar_span_x = 1.1715;
    sonar_span_y = 0.05;
    
    distance_sample_timeout = 3.0; // seconds
    
    no_sonar_features_timeout = 5.0; //seconds
    last_valid_feature_front_front = base::Time::now();
    last_valid_feature_front_right = base::Time::now();
    last_valid_feature_rear_right = base::Time::now();
    
    last_target_heading = base::unknown<double>();
    handle_corner = false;
    
    return true;
}
void DualSonarServoing::updateHook()
{
    States actual_state = RUNNING;
    
    // read input ports
    _orientation_sample.readNewest(current_orientation);
    
    base::samples::LaserScan feature;
    while(_sonarbeam_feature_front.read(feature) == RTT::NewData) 
    {
        if(isInRange(feature.start_angle, front_sonar_front_left_angle, front_sonar_front_right_angle))
        {
            last_valid_feature_front_front = base::Time::now();
            double distance = base::unknown<double>();
            receiveDistanceFromFeature(feature, distance);
            if(!base::isUnknown(distance))
            {
                wall_front_dist.addNewDistanceSample(distance);
            }
        }
        else if(isInRange(feature.start_angle, front_sonar_right_left_angle, front_sonar_right_right_angle))
        {
            last_valid_feature_front_right = base::Time::now();
            double distance = base::unknown<double>();
            receiveDistanceFromFeature(feature, distance);
            if(!base::isUnknown(distance))
            {
                wall_right_front_dist.addNewDistanceSample(distance);
            }
        }
        else
        {
            wall_right_front_dist.distance_collector.clear();
            wall_front_dist.distance_collector.clear();
            wall_front_dist.dirty = false;
            wall_right_front_dist.dirty = false;
        }
    }
    while(_sonarbeam_feature_rear.read(feature) == RTT::NewData) 
    {
        if(isInRange(feature.start_angle, rear_sonar_left_angle, rear_sonar_right_angle))
        {
            last_valid_feature_rear_right = base::Time::now();
            double distance = base::unknown<double>();
            receiveDistanceFromFeature(feature, distance);
            if(!base::isUnknown(distance))
            {
                wall_right_rear_dist.addNewDistanceSample(distance);
            }
        }
        else
        {
            wall_right_rear_dist.distance_collector.clear();
            wall_right_rear_dist.dirty = false;
        }
    }
    
    base::AUVPositionCommand positionCommand;
    positionCommand.z = _fixed_depth.get();
    positionCommand.x = 0.0;
    positionCommand.y = 0.0;
    positionCommand.heading = 0.0;
        
    if(handle_corner)
    {
        actual_state = DETECTED_CORNER;
        if(base::isUnknown(last_target_heading))
        {
            last_target_heading = base::Angle::normalizeRad(current_orientation.getYaw() + M_PI_2);
        }
        positionCommand.heading = base::Angle::normalizeRad(last_target_heading - current_orientation.getYaw());
        if(std::abs(last_target_heading - current_orientation.getYaw()) < 0.1)
        {
            handle_corner = false;
            last_target_heading = base::unknown<double>();
            wall_right_front_dist.distance_collector.clear();
            wall_front_dist.distance_collector.clear();
            wall_front_dist.dirty = false;
            wall_right_front_dist.dirty = false;
            wall_right_rear_dist.distance_collector.clear();
            wall_right_rear_dist.dirty = false;
        }
    }
    else
    {        
        // check for timeouts
        wall_front_dist.checkForTimeout(distance_sample_timeout);
        wall_right_front_dist.checkForTimeout(distance_sample_timeout);
        wall_right_rear_dist.checkForTimeout(distance_sample_timeout);
        
        // check wall distance to the right side
        if(wall_right_rear_dist.isValid())
        {
            actual_state = WALL_SERVOING;
            if(wall_right_rear_dist.dirty)
                positionCommand.y = _wall_distance.get() - wall_right_rear_dist.last_distance;
            else
                positionCommand.y = 0.0;
            
            // do servoing if right wall is near enough
            if(positionCommand.y > -0.5)
            {
                // save last target heading in global coordinates
                if(wall_right_front_dist.dirty && wall_right_front_dist.isValid() && wall_right_rear_dist.isValid())
                {   
                    last_target_heading = base::Angle::normalizeRad(current_orientation.getYaw() + atan2((wall_right_rear_dist.last_distance + sonar_span_y) - wall_right_front_dist.last_distance, sonar_span_x));
                }
                
                // do heading correction
                if(base::isUnknown(last_target_heading))
                    positionCommand.heading = 0.0;
                else
                    positionCommand.heading = base::Angle::normalizeRad(last_target_heading - current_orientation.getYaw());
                
                // check wall distance to the front
                if(wall_front_dist.dirty && wall_front_dist.isValid() && wall_front_dist.last_distance < _wall_distance.get() + 0.5)
                {
                    positionCommand.x = 0.0;
                    handle_corner = true;
                    last_target_heading = base::unknown<double>();
                }
                else
                    positionCommand.x = _servoing_speed.get();
            }
        }
        else
        {
            actual_state = SEARCHING_WALL;
            positionCommand.y = _exploration_speed.get();
        }
    } 
    
    // check for feature timeout
    if((base::Time::now() - last_valid_feature_front_front).toSeconds() > no_sonar_features_timeout
        || (base::Time::now() - last_valid_feature_front_right).toSeconds() > no_sonar_features_timeout
        || (base::Time::now() - last_valid_feature_rear_right).toSeconds() > no_sonar_features_timeout)
    {
        actual_state = MISCONFIGURATION;
    }
       
    // write state if it has changed
    if(last_state != actual_state)
    {
        last_state = actual_state;
        state(actual_state);
    }
    
    // write debug output
    if(_enable_debug_output.get() && _wall_servoing_debug.connected())
    {
        sonar_detectors::DualWallServoingDebugData debugData;
        debugData.wall_distance_front = wall_front_dist.last_distance;
        debugData.wall_distance_right = wall_right_rear_dist.last_distance;
        if(!base::isUnknown(last_target_heading))
            debugData.wall_angle = base::Angle::normalizeRad(last_target_heading - M_PI_2);
        _wall_servoing_debug.write(debugData);
    }
    
    // write relative position command
    if (_position_command.connected())
        _position_command.write(positionCommand);

    //create and wite alignied command
    if (_aligned_command.connected()){
        base::LinearAngular6DCommand alignedCommand;

        alignedCommand.linear(0) = positionCommand.x;
        alignedCommand.linear(1) = positionCommand.y;
        alignedCommand.linear(2) = positionCommand.z;
        alignedCommand.angular(1) = 0.0;
        alignedCommand.angular(2) = positionCommand.heading;

        _aligned_command.write(alignedCommand);
    }
}
void DualSonarServoing::errorHook()
{
    DualSonarServoingBase::errorHook();
}
void DualSonarServoing::stopHook()
{
    DualSonarServoingBase::stopHook();
}
void DualSonarServoing::cleanupHook()
{
    DualSonarServoingBase::cleanupHook();
}

