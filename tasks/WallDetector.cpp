/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WallDetector.hpp"
#include <sonar_detectors/SonarDetectorTypes.hpp>
#include <SonarDetectorTaskTypes.hpp>
#include <sonar_detectors/SonarDetectorMath.hpp>
#include <base/samples/pointcloud.h>

using namespace wall_servoing;

WallDetector::WallDetector(std::string const& name, TaskCore::TaskState initial_state)
    : WallDetectorBase(name, initial_state)
{
}

WallDetector::WallDetector(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : WallDetectorBase(name, engine, initial_state)
{
}

WallDetector::~WallDetector()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See WallDetector.hpp for more detailed
// documentation about them.

bool WallDetector::configureHook()
{
    if (! WallDetectorBase::configureHook())
        return false;
    return true;
}
bool WallDetector::startHook()
{
    if (! WallDetectorBase::startHook())
        return false;
    
    // check if input ports are connected
    if (!_sonarbeam_feature.connected())
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": " 
                    << "Input port 'sonarbeam_feature' is not connected." << RTT::endlog();
        return false;
    }
    
    if (!_orientation_sample.connected())
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": "
                    << "Input port 'orientation_sample' is not connected." << RTT::endlog();
        return false;
    }
    
    if(!_position_sample.connected() && _use_motion_model.get()){
	  RTT::log(RTT::Error) << TaskContext::getName() << ": "
		    << "Input port 'positon_sample' is not connected." << RTT::endlog();
    }
    
   
  
    // set up wall estimation
    //delete centerWallEstimation;
    centerWallEstimation = new sonar_detectors::CenterWallEstimation();
    centerWallEstimation->setFadingOutFactor(_fading_out_factor.get());
    centerWallEstimation->setMinScanPoints(4);
    last_feature_in_range = false; 
    return true;
}
void WallDetector::updateHook()
{
    WallDetectorBase::updateHook();

    // read input ports
    _orientation_sample.readNewest(current_orientation);
    _position_sample.readNewest(current_position);

    base::Angle left_limit = base::Angle::fromRad(_wall_direction.get() + _opening_angle.get()); 
    base::Angle right_limit = base::Angle::fromRad(_wall_direction.get() - _opening_angle.get()); 
    base::Angle opening = base::Angle::fromRad(_opening_angle.get());

    base::samples::LaserScan feature;
    while (_sonarbeam_feature.read(feature) == RTT::NewData){ 
        centerWallEstimation->setEstimationZone(left_limit, right_limit); 
        centerWallEstimation->setWallAngleVariance(opening.getRad() * 0.3);
        centerWallEstimation->setSupposedWallAngle(base::Angle::fromRad(_wall_direction.get()));
        
        //Calculate offset to handle the change over PI -> -PI
        double offset = M_PI - left_limit.getRad();
        if((base::Angle::normalizeRad(feature.start_angle + offset) <= base::Angle::normalizeRad(left_limit.getRad() + offset)) &&
           (base::Angle::normalizeRad(feature.start_angle + offset) >= base::Angle::normalizeRad(right_limit.getRad() + offset))){
            centerWallEstimation->updateFeature(feature, base::Angle::fromRad(current_orientation.getYaw()));
            last_feature_in_range = true;
        } else{
            if(last_feature_in_range){
                std::pair<base::Vector3d, base::Vector3d> wall = centerWallEstimation->getWall();
                detected_distance = sonar_detectors::length(wall.first);
                detected_orientation = atan2(wall.second.y(), wall.second.x());
                detected_position = current_position;
                last_wall_estimation = base::Time::now();
                if(state() != WALL_FOUND){
                    state(WALL_FOUND);
                }
            }
            
            last_feature_in_range = false;
        }
    }
    
    sonar_detectors::Wall wall_out;
    wall_out.last_detection = (base::Time::now()-last_wall_estimation).toSeconds(); 

    if(wall_out.last_detection>_wall_estimation_timeout.get()){
        detected_distance = base::unset<double>();
        detected_orientation = base::unset<double>();
        if(state() != WALL_SEARCHING){
            state(WALL_SEARCHING);
        }
    }

    wall_out.wall_angle = detected_orientation;
    
    if(_use_motion_model){
        wall_out.wall_distance = detected_distance + ((-cos(detected_orientation) * (current_position.position.y() - detected_position.position.y())) + (sin(detected_orientation) * (current_position.position.x() - detected_position.position.x())));
    } else{
        wall_out.wall_distance = detected_distance;
    }

    _wall.write(wall_out);

}
void WallDetector::errorHook()
{
    WallDetectorBase::errorHook();
}
void WallDetector::stopHook()
{
    WallDetectorBase::stopHook();
}
void WallDetector::cleanupHook()
{
    WallDetectorBase::cleanupHook();
}
