/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SingleSonarServoing.hpp"
#include <sonar_detectors/SonarDetectorTypes.hpp>
#include <SonarDetectorTaskTypes.hpp>
#include <sonar_detectors/SonarDetectorMath.hpp>
#include <base/samples/pointcloud.h>

using namespace wall_servoing;

SingleSonarServoing::SingleSonarServoing(std::string const& name, TaskCore::TaskState initial_state)
    : SingleSonarServoingBase(name, initial_state)
    , centerWallEstimation(0), frontWallEstimation(0)
{
    
}

SingleSonarServoing::~SingleSonarServoing()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

// bool Task::configureHook()
// {
//     return true;
// }

bool SingleSonarServoing::startHook()
{
    last_state = PRE_OPERATIONAL;
    wall_state = NO_WALL_FOUND;
    checking_count = 0;
    exploration_checking_count = 0;
    last_distance_to_wall = -1.0;
    last_angle_to_wall.rad = 2.0 * M_PI;
    origin_wall_angle = 2.0 * M_PI;
    alignment_heading.rad = 2.0 * M_PI;
    do_wall_servoing = false;
    wall_servoing = false;
    align_origin_position = false;
    align_origin_heading = false;
    current_orientation.invalidate();
    current_orientation.invalidateOrientation();
    wall_map.setResolution(24);
    detected_corner_msg = false;
    start_corner_msg = base::Time::now();
    alignment_complete_msg = false;
    start_alignment_complete_msg = base::Time::now();
    last_valid_feature_left = base::Time::now();
    last_valid_feature_right = base::Time::now();
    no_sonar_features_timeout = 10.0; //seconds
    inital_wait = true;
    sonar_direction=true;
    check_distance_threshold = _check_distance_threshold.get();
    front_distance = 999;
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
    delete centerWallEstimation;
    centerWallEstimation = new sonar_detectors::CenterWallEstimation();
    centerWallEstimation->setFadingOutFactor(_fading_out_factor.get());
    centerWallEstimation->setMinScanPoints(4);
    
    delete frontWallEstimation;
    frontWallEstimation = new sonar_detectors::CenterWallEstimation();
    frontWallEstimation->setFadingOutFactor(_fading_out_factor.get());
    frontWallEstimation->setMinScanPoints(2);


    return true;
}

void SingleSonarServoing::updateHook()
{
    States actual_state = RUNNING;
    
    // read input ports
    _orientation_sample.readNewest(current_orientation);
    
    // set actual angular limits
    double supposed_wall_direction = do_wall_servoing ? _servoing_wall_direction.get() : _initial_wall_direction.get();
    base::Angle left_limit = base::Angle::fromRad(supposed_wall_direction + _left_opening_angle.get());
    base::Angle right_limit = base::Angle::fromRad(supposed_wall_direction - _right_opening_angle.get());
    centerWallEstimation->setEstimationZone(left_limit, right_limit);
    centerWallEstimation->setWallAngleVariance(_servoing_speed.get() <= 0.0 ? _left_opening_angle.get() * 0.3 : _right_opening_angle.get() * -0.3);
    centerWallEstimation->setSupposedWallAngle(base::Angle::fromRad(supposed_wall_direction));

    double distance_to_wall = last_distance_to_wall;
    
    base::Angle left_front_limit = base::Angle::fromRad(supposed_wall_direction - 0.5 * M_PI + _left_front_angle.get());
    base::Angle right_front_limit = base::Angle::fromRad(supposed_wall_direction - 0.5 * M_PI -  _right_front_angle.get());
    frontWallEstimation->setEstimationZone(left_front_limit, right_front_limit); 
    frontWallEstimation->setWallAngleVariance(0.0);
    frontWallEstimation->setSupposedWallAngle(base::Angle::fromRad(supposed_wall_direction-0.5*M_PI));
    
    bool position_sample = false; //true, when the last recieved sample was a position-sample | false, when it was a laser-scan
    base::samples::RigidBodyState rbs;
    
    //Read position samples
    while(_position_sample.read(rbs) == RTT::NewData){
	base::Vector3d actPosition = rbs.position;
	
	//Calculate the relative position of avalon to the last known position
	double relX = cos(current_orientation.getYaw()) * (actPosition.x() - last_known_position.x()) 
			- sin(current_orientation.getYaw()) * (actPosition.y()-last_known_position.y());
	double relY = cos(current_orientation.getYaw()) * (actPosition.y() - last_known_position.y())
			+ sin(current_orientation.getYaw()) * (actPosition.x() - last_known_position.x());
	
	//Calculate the distance to wall by using the change in position
	distance_to_wall = last_known_distance - cos(current_orientation.getYaw()) * relX + sin(current_orientation.getYaw()) * relY;		
	last_distance_to_wall = distance_to_wall;
	last_position = actPosition;
	
	position_sample = true;
    }
    
    base::samples::LaserScan feature;
    while (_sonarbeam_feature.read(feature) == RTT::NewData) 
    {
        // check if feature is in range
        if(sonar_detectors::isInAngularRange(base::Angle::fromRad(feature.start_angle), left_limit, base::Angle::fromRad(supposed_wall_direction)))
            last_valid_feature_left = base::Time::now();
        else if(sonar_detectors::isInAngularRange(base::Angle::fromRad(feature.start_angle), base::Angle::fromRad(supposed_wall_direction), right_limit))
            last_valid_feature_right = base::Time::now();
                
        // feed estimators
        centerWallEstimation->updateFeature(feature, base::Angle::fromRad(current_orientation.getYaw()));
	
	
	if(feature.start_angle > right_front_limit.rad && feature.start_angle < left_front_limit.rad){
	  frontWallEstimation->updateFeature(feature, base::Angle::fromRad(current_orientation.getYaw()));
	  
	  std::pair<base::Vector3d, base::Vector3d> wall = frontWallEstimation->getWall();
	  base::Vector3d frontWall = wall.first;
	  front_distance = frontWall(0) == 0 && frontWall(1) == 0 ? 999 : sonar_detectors::length(frontWall);
	  std::cout << "Front distance " << front_distance << std::endl;
	  std::cout << "Front wall " << frontWall.transpose() << " - " << wall.second.transpose() << std::endl;
	
	} 
	
	//Detect a change in the sonar-scan direction
	if(((feature.start_angle - last_feature_bearing) > 0.0 && sonar_direction) 
	    || ((feature.start_angle - last_feature_bearing) < 0.0 && !sonar_direction)){
	
	    base::Vector3d wallPos = centerWallEstimation->getWall().first;
	    distance_to_wall = sonar_detectors::length(wallPos);	    
	
	   //save position and distance data
	   last_known_position = last_position;
	   last_known_distance = distance_to_wall;
	    
	    if(sonar_direction)
	      sonar_direction = false;
	    else
	      sonar_direction = true;
	}   
	
	last_feature_bearing = feature.start_angle;
	position_sample = false;
    }
    
    // wait inital some seconds befor the wall-servoing starts
    if(inital_wait)
    {
        if(started_task.microseconds == 0.0)
            started_task = base::Time::now();
        if(!current_orientation.hasValidOrientation())
        {
            RTT::log(RTT::Info) << "waiting for valid orientation" << RTT::endlog();
            return;
        }
        else if((base::Time::now() - started_task).toSeconds() - _wait_until_start > 0.0)
        {
            inital_wait = false;
            alignment_heading.rad = M_PI * 2.0;
        }
        else
        {
            if(alignment_heading.rad > M_PI)
                alignment_heading = base::Angle::fromRad(current_orientation.getYaw());
            
            base::AUVPositionCommand positionCommand;
            positionCommand.x = 0.0;
            positionCommand.y = 0.0;
            positionCommand.z = _fixed_depth.get();
            positionCommand.heading = (alignment_heading - base::Angle::fromRad(current_orientation.getYaw())).getRad();
                    
            // write relative position command
            if (_position_command.connected())
                _position_command.write(positionCommand);
           
            //create commnd for aligned 
            base::LinearAngular6DCommand alignedCommand;
	    base::LinearAngular6DCommand worldCommand;
            alignedCommand.linear(0) = 0.0;
            alignedCommand.linear(1) = 0.0;
            worldCommand.linear(2) = _fixed_depth.get();
            worldCommand.angular(1) = 0.0;
            alignedCommand.angular(2) = (alignment_heading - base::Angle::fromRad(current_orientation.getYaw())).getRad();

            // write aligned position command
            if (_aligned_command.connected())
                _aligned_command.write(alignedCommand);
	    
	     if (_world_command.connected())
                _world_command.write(worldCommand);

            return;
        }
    }
    
    // analyze wall position
    base::Vector3d wallPos = centerWallEstimation->getWall().first;    
    if (wallPos.x() == 0.0 && wallPos.y() == 0.0)
    {
        wall_state = NO_WALL_FOUND;
    }
    else
    {	
	  //if the motion model is not used, update the wall-distance by using sonar-data
	 if((!_use_motion_model.get() && !position_sample) || !wall_servoing){	   
	      distance_to_wall = sonar_detectors::length(wallPos);	    
	 }
	 current_wall_angle = base::Angle::fromRad(atan2(wallPos.y(), wallPos.x()));
        
        if (last_distance_to_wall < 0)
        {
            last_distance_to_wall = distance_to_wall;
        }
        if(last_angle_to_wall.rad > M_PI)
        {
            last_angle_to_wall = current_wall_angle;
        }
        
        if(distance_to_wall < _minimal_wall_distance.get())
        {
            wall_state = WALL_TO_NEAR;
        }   
        else if (distance_to_wall > last_distance_to_wall + check_distance_threshold ||
                 distance_to_wall < last_distance_to_wall - check_distance_threshold)
        {
            wall_state = DISTANCE_DIFF;
        }
        else if(check_angle_threshold < std::abs(base::Angle::fromRad(last_angle_to_wall.rad - current_wall_angle.rad).getRad()) )
        {
            wall_state = ANGLE_DIFF;
        }
        else
        {
            wall_state = WALL_FOUND;
        }
    }
    
    
    if(checking_count >= checking_wall_samples)
    {
        if(do_wall_servoing)
            wall_servoing = true;
        else
            align_origin_position = true;
            
    }
    else if(checking_count <= 0)
    {	
	std::cout << "Lost wall at checking_count : "<< std::endl;
        wall_servoing = false;
        align_origin_position = false;
    }
    
    
    Eigen::Vector3d relative_target_position(0.0,0.0,0.0);
    base::Angle relative_target_heading = base::Angle::fromRad(0.0);
    
    if(wall_servoing)
    {
        actual_state = WALL_SERVOING;
        exploration_checking_count = 0;
        switch(wall_state)
        {
            case NO_WALL_FOUND:
                checking_count--;
                break;
            case WALL_TO_NEAR:
            case DISTANCE_DIFF:
            case ANGLE_DIFF:
                checking_count = 0;
                actual_state = LOST_WALL;		
                break;
            case WALL_FOUND:
            {
                wall_map.updateAngle(current_wall_angle.getRad());
                last_distance_to_wall = distance_to_wall;
                last_angle_to_wall = current_wall_angle;
                
                // do servoing
                relative_target_position.y() = _servoing_speed.get();
		
		if( fabs(relative_target_position.y()) > front_distance - _wall_distance && _use_front_distance.get()){
		    relative_target_position.y() =  _wall_distance - front_distance;
		     std::cout << "Reduce Speed to: " << relative_target_position.y() << std::endl;
		 } 
		
                // calculate ralative heading correction
                relative_target_heading = current_wall_angle - base::Angle::fromRad(current_orientation.getYaw() + _servoing_wall_direction.get());
                // calculate new x
                relative_target_position.x() = distance_to_wall - _wall_distance.get();
                
                // corner detection
                if (origin_wall_angle > M_PI)
                {
                    // save inital wall angle
                    origin_wall_angle = current_wall_angle.rad;
                }
                else if(std::abs(base::Angle::fromRad(origin_wall_angle - current_wall_angle.rad).getRad()) >= M_PI * 0.40)
                {
                    // save new wall angle
                    origin_wall_angle = base::Angle::fromRad(current_wall_angle.rad + copysign(0.10 * M_PI, base::Angle::fromRad(origin_wall_angle - current_wall_angle.rad).getRad())).getRad();
                    // show detected corner msg and state
                    detected_corner_msg = true;
                    start_corner_msg = base::Time::now();
                }
                break;
            }
            default:
                std::runtime_error("received unknown wall state!");
            }
    }
    else if(align_origin_position)
    {
        actual_state = ORIGIN_ALIGNMENT;
        exploration_checking_count = 0;
        
        // align heading
        if(align_origin_heading)
        {
            base::Angle angle_diff = last_angle_to_wall - base::Angle::fromRad(current_orientation.getYaw() + _servoing_wall_direction.get());
            if(std::abs(angle_diff.rad) > 0.1)
                relative_target_heading = angle_diff;
            else
            {
                align_origin_heading = false;
                align_origin_position = false;
                do_wall_servoing = true;
                // show alignment complete state
                alignment_complete_msg = true;
                start_alignment_complete_msg = base::Time::now();
                alignment_heading.rad = 2 * M_PI;
            }
        }
        else
        {
            if(alignment_heading.rad > M_PI)
                alignment_heading = base::Angle::fromRad(current_orientation.getYaw());
            switch(wall_state)
            {
                case NO_WALL_FOUND:
                    checking_count--;
                    break;
                case WALL_TO_NEAR:
                case DISTANCE_DIFF:
                case ANGLE_DIFF:
                    checking_count = 0;
                    actual_state = LOST_WALL;
		     std::cout << "Lost wall at align_origin_position : " << wall_state << std::endl;
                    break;
                case WALL_FOUND:
                {
                    last_distance_to_wall = distance_to_wall;
                    last_angle_to_wall = current_wall_angle;
                    
                    // align position
                    double distance_diff = distance_to_wall - _wall_distance.get();
                    if(distance_diff > 0.5)
                    {
                        relative_target_heading = alignment_heading - base::Angle::fromRad(current_orientation.getYaw());
                        relative_target_position.x() = distance_diff;
                    }
                    else
                        align_origin_heading = true;
                    
                    break;
                }
                default:
                    std::runtime_error("received unknown wall state!"); 
            }
        }
    }
    else
    {	
	 
        actual_state = LOST_WALL;
        // switch to exploration mode after some samples
        if(exploration_checking_count < exploration_mode_samples)
        {
            exploration_checking_count++;
        }
        
        switch(wall_state)
        {
            case NO_WALL_FOUND:
                checking_count = 0;
                last_distance_to_wall = -1;
                last_angle_to_wall.rad = 2.0 * M_PI;
                break;
            case WALL_TO_NEAR:
                // drive back
                exploration_checking_count--;
                relative_target_position.x() = -_exploration_speed.get() * 2.0;
                break;
            case DISTANCE_DIFF:
            case ANGLE_DIFF:
                checking_count = 0;
                last_distance_to_wall = distance_to_wall;
                last_angle_to_wall = current_wall_angle;
                break;
            case WALL_FOUND:
                checking_count++;
                exploration_checking_count--;
                last_distance_to_wall = distance_to_wall;
                last_angle_to_wall = current_wall_angle;
                actual_state = CHECKING_WALL;
                break;
            default:
                std::runtime_error("received unknown wall state!");
        }
        
        // exploration mode
        if(exploration_checking_count >= exploration_mode_samples)
        {
            exploration_checking_count = exploration_mode_samples;
            actual_state = SEARCHING_WALL;
            if(do_wall_servoing && wall_map.isHealthy())
            {
                relative_target_heading = wall_map.getAngleForBestWall() - base::Angle::fromRad(current_orientation.getYaw() + _servoing_wall_direction.get());
                relative_target_position.x() = -_exploration_speed.get() * 2.0;
                if(std::abs(relative_target_heading.rad) <= 0.1 * M_PI)
                {
                    relative_target_position.x() = _exploration_speed.get();
                }
            }
            else
            {
                relative_target_position.x() = _exploration_speed.get();
            }
            
      
        }
    }
    
    // apply wall servoing direction
    if(do_wall_servoing)
        relative_target_position = Eigen::AngleAxisd(_servoing_wall_direction.get(), Eigen::Vector3d::UnitZ()) * relative_target_position;
    else
        relative_target_position = Eigen::AngleAxisd(_initial_wall_direction.get(), Eigen::Vector3d::UnitZ()) * relative_target_position;
    
     // create relative position command
    base::AUVPositionCommand positionCommand;
    positionCommand.x = std::abs(relative_target_position.x()) < 0.001 ? 0.0 : relative_target_position.x();
    positionCommand.y = std::abs(relative_target_position.y()) < 0.001 ? 0.0 : relative_target_position.y();
    positionCommand.z = _fixed_depth.get();
    positionCommand.heading = relative_target_heading.getRad();
    
    //create commnd for aligned 
    base::LinearAngular6DCommand alignedCommand;
    base::LinearAngular6DCommand worldCommand;
    alignedCommand.linear(0) = std::abs(relative_target_position.x()) < 0.001 ? 0.0 : relative_target_position.x();
    alignedCommand.linear(1) = std::abs(relative_target_position.y()) < 0.001 ? 0.0 : relative_target_position.y();
    worldCommand.linear(2) = _fixed_depth.get();
    worldCommand.angular(1) = 0.0;
    alignedCommand.angular(2) = relative_target_heading.getRad();
 
    // print detected corner msg for 2 seconds
    if(detected_corner_msg)
    {
        RTT::log(RTT::Info) << "found corner" << RTT::endlog();
        actual_state = DETECTED_CORNER;
        if((base::Time::now() - start_corner_msg).toSeconds() > 2.0)
        {
            detected_corner_msg = false;
        }
    }
    // print alignment complete msg for 2 seconds
    if(alignment_complete_msg)
    {
        actual_state = ALIGNMENT_COMPLETE;
        if((base::Time::now() - start_alignment_complete_msg).toSeconds() > 2.0)
        {
            alignment_complete_msg = false;
        }
    }
    
    // check for feature timeout
    if((base::Time::now() - last_valid_feature_left).toSeconds() > no_sonar_features_timeout
        || (base::Time::now() - last_valid_feature_right).toSeconds() > no_sonar_features_timeout)
    {
        actual_state = MISCONFIGURATION;
    }

    // write state if it has changed
    if(last_state != actual_state)
    {
        last_state = actual_state;
        state(actual_state);
    }
    
    // write relative position command
    if (_position_command.connected())
        _position_command.write(positionCommand);
    
    // write aligned position command
    if (_aligned_command.connected())
        _aligned_command.write(alignedCommand);
    
    if (_world_command.connected())
        _world_command.write(worldCommand);
    
    // write detection debug data
    if(_enable_debug_output.get())
    {
        sonar_detectors::WallServoingDebugData debugData;
        debugData.time = base::Time::now();
        std::pair< base::Vector3d, base::Vector3d > wall = centerWallEstimation->getWall();
        debugData.wall.push_back(wall.first);
        debugData.wall.push_back(wall.second);
        debugData.wall_distance = distance_to_wall;
        debugData.wall_angle = current_wall_angle.getRad();
        debugData.relative_wall_position = current_orientation.orientation.conjugate() * wallPos;
        debugData.pointCloud.points = centerWallEstimation->getPointCloud();
        debugData.pointCloud.time = base::Time::now();
        
        _wall_servoing_debug.write(debugData);
    }
}

void SingleSonarServoing::errorHook()
{
    SingleSonarServoingBase::errorHook();
}

void SingleSonarServoing::stopHook()
{
    SingleSonarServoingBase::stopHook();
}

void SingleSonarServoing::cleanupHook()
{

    if (centerWallEstimation)
    {
        delete centerWallEstimation;
	delete frontWallEstimation;
        centerWallEstimation = 0;
    }
}

