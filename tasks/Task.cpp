/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace asv_localization;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::gps_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_samples_sample)
{
   
  if(firstPositionRecieved){
 
    
    if(firstOrientationRecieved){
      base::Vector3d pos; 
      
      for(int i = 0; i < 3; i++)
	  pos[1] = gps_samples_sample.position[i] - firstGpsSample.position[i];
      
      base::Matrix3d covariance = base::Matrix3d::Identity() * _gps_error.get();
      
      if(ekf.positionObservation( gps_samples_sample.position, covariance, _gps_reject_threshold.get() )){
	std::cout << "Rejected GPS-Sample" << std::endl;
      }else{
	
	double relX = cos(base::getYaw(ekf.getRotation())) * (gps_samples_sample.position[0] - lastGpsSample.position[0])
			- sin(base::getYaw(ekf.getRotation())) * (gps_samples_sample.position[1] - lastGpsSample.position[1]);
			
	double relY = sin(base::getYaw(ekf.getRotation())) * (gps_samples_sample.position[0] - lastGpsSample.position[0])		
			+ cos(base::getYaw(ekf.getRotation())) * (gps_samples_sample.position[1] - lastGpsSample.position[1]);
			
	base::Vector3d vel;
	vel[0] = relX / (ts.toSeconds() - lastGpsTime.toSeconds());
	vel[1] = relY / (ts.toSeconds() - lastGpsTime.toSeconds());
	vel[2] = 0;
	
	base::Matrix3d covar = base::Matrix3d::Identity() * _velocity_error.get() ;
	
	
	if(ekf.velocityObservation(vel, covar, _velocity_reject_threshold.get()))
	  std::cout << "Rejected Velocity" << std::endl;
	  
	lastGpsSample = gps_samples_sample;
      }
      
    }    
    
  }else{
    
    base::Matrix3d covariance = base::Matrix3d::Identity() * _gps_error.get();
    base::Vector3d pos;
    pos << 0,0,0;
    
    ekf.setPosition(pos , covariance);
    
    lastGpsSample = gps_samples_sample;
    firstGpsSample = gps_samples_sample;
    firstPositionRecieved = true;
  }
  
  
  lastGpsTime = ts;
  
}
void Task::imu_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample)
{
  if(!lastImuTime.isNull() && firstOrientationRecieved){
    double dt = ts.toSeconds() - lastImuTime.toSeconds();
    
    Eigen::Matrix<double, 9, 9 >  process_noise = Eigen::Matrix<double, 9, 9>::Zero();
    process_noise(0,0) = _acceleration_error.get();
    process_noise(1,1) = _acceleration_error.get();
    process_noise(2,2) = _acceleration_error.get();
    
    process_noise(3,3) = _acceleration_error.get() * dt;
    process_noise(4,4) = _acceleration_error.get() * dt;
    process_noise(5,5) = _acceleration_error.get() * dt;
    
    process_noise(6,6) = _acceleration_error.get() * dt * dt;
    process_noise(7,7) = _acceleration_error.get() * dt * dt;
    process_noise(8,8) = _acceleration_error.get() * dt * dt;
    
    ekf.predict( imu_samples_sample.acc, dt, process_noise);
      
  }
  
  lastImuTime = ts;
  
}
void Task::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{    
  
  firstOrientationRecieved = true;
  
  ekf.setRotation(orientation_samples_sample.orientation);
  
  base::samples::RigidBodyState rbs;
  rbs.position = ekf.getPosition();
  rbs.velocity = ekf.getVelocity();
  rbs.cov_position = ekf.getPositionCovariance();
  rbs.cov_velocity = ekf.getVelocityCovariance();
  _pose_samples.write(rbs);
  
  
}


void Task::velocity_samplesCallback(const:: base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample){
  
  if(!lastVelocityTime.isNull()){
    double dt = ts.toSeconds() - lastVelocityTime.toSeconds();
    
    base::Vector3d acc;
    acc[0] = (velocity_samples_sample.velocity[0] - lastVelocitySample.velocity[0]) / dt;
    acc[1] = (velocity_samples_sample.velocity[1] - lastVelocitySample.velocity[1]) / dt;
    acc[2] = (velocity_samples_sample.velocity[2] - lastVelocitySample.velocity[2]) / dt;
    
    
    Eigen::Matrix<double, 9, 9 >  process_noise = Eigen::Matrix<double, 9, 9>::Zero();
    process_noise(0,0) = _acceleration_error.get();
    process_noise(1,1) = _acceleration_error.get();
    process_noise(2,2) = _acceleration_error.get();
    
    process_noise(3,3) = _acceleration_error.get() * dt;
    process_noise(4,4) = _acceleration_error.get() * dt;
    process_noise(5,5) = _acceleration_error.get() * dt;
    
    process_noise(6,6) = _acceleration_error.get() * dt * dt;
    process_noise(7,7) = _acceleration_error.get() * dt * dt;
    process_noise(8,8) = _acceleration_error.get() * dt * dt;
    
    ekf.predict( acc, dt, process_noise);
    
    
  }
  
  lastVelocityTime = ts;
  lastVelocitySample = velocity_samples_sample;
  
}  


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.




bool Task::configureHook()
{
    
    if (! RTT::TaskContext::configureHook())
        return false;
    
    firstPositionRecieved = false;
    firstOrientationRecieved = false;
    lastImuTime = base::Time();
    lastGpsTime = base::Time();
    lastVelocityTime = base::Time();
    
    return true;
    
}



bool Task::startHook()
{
    
    if (! RTT::TaskContext::startHook())
        return false;
    

    ekf = pose_ekf::KFD_PosVelAcc();

    
    return true;
    
}



void Task::updateHook()
{
    
    RTT::TaskContext::updateHook();
    

    

    
}



void Task::errorHook()
{
    
    RTT::TaskContext::errorHook();
    

    

    
}



void Task::stopHook()
{
    
    RTT::TaskContext::stopHook();
    

    

    
}



void Task::cleanupHook()
{
    
    RTT::TaskContext::cleanupHook();
    

    

    
}

