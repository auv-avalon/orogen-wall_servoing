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
 //std::cout << "GPS callback" << std::endl;
  
  if(firstPositionRecieved){
     
    if(firstOrientationRecieved){
      base::Vector3d pos; 
      
      for(int i = 0; i < 3; i++)
	  pos[i] = gps_samples_sample.position[i] - firstGpsSample.position[i];
      
      base::Matrix3d covariance = base::Matrix3d::Identity() * _gps_error.get();
      
      if(ekf.positionObservation( gps_samples_sample.position, covariance, _gps_reject_threshold.get() )){
	std::cout << "Rejected GPS-Sample" << std::endl;
      }else if(_estimate_velocity.get() ){
	
	samplesCount++;
		
	if(samplesCount >= _velocity_estimation_count.get() ){
	  
	  //Calculate the relative movement between the samples
	  double relX = cos(base::getYaw(ekf.getRotation())) * (gps_samples_sample.position[0] - lastGpsSample.position[0])
			  - sin(base::getYaw(ekf.getRotation())) * (gps_samples_sample.position[1] - lastGpsSample.position[1]);
			  
	  double relY = sin(base::getYaw(ekf.getRotation())) * (gps_samples_sample.position[0] - lastGpsSample.position[0])		
			  + cos(base::getYaw(ekf.getRotation())) * (gps_samples_sample.position[1] - lastGpsSample.position[1]);
	  
	  //Calculate the velocity from movement and time-delay		  
	  base::Vector3d vel;
	  vel[0] = relX / (ts.toSeconds() - lastGpsSample.time.toSeconds());
	  vel[1] = relY / (ts.toSeconds() - lastGpsSample.time.toSeconds());
	  vel[2] = 0;
	  
	  base::Matrix3d covar = base::Matrix3d::Identity() * _velocity_error.get() ;
	  
	  //Observation
	  if(ekf.velocityObservation(vel, covar, _velocity_reject_threshold.get()))
	    std::cout << "Rejected Velocity" << std::endl;
	    
	  lastGpsSample = gps_samples_sample;
	  samplesCount = 0;
      } 
      }
      
    }    
    
  }else{ //First gps-sample
    
    base::Matrix3d covariance = base::Matrix3d::Identity() * _gps_error.get();
    
    lastGpsSample = gps_samples_sample;
    firstGpsSample = gps_samples_sample;
    firstPositionRecieved = true;
    
    //Use 0,0,0 as origin
    if(!_initial_gps_origin.get())
      firstGpsSample.position = base::Vector3d::Zero();
    
    ekf.setPosition( firstGpsSample.position , covariance);

    std::cout << "Initialize Position";
  }
  
  
  lastGpsTime = ts;
  
}

void Task::imu_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample)
{
  if(!lastImuTime.isNull() && firstOrientationRecieved){
    double dt = ts.toSeconds() - lastImuTime.toSeconds();
    
    //Covariance
    Eigen::Matrix<double, 9, 9 >  process_noise = Eigen::Matrix<double, 9, 9>::Zero();
    //Acceleration noise
    process_noise(0,0) = _acceleration_error.get();
    process_noise(1,1) = _acceleration_error.get();
    process_noise(2,2) = _acceleration_error.get();
    //Velocity noise: sum up covariance 
    process_noise(3,3) = _acceleration_error.get() * dt;
    process_noise(4,4) = _acceleration_error.get() * dt;
    process_noise(5,5) = _acceleration_error.get() * dt;
    //Position noise: sum up velocity covariancee
    process_noise(6,6) = _acceleration_error.get() * dt * dt;
    process_noise(7,7) = _acceleration_error.get() * dt * dt;
    process_noise(8,8) = _acceleration_error.get() * dt * dt;
    
    ekf.predict( imu_samples_sample.acc, dt, process_noise);
      
  }
  
  lastImuTime = ts;
  
}

void Task::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{    
  //std::cout << "Orientation callback" << std::endl;
  firstOrientationRecieved = true;
  
  ekf.setRotation(orientation_samples_sample.orientation);
  
  //Write out actual state
  base::samples::RigidBodyState rbs;
  rbs.position = ekf.getPosition();
  rbs.velocity = ekf.getVelocity();
  rbs.cov_position = ekf.getPositionCovariance();
  rbs.cov_velocity = ekf.getVelocityCovariance();
  _pose_samples.write(rbs);
  
  
}


void Task::velocity_samplesCallback(const:: base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample){
  //std::cout << "Velocity callback" << std::endl;
  /*if(!lastVelocityTime.isNull()){
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
    
    
  }*/
  
  base::Matrix3d cov = base::Matrix3d::Identity() * _velocity_error.get();
  
  if( !ekf.velocityObservation( velocity_samples_sample.velocity, cov, _velocity_reject_threshold.get()))
    std::cout << "Rejected velocity" << std::endl;
  
  
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
    samplesCount = 0;
    
    strAligner.setTimeout( base::Time::fromSeconds(_max_delay.get()));
    const double buffer_size_factor = 2.0;
    
    //Register data streams at the stream aligner
    if( _gps_samples.connected()){
      
      gpsID = strAligner.registerStream<base::samples::RigidBodyState>(
	    boost::bind( &asv_localization::Task::gps_samplesCallback, this, _1, _2 ),
	    buffer_size_factor * std::ceil( _max_delay.get() / _gps_period.get() ),
	    base::Time::fromSeconds( _gps_period.get() ) );      
    }
    
    
    if( _orientation_samples.connected()){
      
      orientationID = strAligner.registerStream<base::samples::RigidBodyState>(
	    boost::bind( &asv_localization::Task::orientation_samplesCallback, this, _1, _2 ),
	    buffer_size_factor * std::ceil( _max_delay.get() / _ori_period.get() ),
	    base::Time::fromSeconds( _ori_period.get() ) );      
    }
    
    if( _velocity_samples.connected()){
      
      velocityID = strAligner.registerStream<base::samples::RigidBodyState>(
	    boost::bind( &asv_localization::Task::velocity_samplesCallback, this, _1, _2 ),
	    buffer_size_factor * std::ceil( _max_delay.get() / _vel_period.get() ),
	    base::Time::fromSeconds( _vel_period.get() ) );      
    }
    
    if( _imu_samples.connected()){
      
      imuID = strAligner.registerStream<base::samples::IMUSensors>(
	    boost::bind( &asv_localization::Task::imu_samplesCallback, this, _1, _2 ),
	    buffer_size_factor * std::ceil( _max_delay.get() / _imu_period.get() ),
	    base::Time::fromSeconds( _imu_period.get() ) );      
    }      
    
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
    
    //Collect new input data and push them into the stream aligner
    base::samples::RigidBodyState orientation;
    while( _orientation_samples.read(orientation) == RTT::NewData )
    {
 	strAligner.push( orientationID, orientation.time, orientation );
	
    }

    base::samples::RigidBodyState gps_sample;
    while( _gps_samples.read(gps_sample) == RTT::NewData )
    {
 	strAligner.push( gpsID, gps_sample.time, gps_sample );
	
    } 
    
    base::samples::RigidBodyState velocity_sample;
    while( _velocity_samples.read(velocity_sample) == RTT::NewData )
    {
 	strAligner.push( velocityID, velocity_sample.time, velocity_sample );
	
    } 
    
    base::samples::IMUSensors imu_sample;
    while( _imu_samples.read(imu_sample) == RTT::NewData )
    {
 	strAligner.push( imuID, imu_sample.time, imu_sample );
	
    }     
    
    //Excecute stream-aligner steps
    while(strAligner.step());  
    
    
    //std::cout << "Update" << std::endl;
    //_stream_aligner_status.write(_stream_aligner.getStatus());
    
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

