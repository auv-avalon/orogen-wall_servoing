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
  
  base::Vector3d pos;
  
  //Convert EN-Frame to NW-Frame
  pos[0] = gps_samples_sample.position[1];
  pos[1] = -gps_samples_sample.position[0];
  pos[2] = 0.0; //We asume, that we are only on the surface
  
  base::samples::RigidBodyState rbs = gps_samples_sample;
  rbs.position = pos;
  
  //If we have a initial position, then observe. Else initialize position
  if(firstPositionRecieved){
     
    if(firstOrientationRecieved){       
      
      pos = pos - firstGpsSample.position;
      
      base::Matrix3d covariance = base::Matrix3d::Identity() * _gps_error.get();
      
      //Observe
      if(ekf.positionObservation( pos, covariance, _gps_reject_threshold.get() )){
	std::cout << "Rejected GPS-Sample" << std::endl;
      }else if(_estimate_velocity.get() ){ //When the observation was valid and we want to estimate the velocity ->
	
	gpsPositions.push_back(rbs);
		
	if(gpsPositions.full() ){
	  
	  base::samples::RigidBodyState oldSample = *(gpsPositions.begin()+1); //Get the first sample 
	  
	  //Calculate the relative movement between the samples
	  double relX = cos(base::getYaw(ekf.getRotation())) * (pos[0] - oldSample.position[0])
			  - sin(base::getYaw(ekf.getRotation())) * (pos[1] - oldSample.position[1]);
			  
	  double relY = sin(base::getYaw(ekf.getRotation())) * (pos[0] - oldSample.position[0])		
			  + cos(base::getYaw(ekf.getRotation())) * (pos[1] - oldSample.position[1]);
	  
	  //Calculate the velocity from movement and time-delay		  
	  base::Vector3d vel;
	  vel[0] = relX / (ts.toSeconds() - oldSample.time.toSeconds());
	  vel[1] = relY / (ts.toSeconds() - oldSample.time.toSeconds());
	  vel[2] = 0;
	  
	  base::Matrix3d covar = base::Matrix3d::Identity() * _velocity_error.get() ;
	  actualVelocity = vel;
	  
	  //Velocity-Observation
	  if(ekf.velocityObservation(vel, covar, _velocity_reject_threshold.get()))
	    std::cout << "Rejected Velocity" << std::endl;	    

      } 
      }
      
    }    
    
  }else{ //First gps-sample
    
    base::Matrix3d covariance = base::Matrix3d::Identity() * _gps_error.get();
    
    
    lastGpsSample = rbs;
    firstGpsSample = rbs;
    firstPositionRecieved = true;
    
    //Use 0,0,0 as origin
    if(!_initial_gps_origin.get())
      firstGpsSample.position = base::Vector3d::Zero();
    
    ekf.setPosition( pos , covariance);

    std::cout << "Initialize Position";
  }
  
  
  lastGpsTime = ts;
  
}

void Task::imu_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample)
{  
  
  if(!lastImuTime.isNull() && firstOrientationRecieved){
    
    base::samples::IMUSensors sample = imu_samples_sample;
    
    double pitch = base::getPitch(lastOrientation) - 0.1; //Offset for imu calibration TODO calibrate imu!!
    double roll = base::getRoll(lastOrientation) ;//+ 0.034;
    const double gravity = 9.871; 
    
    //The asv-imu is turned to 90 degrees -> change x and y acceleration
    //Use pitch and roll to eliminate gravity
    sample.acc[0] = -imu_samples_sample.acc[1] - gravity * sin(pitch);
    sample.acc[1] = imu_samples_sample.acc[0] + gravity * sin(roll);  
    sample.acc[2] = 0.0; //gravity; //As we only drive on the surface, there is no z-acceleration
    
    //std::cout << "Pitch: " << pitch << " Roll: " << roll << std::endl;
    //std::cout << "New Acceleration: " << sample.acc.transpose() << std::endl;
    
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
    
    ekf.predict( sample.acc, dt, process_noise);
      
  }
  
  lastImuTime = ts;
  
}

void Task::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{    
  //std::cout << "Orientation callback" << std::endl;
  firstOrientationRecieved = true;
  
  base::Orientation ori = imuRotation * orientation_samples_sample.orientation;
  lastOrientation = ori;

  ekf.setRotation(ori);
  
  //Write out actual state  
  if(firstPositionRecieved){
    
    base::samples::RigidBodyState rbs;
    rbs.position = ekf.getPosition();
    rbs.position[2] = 0.0;
    
    if(_use_gps_velocity.get())
      rbs.velocity = actualVelocity;
    else
      rbs.velocity = ekf.getVelocity();
    
    rbs.orientation = lastOrientation;
    rbs.angular_velocity = orientation_samples_sample.angular_velocity;
    rbs.time = base::Time::now();
    rbs.cov_position = ekf.getPositionCovariance();
    rbs.cov_velocity = ekf.getVelocityCovariance();
    _pose_samples.write(rbs);
    
  }  
  
  
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
    
    orientationID = -1;
    imuID = -1;
    gpsID = -1;
    velocityID = -1;
    actualVelocity = base::Vector3d::Zero();
    
    imuRotation = base::Quaterniond(Eigen::AngleAxisd(_imu_rotation.get() , Eigen::Vector3d::UnitZ()) );
    
    gpsPositions = boost::circular_buffer<base::samples::RigidBodyState>(_velocity_estimation_count.get());
    
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
    

    //ekf = pose_ekf::KFD_PosVelAcc();
    
    return true;
    
}



void Task::updateHook()
{

    RTT::TaskContext::updateHook();
    
    //Collect new input data and push them into the stream aligner
    
    if(orientationID != -1){
      base::samples::RigidBodyState orientation;
      while( _orientation_samples.read(orientation) == RTT::NewData )
      {
	  strAligner.push( orientationID, orientation.time, orientation );
	  
      }
    }  

    
    if(gpsID != -1){
      base::samples::RigidBodyState gps_sample;
      while( _gps_samples.read(gps_sample) == RTT::NewData )
      {
	  strAligner.push( gpsID, gps_sample.time, gps_sample );
	  
      }
    }  
    
    if(velocityID != -1){
      base::samples::RigidBodyState velocity_sample;
      while( _velocity_samples.read(velocity_sample) == RTT::NewData )
      {
	  strAligner.push( velocityID, velocity_sample.time, velocity_sample );
	  
      }
    }  
    
    if(imuID != -1){
      base::samples::IMUSensors imu_sample;
      while( _imu_samples.read(imu_sample) == RTT::NewData )
      {
	  strAligner.push( imuID, imu_sample.time, imu_sample );
	  
      }
    }   
    
    //Excecute stream-aligner steps
    while(strAligner.step());  
    
    
    //std::cout << "Update" << std::endl;
    _stream_aligner_status.write(strAligner.getStatus());
        
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

