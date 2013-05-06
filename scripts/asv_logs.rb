require 'vizkit'
require 'orocos'
include Orocos

Orocos.initialize

log = Orocos::Log::Replay.open("imu.7.log", "gps.6.log")

Orocos.run  "asv_localization::Task"=> "asv_localization"  ,:wait => 10000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
        
    
    asv_localization = TaskContext.get 'asv_localization'
    gps = log.task "gps"
    imu = log.task "imu"

    gps.position_samples.connect_to asv_localization.gps_samples
    imu.orientation_samples.connect_to asv_localization.orientation_samples
    imu.raw_sensors.connect_to asv_localization.imu_samples
    
    asv_localization.configure
    asv_localization.start
    


    #Vizkit.display simulation
    #Vizkit.display asv_actuators
    #Vizkit.display asv_localization
    
    Vizkit.control log
    Vizkit.exec


end

