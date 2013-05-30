require 'vizkit'
require 'orocos'
include Orocos
#Orocos::CORBA.name_service="192.168.128.60"
Orocos.initialize

#log = Orocos::Log::Replay.open("imu.7.log", "gps.6.log")
log = Orocos::Log::Replay.open("../../../../../logs/asv_unisee_28_5/asv_base_control.7.log")

Orocos.run  "asv_localization::Task"=> "asv_localization2"  ,:wait => 10000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
        
    
    asv_localization = TaskContext.get 'asv_localization2'
    gps = TaskContext.get "gps"
    imu = TaskContext.get "imu"
    
    asv_localization.gps_error = 1.0
    asv_localization.acceleration_error = 0.2
    asv_localization.velocity_error = 0.2

    asv_localization.gps_reject_threshold = 999999999999999
    asv_localization.velocity_reject_threshold = 9999999999999999

    asv_localization.gps_period = 0.1
    asv_localization.imu_period = 0.01
    asv_localization.ori_period = 0.01
    asv_localization.max_delay = 0.1

    asv_localization.initial_gps_origin = false
    asv_localization.estimate_velocity = true
    asv_localization.velocity_estimation_count = 100

    asv_localization.use_gps_velocity = 1

    asv_localization.imu_rotation = Math::PI * 55/180;

    asv_localization.relative_gps_position = Eigen::Vector3.new(0.3,0.38,0)


    log.gps.position_samples.connect_to asv_localization.gps_samples
    log.imu.orientation_samples.connect_to asv_localization.orientation_samples
    log.imu.compensated_sensors.connect_to asv_localization.imu_samples
    
    asv_localization.configure
    asv_localization.start
    


    #Vizkit.display simulation
    #Vizkit.display asv_actuators
    Vizkit.display imu
    Vizkit.display asv_localization
    
    Vizkit.control log
    Vizkit.exec
#6833.46
#Sum vel: 2725.46 Varianz: 0.115441 Std-Abweichung: 0.339767
#   Sum vel: 2447.3 Varianz: 0.103651 Std-Abweichung: 0.321948
#    Sum vel: 2480.12 Varianz: 0.105036 Std-Abweichung: 0.324093
#Sum vel: 2235.1 Varianz: 0.0946637 Std-Abweichung: 0.307675


end

