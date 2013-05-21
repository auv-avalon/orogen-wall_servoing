require 'vizkit'
include Orocos

Orocos.initialize

widget = Vizkit.load "simulator.ui"

Orocos.run "AvalonSimulation", "asv_localization::Task"=> "asv_localization"  ,:wait => 10000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
    simulation = TaskContext.get 'avalon_simulation'
    simulation.scenefile = "#{ENV['AUTOPROJ_PROJECT_BASE']}/simulation/orogen/avalon_simulation/configuration/avalon.scn"

    simulation.debug_sonar = false 
    simulation.use_osg_ocean = false 
    simulation.enable_gui = true
    simulation.configure
    simulation.start
    asv_actuators = TaskContext.get 'asv_actuators'
    asv_actuators.configure
    asv_actuators.start
    asv_writer = asv_actuators.command.writer
    state = TaskContext.get 'state_estimator'
    state.configure
    state.start
    
    
    asv_localization = TaskContext.get 'asv_localization'

    asv_actuators.pose_samples.connect_to asv_localization.gps_samples
    asv_actuators.pose_samples.connect_to asv_localization.orientation_samples
    asv_actuators.pose_samples.connect_to asv_localization.velocity_samples
    
    asv_localization.configure
    asv_localization.start
    
    widget.joystick1.connect(SIGNAL('axisChanged(double,double)'))do |x,y|
        sample = asv_writer.new_sample
        sample.time = Time.now 
        0.upto(3) do
            sample.mode << :DM_PWM
            sample.target << 0;
        end
        sample.target[0] = x
        sample.target[1] = x
        sample.target[2] = -y
        sample.target[3] = y
        asv_writer.write sample
    end

    #Vizkit.display simulation
    #Vizkit.display asv_actuators
    #Vizkit.display asv_localization
    widget.show 
    Vizkit.exec


end
