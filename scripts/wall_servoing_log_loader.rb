require 'vizkit'
require 'orocos'
include Orocos

Orocos.initialize

#view3d = Vizkit.vizkit3d_widget
#view3d.show()

log = Orocos::Log::Replay.open("../../../../logs/scripts2/sonar.12.log","../../../../logs/scripts2/uw_particle_localization_test.12.log")

Orocos.run 'wall_servoing_test', 'sonar_feature_estimator_test', 'auv_rel_pos_controller::Task' => 'auv_rel_pos_controller' , 'avalon_control::MotionControlTask' => 'motion_control', :wait => 10  do



    sonar = log.task "sonar"

    state_estimator = log.task "uw_particle_localization"

    


    ## wall_servoing
    wall_servoing = Orocos::TaskContext.get 'wall_servoing'
    # wall estimation settings
    wall_servoing.left_opening_angle = 0.2 * Math::PI
    wall_servoing.right_opening_angle = 0.6 * Math::PI

    wall_servoing.fading_out_factor = 0.02
    # controller settings
    wall_servoing.wall_distance = 4.0
    wall_servoing.fixed_depth = -2.5
    wall_servoing.servoing_speed = -1.0
    wall_servoing.exploration_speed = 0.1
    wall_servoing.servoing_wall_direction = 0.5*Math::PI #-0.25 * Math::PI - 0.8  #0.4#0.78538 # 1/4 PI
    wall_servoing.initial_wall_direction = 0.0
    wall_servoing.check_distance_threshold = 2.0
    
    wall_servoing.use_front_distance = true
    wall_servoing.left_front_angle = 0.1
    wall_servoing.right_front_angle = 0.1
    
    wall_servoing.use_motion_model = false
    wall_servoing.enable_debug_output = true
    ##

    ## feature_estimator
    feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'
    feature_estimator.derivative_history_length = 1
    feature_estimator.enable_debug_output = true
    feature_estimator.plain_threshold = 0.5
    feature_estimator.signal_threshold = 0.7
    ##

    ## auv_rel_pos_controller
    relPosController = Orocos::TaskContext.get 'auv_rel_pos_controller'
    relPosController.rel_heading = true
    relPosController.timeout = 0

    pid_settings = relPosController.controller_x
    pid_settings.zero!
    pid_settings.Ts = 0.01
    pid_settings.K = 0.5
    pid_settings.Ti = 0
    pid_settings.Td = 10
    pid_settings.YMin = -1.0 #-0.2
    pid_settings.YMax = 1.0 #0.2
    relPosController.controller_x = pid_settings
    pid_settings = relPosController.controller_y
    pid_settings.zero!
    pid_settings.Ts = 0.01
    pid_settings.K = 0.5
    pid_settings.Ti = 0
    pid_settings.Td = 10
    pid_settings.YMin = -0.8 #-0.2
    pid_settings.YMax = 0.8 #0.2
    relPosController.controller_y = pid_settings
    ##

    ## MotionControlTask
    motion_control = Orocos::TaskContext.get 'motion_control'
    motion_control.timeout = 0.0
    motion_control.cutoff = [0.65, 0.65, 0.65, 0.65, 0.65, 0.5]
    motion_control.z_coupling_factor = 0.0
    motion_control.y_coupling_factor = 0.0
    motion_control.y_factor = 1.0
    motion_control.x_factor = 1.0
    motion_control.pitch_target = 0.0

    pid_settings = motion_control.z_pid
    pid_settings.zero!
    pid_settings.p = -1.5
    pid_settings.d = 0.0
    pid_settings.min = -1.0
    pid_settings.max = 1.0
    motion_control.z_pid = pid_settings

    pid_settings.zero!
    pid_settings.p = -0.6
    pid_settings.d = 0.0
    pid_settings.min = -0.4
    pid_settings.max = 0.4
    motion_control.heading_pid = pid_settings

    pid_settings.zero!
    pid_settings.p = 2.0
    pid_settings.d = 0.0
    pid_settings.min = -1.0
    pid_settings.max = 1.0
    motion_control.pitch_pid = pid_settings
    ##

    ## connections
    sonar.sonar_beam.connect_to feature_estimator.sonar_input, :type => :buffer, :size => 100
    feature_estimator.new_feature.connect_to wall_servoing.sonarbeam_feature, :type => :buffer, :size => 100
    wall_servoing.position_command.connect_to relPosController.position_command
    relPosController.motion_command.connect_to motion_control.motion_commands

    state_estimator.pose_samples.connect_to wall_servoing.orientation_sample
    #state_estimator.pose_samples.connect_to wall_servoing.position_sample
    state_estimator.pose_samples.connect_to relPosController.position_sample
    state_estimator.pose_samples.connect_to feature_estimator.orientation_sample
    state_estimator.pose_samples.connect_to motion_control.pose_samples
    ##

    ## start tasks
    motion_control.configure
    motion_control.start

    relPosController.configure
    relPosController.start

    wall_servoing.configure
    wall_servoing.start

    feature_estimator.start
    ##

    ## run visualizations
    #sonarfeatureviz = Vizkit.default_loader.SonarFeatureVisualization
    #wallviz = Vizkit.default_loader.WallVisualization
    #auv_avalon = Vizkit.default_loader.AUVAvalonVisualization
    #auv_avalon.showDesiredModelPosition(true)

    #Vizkit.connect_port_to 'wall_servoing', 'wall_servoing_debug', :pull => false, :update_frequency => 33 do |sample, _|
    #    sonarfeatureviz.updatePointCloud(sample.pointCloud)
    #    wallviz.updateWallData(sample.wall)
    #end
    
    #wall_servoing.position_command.connect_to do |data,_|
    #    auv_avalon.updateDesiredPosition(data)
    #    data
    #end
    
    #state_estimator.pose_samples.connect_to do |data,_|
    #    auv_avalon.updateRigidBodyState(data)
    #    data
    #end

    #Vizkit.display wall_servoing
    #Vizkit.display state_estimator
    #Vizkit.display feature_estimator
    #Vizkit.display sonar
    
    Vizkit.control log
    Vizkit.exec
end
