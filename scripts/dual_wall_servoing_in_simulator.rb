require 'vizkit'
require 'orocos'
include Orocos

Orocos.initialize

Orocos.run 'AvalonSimulation', 'wall_servoing_test', 'sonar_feature_estimator_test', 'auv_rel_pos_controller', 'avalon_control_simulation', :wait => 10  do

    ## simulator
    simulation = TaskContext.get 'avalon_simulation'
    simulation.debug_sonar = 1
    simulation.enable_gui = true
    simulation.use_osg_ocean = false
    simulation.scenefile = "#{ENV['AUTOPROJ_PROJECT_BASE']}/simulation/orogen/avalon_simulation/configuration/demo.scn"
    simulation.configure
    simulation.start

    actuactors = TaskContext.get 'actuators'
    actuactors.configure
    actuactors.start

    sonar = TaskContext.get 'sonar'
    sonar.ping_pong_mode = true
    sonar.start_angle = 0.03 * Math::PI
    sonar.end_angle = -0.53 * Math::PI
    sonar.maximum_distance = 20.0
    sonar.configure
    sonar.start

    sonar_rear = TaskContext.get 'sonar_rear'
    sonar_rear.ping_pong_mode = true
    sonar_rear.start_angle = 0.01 * Math::PI
    sonar_rear.end_angle = -0.01 * Math::PI
    sonar_rear.maximum_distance = 20.0
    sonar_rear.configure
    sonar_rear.start

    state_estimator = TaskContext.get 'state_estimator'
    state_estimator.configure
    state_estimator.start

    simulation.setPosition(0,0,0)
    #simulation.setPosition(0,-15,0)
    #simulation.setOrientation(0,0,0.707,0.707)
    #simulation.setOrientation(0,0,0.0,0.0)
    ##

    ## wall_servoing
    wall_servoing = Orocos::TaskContext.get 'dual_wall_servoing'
    # wall estimation settings
    wall_servoing.opening_angle = 0.1 * Math::PI
    # controller settings
    wall_servoing.wall_distance = 2.5
    wall_servoing.fixed_depth = -1.5
    wall_servoing.servoing_speed = 0.5
    wall_servoing.exploration_speed = -0.1
    ##

    ## feature_estimator
    feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'
    feature_estimator.derivative_history_length = 1
    feature_estimator.enable_debug_output = true
    feature_estimator_2 = Orocos::TaskContext.get 'sonar_feature_estimator_2'
    feature_estimator_2.derivative_history_length = 1
    feature_estimator_2.enable_debug_output = true
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
    motion_control.cutoff = 0.5
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
    pid_settings.p = -2.0
    pid_settings.d = 0.0
    pid_settings.min = -1.0
    pid_settings.max = 1.0
    motion_control.heading_pid = pid_settings

    pid_settings.zero!
    pid_settings.p = 2.0
    pid_settings.d = 0.0
    pid_settings.min = -1.0
    pid_settings.max = 1.0
    motion_control.pitch_pid = pid_settings
    ##

    ## connections
    sonar.sonar_beam.connect_to feature_estimator.sonar_input
    sonar_rear.sonar_beam.connect_to feature_estimator_2.sonar_input
    feature_estimator.new_feature.connect_to wall_servoing.sonarbeam_feature_front
    feature_estimator_2.new_feature.connect_to wall_servoing.sonarbeam_feature_rear
    wall_servoing.position_command.connect_to relPosController.position_command
    relPosController.motion_command.connect_to motion_control.motion_commands
    motion_control.hbridge_commands.connect_to actuactors.command

    state_estimator.pose_samples.connect_to wall_servoing.orientation_sample
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
    feature_estimator_2.start
    ##

    Vizkit.display wall_servoing

    Vizkit.exec
end
