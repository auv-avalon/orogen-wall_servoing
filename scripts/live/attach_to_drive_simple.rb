require 'orocos'
include Orocos

Orocos::CORBA.name_service = "127.0.0.1"
#Orocos::CORBA.name_service = "192.168.128.51"
Orocos.initialize

Orocos.run 'wall_servoing', 'sonar_feature_estimator', 'auv_rel_pos_controller', :wait => 10  do

    ## wall_servoing
    wall_servoing = Orocos::TaskContext.get 'wall_servoing'
    # wall estimation settings
    wall_servoing.left_opening_angle = 0.35 * Math::PI
    wall_servoing.right_opening_angle = 0.35 * Math::PI
    wall_servoing.wall_estimation_ransac_threshold = 0.5
    wall_servoing.wall_estimation_ransac_min_inliers = 0.85
    wall_servoing.dbscan_epsilon = 0.08726646259971647 * 1.5
    wall_servoing.fading_out_factor = 0.02
    # controller settings
    wall_servoing.wall_distance = 2.0
    wall_servoing.fixed_depth = -1.1
    wall_servoing.servoing_speed = 0.5
    wall_servoing.exploration_speed = 0.1
    wall_servoing.servoing_wall_direction = -0.4#0.78538 # 1/4 PI
    wall_servoing.inital_wall_direction = 0.0
    ##

    ## feature_estimator
    feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'
    feature_estimator.derivative_history_length = 1
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
    pid_settings.Td = 1.0
    pid_settings.YMin = -0.8
    pid_settings.YMax = 0.8
    relPosController.controller_x = pid_settings
    pid_settings = relPosController.controller_y
    pid_settings.zero!
    pid_settings.Ts = 0.01
    pid_settings.K = 0.8
    pid_settings.Ti = 0
    pid_settings.Td = 2.0
    pid_settings.YMin = -0.8
    pid_settings.YMax = 0.8
    relPosController.controller_y = pid_settings
    ##

    sonar = Orocos::TaskContext.get 'sonar'
    #sonar_config_writer = sonar.config_port.writer
    #config = sonar.config
    #config.cont = 0
    #config.maximumDistance = 5.0
    #config.leftLimit.rad = -0.785398163
    #config.rightLimit.rad = 0.785398163
    #sonar_config_writer.write(config)

    orientation_estimator = Orocos::TaskContext.get 'state_estimator'
    motion_control = Orocos::TaskContext.get 'motion_control'
    motion_control.timeout = 0

    ## connections
    motion_control.motion_commands.disconnect_all

    sonar.BaseScan.connect_to feature_estimator.sonar_input
    feature_estimator.new_feature.connect_to wall_servoing.sonarbeam_feature
    wall_servoing.position_command.connect_to relPosController.position_command
    relPosController.motion_command.connect_to motion_control.motion_commands

    orientation_estimator.orientation_samples.connect_to wall_servoing.orientation_sample
    orientation_estimator.orientation_samples.connect_to relPosController.position_sample
    orientation_estimator.orientation_samples.connect_to feature_estimator.orientation_sample
    ##

    ## start tasks
    relPosController.configure
    relPosController.start

    wall_servoing.start

    feature_estimator.start
    ##

    loop do 
    end
end
