require 'vizkit'
require 'orocos'
include Orocos

Orocos::CORBA.name_service = "127.0.0.1"
#Orocos::CORBA.name_service = "192.168.128.51"
Orocos.initialize

Orocos.run 'wall_servoing', 'sonar_feature_estimator', 'auv_rel_pos_controller', :wait => 10  do

    ## wall_servoing
    wall_servoing = Orocos::TaskContext.get 'wall_servoing'
    # wall estimation settings
    wall_servoing.wall_estimation_start_angle = 0.25 * Math::PI
    wall_servoing.wall_estimation_end_angle = -0.35 * Math::PI
    wall_servoing.wall_estimation_ransac_threshold = 0.5
    wall_servoing.wall_estimation_ransac_min_inliers = 0.85
    wall_servoing.dbscan_epsilon = 0.08726646259971647 * 1.5
    wall_servoing.fading_out_factor = 0.02
    # controller settings
    wall_servoing.wall_distance = 3.0
    wall_servoing.fixed_depth = -1.0
    wall_servoing.servoing_speed = 0.5
    wall_servoing.exploration_speed = 0.1
    wall_servoing.heading_modulation = 0.4#0.78538 # 1/4 PI
    ##

    ## feature_estimator
    feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'
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
    orientation_estimator = Orocos::TaskContext.get 'orientation_estimator'
    motion_control = Orocos::TaskContext.get 'motion_control'

    ## connections
    motion_control.disconnect_all

    sonar.sonar_beam.connect_to feature_estimator.sonar_input
    feature_estimator.new_feature.connect_to wall_servoing.sonarbeam_feature
    wall_servoing.position_command.connect_to relPosController.position_command
    relPosController.motion_command.connect_to motion_control.motion_commands

    orientation_estimator.pose_samples.connect_to wall_servoing.orientation_sample
    orientation_estimator.pose_samples.connect_to relPosController.position_sample
    orientation_estimator.pose_samples.connect_to feature_estimator.orientation_sample
    ##

    ## start tasks
    relPosController.configure
    relPosController.start

    wall_servoing.start

    feature_estimator.start
    ##

end
