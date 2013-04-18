require 'vizkit'
require 'orocos'
include Orocos

Orocos.initialize

#view3d = Vizkit.vizkit3d_widget
#view3d.show()

Orocos.run 'AvalonSimulation', 'wall_servoing_test', 'sonar_feature_estimator_test', "avalon_back_base_control", "controldev",  :wait => 10  do
    Orocos.log_all
    ## simulator
    ## simulator
    simulation = TaskContext.get 'avalon_simulation'
    simulation.debug_sonar = 1
    simulation.enable_gui = true
    simulation.use_osg_ocean = false
    simulation.configure
    simulation.start
    actuators = TaskContext.get 'actuators'
    actuators.configure
    actuators.start
    writer = actuators.command.writer
    sonar = TaskContext.get 'sonar'
    sonar.ping_pong_mode = true
    sonar.left_limit = 0.65 * Math::PI
    sonar.right_limit = -0.2 * Math::PI
    sonar.configure
    sonar.start
    state_estimator = TaskContext.get 'state_estimator'
    state_estimator.configure
    state_estimator.start
    
    #simulation.setPosition(55,0,0)
    simulation.setPosition(45,20,0)
    #simulation.setPosition(-40,-15,0)
   
    simulation.setOrientation(0,0,0.707,0.707)
    ##

    ## wall_servoing
    wall_servoing = Orocos::TaskContext.get 'wall_servoing'
    # wall estimation settings
    wall_servoing.left_opening_angle = 0.5 * Math::PI
    wall_servoing.right_opening_angle = 0.35 * Math::PI
    wall_servoing.wall_estimation_ransac_threshold = 0.5
    wall_servoing.wall_estimation_ransac_min_inliers = 0.85
    wall_servoing.dbscan_epsilon = 0.08726646259971647 * 1.5
    wall_servoing.fading_out_factor = 0.02
    # controller settings
    wall_servoing.wall_distance = 4.0
    wall_servoing.fixed_depth = -2.5
    wall_servoing.servoing_speed = -0.4
    wall_servoing.exploration_speed = 0.1
    wall_servoing.servoing_wall_direction = 0.5*Math::PI #-0.25 * Math::PI - 0.8  #0.4#0.78538 # 1/4 PI
    wall_servoing.initial_wall_direction = 0.0
    wall_servoing.check_distance_threshold = 2.0
    
    wall_servoing.use_motion_model = true
    wall_servoing.enable_debug_output = true
    ##

    ## feature_estimator
    feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'
    feature_estimator.derivative_history_length = 1
    feature_estimator.enable_debug_output = true
    feature_estimator.plain_threshold = 0.5
    feature_estimator.signal_threshold = 0.7
    ##

    @world_controller = TaskContext.get "world_controller"
    expected = @world_controller.expected_inputs
    expected.linear[0] = false
    expected.linear[1] = false
    expected.linear[2] = true
    expected.angular[0] = false
    expected.angular[1] = true
    expected.angular[2] = false
    @world_controller.expected_inputs = expected
    state_estimator.pose_samples.connect_to @world_controller.pose_sample
    
    @relative_controller = TaskContext.get "relative_controller"
    expected = @relative_controller.expected_inputs
    expected.linear[0] = true
    expected.linear[1] = true
    expected.linear[2] = false
    expected.angular[0] = false
    expected.angular[1] = false
    expected.angular[2] = true
    @relative_controller.expected_inputs = expected
    state_estimator.pose_samples.connect_to @relative_controller.pose_sample
    
    @aligned_controller = TaskContext.get "aligned_controller"
    expected = @aligned_controller.expected_inputs
    expected.linear[0] = true
    expected.linear[1] = true
    expected.linear[2] = true
    expected.angular[0] = false
    expected.angular[1] = true
    expected.angular[2] = true
    @aligned_controller.expected_inputs = expected
    pid = @aligned_controller.pid_settings
    pid_linear = pid.linear
    setting = pid_linear[0]
    setting.Ts   = 0.01
    setting.K    = 1
    setting.Ti   = 0
    setting.Td   = 0
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_linear[0] = setting
    setting = pid_linear[1]
    setting.Ts   = 0.01
    setting.K    = 1
    setting.Ti   = 0
    setting.Td   = 0
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_linear[1] = setting
    setting = pid_linear[2]
    setting.Ts   = 0.01
    setting.K    = 0.1
    setting.Ti   = 0
    setting.Td   = 0
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_linear[2] = setting
    pid.linear = pid_linear
    pid_angular = pid.angular
    setting = pid_angular[0]
    setting.Ts   = 0.01
    setting.K    = 0.00000000000000001
    setting.Ti   = 0
    setting.Td   = 0
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_angular[0] = setting
    setting = pid_angular[1]
    setting.Ts   = 0.01
    setting.K    = 0.2
    setting.Ti   = 0
    setting.Td   = 0
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_angular[1] = setting
    setting = pid_angular[2]
    setting.Ts   = 0.01
    setting.K    = 0.1
    setting.Ti   = 0
    setting.Td   = 0
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_angular[2] = setting
    pid.angular = pid_angular
    @aligned_controller.pid_settings = pid
                                                                        
    #print "Connect DEPTH_ORIENTATION_READER -> ALIGNED_CONTROLLER\n"
    state_estimator.pose_samples.connect_to @aligned_controller.pose_sample
    @relative_controller.cmd_out.connect_to @aligned_controller.cmd_in
    #print "WORLD_CONTROLLER -> ALIGNED_CONTROLLER\n"
    @world_controller.cmd_out.connect_to @aligned_controller.cascade
    
    
    @aligned_velocity_controller = TaskContext.get "aligned_velocity_controller"
    expected = @aligned_velocity_controller.expected_inputs
    expected.linear[0] = true
    expected.linear[1] = true
    expected.linear[2] = true
    expected.angular[0] = false
    expected.angular[1] = true
    expected.angular[2] = true
    @aligned_velocity_controller.expected_inputs = expected

    @aligned_velocity_controller.drive_simple = true

    pid = @aligned_velocity_controller.pid_settings
    pid_linear = pid.linear
    setting = pid_linear[0]
    setting.Ts   = 0.01
    setting.K    = 1
    setting.Ti   = 0
    setting.Td   = 0
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_linear[0] = setting
    setting = pid_linear[1]
    setting.Ts   = 0.01
    setting.K    = 1
    setting.Ti   = 0
    setting.Td   = 0
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_linear[1] = setting
    setting = pid_linear[2]
    setting.Ts   = 0.01
    setting.K    = 1.65
    setting.Ti   = 0.215
    setting.Td   = 0.388
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_linear[2] = setting
    pid.linear = pid_linear
    pid_angular = pid.angular
    setting = pid_angular[0]
    setting.Ts   = 0.01
    setting.K    = 0.0000000001
    setting.Ti   = 0
    setting.Td   = 0
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_angular[0] = setting
    setting = pid_angular[1]
    setting.Ts   = 0.01
    setting.K    = 1
    setting.Ti   = 0.02
    setting.Td   = 0.0133333333
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_angular[1] = setting
    setting = pid_angular[2]
    setting.Ts   = 0.01
    setting.K    = 6.666
    setting.Ti   = 1.375
    setting.Td   = 0.09166
    setting.N    = 0
    setting.B    = 1
    setting.Tt   = -1
    setting.YMin = -1.0
    setting.YMax = 1.0
    pid_angular[2] = setting
    pid.angular = pid_angular
    @aligned_velocity_controller.pid_settings = pid
                                                                                                                                                                                                                                    

    #print "Connect DEPTH_ORIENTATION_READER -> ALIGNED_VELOCITY_CONTROLLER\n"
    state_estimator.pose_samples.connect_to @aligned_velocity_controller.pose_sample
    @aligned_controller.cmd_out.connect_to @aligned_velocity_controller.cascade
    
    @force_torque_controller = TaskContext.get "force_torque_controller"
    cut_off = @force_torque_controller.cut_off
    cut_off.push(1.0)
    cut_off.push(1.0)
    cut_off.push(1.0)
    cut_off.push(1.0)
    cut_off.push(1.0)
    cut_off.push(1.0)
    @force_torque_controller.cut_off = cut_off
    
    calibration = @force_torque_controller.calibration
    calibration.resize(6,6)
    calibration[0, 0] = 0
    calibration[0, 1] = 0
    calibration[0, 2] = -1 #-0.6
    calibration[0, 3] = -1 #-0.65
    calibration[0, 4] = 0
    calibration[0, 5] = 0

    calibration[1, 0] = 0
    calibration[1, 1] = 0
    calibration[1, 2] = 0
    calibration[1, 3] = 0
    calibration[1, 4] = 0.0001 #0.2
    calibration[1, 5] = -0.8 #-0.55

    calibration[2, 0] = 0.02 #0.3
    calibration[2, 1] = -1.0 #-0.6
    calibration[2, 2] = 0
    calibration[2, 3] = 0
    calibration[2, 4] = 0
    calibration[2, 5] = 0

    calibration[3, 0] = 0
    calibration[3, 1] = 0
    calibration[3, 2] = 0
    calibration[3, 3] = 0
    calibration[3, 4] = 0
    calibration[3, 5] = 0

    calibration[4, 0] = 1.0 #0.45
    calibration[4, 1] = 0.1 #0.15
    calibration[4, 2] = 0
    calibration[4, 3] = 0
    calibration[4, 4] = 0
    calibration[4, 5] = 0

    calibration[5, 0] = 0
    calibration[5, 1] = 0
    calibration[5, 2] = 0
    calibration[5, 3] = 0
    calibration[5, 4] = -1.0 #-0.6
    calibration[5, 5] = -0.2 #-0.4
    @force_torque_controller.calibration = calibration
    #print "FORCE_TORQUE_CONTROLLER -> HBRIDGE\n"
    @force_torque_controller.motor_command.connect_to actuators.command
    @aligned_velocity_controller.cmd_out.connect_to @force_torque_controller.cascade
    state_estimator.pose_samples.connect_to @force_torque_controller.pose_sample
    
    
    

    ## connections
    sonar.sonar_beam.connect_to feature_estimator.sonar_input, :type => :buffer, :size => 100
    feature_estimator.new_feature.connect_to wall_servoing.sonarbeam_feature, :type => :buffer, :size => 100
 
    state_estimator.pose_samples.connect_to wall_servoing.orientation_sample
    state_estimator.pose_samples.connect_to wall_servoing.position_sample
    state_estimator.pose_samples.connect_to feature_estimator.orientation_sample
    
    wall_servoing.aligned_command.connect_to @relative_controller.cmd_in
    wall_servoing.world_command.connect_to @world_controller.cmd_in

    ##
    
    @aligned_controller.start
    @aligned_velocity_controller.start
    @force_torque_controller.start    
    @world_controller.start
    @relative_controller.start
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
    #Vizkit.display simulation    

    Vizkit.exec
end
