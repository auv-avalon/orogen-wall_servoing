require 'vizkit'
require 'orocos'
include Orocos

Orocos.initialize

#view3d = Vizkit.vizkit3d_widget
#view3d.show()

Orocos.run "AvalonSimulation" , "wall_servoing_test", "sonar_feature_estimator::Task" => "sonar_feature_estimator" ,:wait => 100, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
    

    Orocos.conf.load_dir(File.join(ENV['AUTOPROJ_PROJECT_BASE'],"bundles", "avalon", "config", "orogen")) 
    simulation = TaskContext.get 'avalon_simulation'
    
      white_light = TaskContext.get 'white_light'
#     white_light.interval_mode = 1
#     white_light.constantInterval = 3000
      white_light.interval_mode = 2
      white_light.randomInterval_min = 1;
      white_light.randomInterval_max = 5000;
      white_light.start
      
    Orocos.conf.apply(simulation,['default'])
    simulation.configure
    simulation.start

# Konfiguration der Aktuatoren für alle Fahrzeuge:    
    require 'actuators'
    values = ActuatorsConfig.new()
    
    actuators = TaskContext.get 'avalon_actuators'
    actuators.node_name = "avalon"
    actuators.amount_of_actuators = values.avalon_amount_of_actuators
    actuators.maximum_thruster_force = values.avalon_maximum_thruster_force    
    actuators.thruster_position = values.avalon_thruster_position    
    actuators.thruster_direction = values.avalon_thruster_direction    
    actuators.linear_damp = [[8, 24, 24, 10, 10, 10]]
    actuators.square_damp = [[0, 0, 0, 0, 0, 0]]
    actuators.thruster_coefficients = [0.005, 0.005, 0.005, 0.005, 0.005, 0.005]
    actuators.buoyancy_force = 72
    actuators.cob = [[0, 0, 0.08]]
    actuators.voltage = 28
    actuators.configure
    actuators.start
    

# Camera configuration

    front_cam = TaskContext.get 'front_camera'
    front_cam.name = 'front_cam'
    front_cam.configure
   # front_cam.start
    
    bottom_cam = TaskContext.get 'bottom_camera'
    bottom_cam.name = 'bottom_cam'
    bottom_cam.configure
   # bottom_cam.start
    
    sonar = TaskContext.get 'sonar'
    sonar.node_name = "sonar_top_sensor"
    sonar.left_limit = 0.75 * Math::PI
    sonar.right_limit = -0.25 * Math::PI
    sonar.resolution = 0.1
    sonar.maximum_distance = 40.0
    sonar.ping_pong_mode = true
    sonar.configure
    sonar.start
    
    sonar_rear = TaskContext.get 'sonar_rear'
    sonar_rear.node_name = "sonar_rear_sensor"
    sonar_rear.left_limit = 0.7*Math::PI
    sonar_rear.right_limit = 0.3*Math::PI
    sonar_rear.resolution = 0.1
    sonar_rear.maximum_distance = 40.0
    sonar_rear.ping_pong_mode = true
    sonar_rear.configure
   # sonar_rear.start
        
    ground_distance = TaskContext.get 'ground_distance'
    ground_distance.node_name = "ground_distance_sensor"
    ground_distance.configure
    ground_distance.start

    imu = TaskContext.get 'imu'
    imu.name = "avalon"
    imu.configure
    imu.start

    ## wall_servoing
    wall_servoing = Orocos::TaskContext.get 'wall_servoing'
    # wall estimation settings
    wall_servoing.left_opening_angle = 0.35 * Math::PI
    wall_servoing.right_opening_angle = 0.5 * Math::PI
   # wall_servoing.wall_estimation_ransac_threshold = 0.5
   # wall_servoing.wall_estimation_ransac_min_inliers = 0.85
   # wall_servoing.dbscan_epsilon = 0.08726646259971647 * 1.5
    wall_servoing.fading_out_factor = 0.02
    # controller settings
    wall_servoing.wall_distance = 3.0
    wall_servoing.fixed_depth = -2.5
    wall_servoing.servoing_speed = -1.0
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
    
    sonar.sonar_beam.connect_to feature_estimator.sonar_input, :type => :buffer, :size => 100
    feature_estimator.new_feature.connect_to wall_servoing.sonarbeam_feature, :type => :buffer, :size => 100

    imu.pose_samples.connect_to(wall_servoing.orientation_sample)
    imu.pose_samples.connect_to(wall_servoing.position_sample)
    imu.pose_samples.connect_to(feature_estimator.orientation_sample)

    wall_servoing.configure
    wall_servoing.start

    feature_estimator.configure
    feature_estimator.start
    
    ## wall_servoing
    wall_detector = Orocos::TaskContext.get 'wall_detector'
    # wall estimation settings
    wall_detector.opening_angle = 0.2 * Math::PI
    wall_detector.fading_out_factor = 0.006
    wall_detector.wall_direction =  0.5 * Math::PI
    wall_detector.use_motion_model = true
    wall_detector.wall_estimation_timeout = 10
    # controller settings
    
    imu.pose_samples.connect_to(wall_detector.orientation_sample)
    imu.pose_samples.connect_to(wall_detector.position_sample)
    feature_estimator.new_feature.connect_to wall_detector.sonarbeam_feature, :type => :buffer, :size => 100
    
    wall_detector.configure
    wall_detector.start
    ##
#Control Draft
###########################WORLD_TO_ALIGNED
    world_to_aligned = TaskContext.get 'world_to_aligned'
    
    Orocos.conf.apply(world_to_aligned,['default'])
    
    expected = world_to_aligned.expected_inputs
        expected.linear[0] = false
        expected.linear[1] = false
        expected.linear[2] = true

        expected.angular[0] = false
        expected.angular[1] = true
        expected.angular[2] = false
    world_to_aligned.expected_inputs = expected

    imu.pose_samples.connect_to(world_to_aligned.pose_samples)
    
    world_to_aligned.configure()
###########################ALIGNED_POSITION_CONTROLLER
    apc = TaskContext.get 'aligned_position_controller'
    
    Orocos.conf.apply(apc,['default_aligned_position_simulation'])
    
    expected = apc.expected_inputs
        expected.linear[0] = false
        expected.linear[1] = true
        expected.linear[2] = true

        expected.angular[0] = false
        expected.angular[1] = true
        expected.angular[2] = true
    apc.expected_inputs = expected
    
    imu.pose_samples.connect_to(apc.pose_samples)
    
    apc.configure()
###########################ALIGNED_VELOCITY_CONTROLLER
    avc = TaskContext.get 'aligned_velocity_controller'
    
    Orocos.conf.apply(avc,['default_aligned_velocity_simulation'])
    
    imu.pose_samples.connect_to(avc.pose_samples)
    
    avc.configure()
###########################ALIGNED_TO_BODY
    aligned_to_body = TaskContext.get 'aligned_to_body'
    
    Orocos.conf.apply(aligned_to_body,['default'])

    imu.pose_samples.connect_to(aligned_to_body.orientation_samples)
    
    aligned_to_body.configure()

    
###########################ACCELERATION_CONTROLLER
    acceleration_controller = TaskContext.get 'acceleration_controller'
    
    Orocos.conf.apply(acceleration_controller,['default_simulation'])
    
    acceleration_controller.configure
###########################CONNECTIONS
   
    acceleration_controller.cmd_out.connect_to(actuators.command) 
    aligned_to_body.cmd_out.connect_to(acceleration_controller.cmd_cascade)
    avc.cmd_out.connect_to(aligned_to_body.cmd_cascade)
    wall_servoing.aligned_velocity_command.connect_to(avc.cmd_in)
    apc.cmd_out.connect_to(avc.cmd_cascade)
    wall_servoing.aligned_position_command.connect_to(apc.cmd_in)
    world_to_aligned.cmd_out.connect_to(apc.cmd_cascade)
    wall_servoing.world_command.connect_to(world_to_aligned.cmd_in)
    
    world_to_aligned.start
    apc.start
    puts "sleep5"
    sleep 5
    avc.start
    aligned_to_body.start
    acceleration_controller.start
    
    loop do
        sleep 0.5
    end

end
