require 'orocos'
require 'vizkit'
include Orocos

Orocos.initialize
log = Orocos::Log::Replay.open(ARGV, Typelib::Registry.new)
#sonar_port = log.find_port("/base/samples/SonarBeam")
sonar_port = log.sonar.BaseScan
orentation_port = log.state_estimator.orientation_samples
puts sonar_port.stream.info.interval_rt.first

Orocos.run 'wall_servoing_test', 'sonar_feature_estimator_test' do

    sonardetector = Orocos::TaskContext.get 'wall_servoing'
    feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'
    #feature_estimator.derivative_history_length = 1

    # wall estimation settings
    sonardetector.left_opening_angle = Math::PI * 0.35
    sonardetector.right_opening_angle = Math::PI * 0.25
    sonardetector.wall_estimation_ransac_threshold = 0.2
    sonardetector.wall_estimation_ransac_min_inliers = 0.8
    sonardetector.dbscan_epsilon = 0.08726646259971647 * 1.5
    sonardetector.fading_out_factor = 0.02
    sonardetector.enable_debug_output = true

    # controller settings
    sonardetector.wall_distance = 3
    sonardetector.fixed_depth = -1.5
    sonardetector.servoing_speed = 0.5
    sonardetector.exploration_speed = 0.1
    sonardetector.servoing_wall_direction = -0.4
    sonardetector.inital_wall_direction = 0.0

    sonar_port.connect_to feature_estimator.sonar_input, :type => :buffer, :size => 100
    feature_estimator.new_feature.connect_to sonardetector.sonarbeam_feature, :type => :buffer, :size => 100
    orentation_port.connect_to sonardetector.orientation_sample
    orentation_port.connect_to feature_estimator.orientation_sample

    view3d = Vizkit.vizkit3d_widget
    view3d.show()
    sonarfeatureviz = Vizkit.default_loader.SonarFeatureVisualization
    wallviz = Vizkit.default_loader.WallVisualization
    auv_avalon = Vizkit.default_loader.AUVAvalonVisualization
    auv_avalon.showDesiredModelPosition(true)
    #sonarbeamviz = Vizkit.default_loader.SonarBeamVisualization

    # Connect debug port to vizkit plugins
    con = Vizkit.connect_port_to 'wall_servoing', 'wall_servoing_debug', :pull => false, :update_frequency => 33 do |sample, name|
        sonarfeatureviz.updatePointCloud(sample.pointCloud)
        wallviz.updateWallData(sample.wall)
    end 

    con = Vizkit.connect_port_to 'wall_servoing', 'position_command', :pull => false, :update_frequency => 33 do |sample, name|
        auv_avalon.updateDesiredPosition(sample)
    end 

    log.state_estimator.orientation_samples do |sample|
        auv_avalon.updateRigidBodyState(sample)
        #sonarbeamviz.updateBodyState(sample)
        sample
    end 
    
    #log.sonar.BaseScan do |sample|
    #    sonarbeamviz.updateSonarBeam(sample)
    #    sample
    #end 

    sonardetector.start

    feature_estimator.start

    viewer = Vizkit.default_loader.StructViewer
    log.sonar.BaseScan.connect_to viewer
    viewer.show

    task_inspector = Vizkit.default_loader.task_inspector
    task_inspector.config(sonardetector)
    task_inspector.show
    Vizkit.control log
    Vizkit.exec
end


