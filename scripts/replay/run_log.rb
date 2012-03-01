require 'orocos'
require 'vizkit'
include Orocos

Orocos.initialize
log = Orocos::Log::Replay.open(ARGV, Typelib::Registry.new)
#sonar_port = log.find_port("/base/samples/SonarBeam")
sonar_port = log.sonar.BaseScan
orentation_port = log.state_estimator.orientation_samples
puts sonar_port.stream.info.interval_rt.first

Orocos.run 'wall_servoing', 'sonar_feature_estimator' do

    sonardetector = Orocos::TaskContext.get 'wall_servoing'
    feature_estimator = Orocos::TaskContext.get 'sonar_feature_estimator'
    #feature_estimator.derivative_history_length = 1

    # wall estimation settings
    sonardetector.wall_estimation_start_angle = Math::PI * 0.25
    sonardetector.wall_estimation_end_angle = -Math::PI * 0.25
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
    sonardetector.heading_modulation = 0.41

    sonar_port.connect_to feature_estimator.sonar_input
    feature_estimator.new_feature.connect_to sonardetector.sonarbeam_feature
    orentation_port.connect_to sonardetector.orientation_sample
    orentation_port.connect_to feature_estimator.orientation_sample

    view3d = Vizkit.default_loader.create_widget('vizkit::Vizkit3DWidget')
    view3d.show()
    sonarfeatureviz = view3d.createPlugin('sonarfeature', 'SonarFeatureVisualization')
    wallviz = view3d.createPlugin('wall', 'WallVisualization')
    auv_avalon = view3d.createPlugin('auv_avalon', 'AUVAvalonVisualization')
    auv_avalon.showDesiredModelPosition(true)
    #sonarbeamviz = view3d.createPlugin('sonarbeam', 'SonarBeamVisualization')

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


