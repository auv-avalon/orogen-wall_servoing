require 'orocos'
require 'vizkit'
require 'vizkit/widgets/task_inspector/task_inspector'
include Orocos

Orocos.initialize
log = Orocos::Log::Replay.open(ARGV)

Orocos.run 'sonardetector' do

    sonardetector = Orocos::TaskContext.get 'sonardetector'

    # sonarbeam settings
    sonardetector.enable_beam_threshold = true
    sonardetector.beam_threshold_min = 1.5
    sonardetector.beam_threshold_max = 15
    sonardetector.min_response_value = 30

    # wall estimation settings
    sonardetector.wall_estimation_start_angle = -1
    sonardetector.wall_estimation_end_angle = -1
    sonardetector.wall_estimation_ransac_threshold = 0.5
    sonardetector.wall_estimation_ransac_min_inliers = 0.6

    # controller settings
    sonardetector.wall_distance = 2
    sonardetector.fixed_depth = -1.5
    sonardetector.servoing_speed = 0.2
    sonardetector.exploration_speed = 0.2
    sonardetector.heading_modulation = 0.0

    log.sonar.BaseScan.connect_to sonardetector.sonar_input
    log.orientation_estimator.orientation_samples.connect_to sonardetector.orientation_sample

    view3d = Vizkit.default_loader.create_widget('vizkit::Vizkit3DWidget')
    view3d.show()
    sonarfeatureviz = view3d.createPlugin('sonarfeature', 'SonarFeatureVisualization')
    wallviz = view3d.createPlugin('wall', 'WallVisualization')
    auv_avalon = view3d.createPlugin('auv_avalon', 'AUVAvalonVisualization')
    auv_avalon.showDesiredModelPosition(true)
    #sonarbeamviz = view3d.createPlugin('sonarbeam', 'SonarBeamVisualization')

    # Connect debug port to vizkit plugins
    con = Vizkit.connect_port_to 'sonardetector', 'wall_data', :type => :buffer, :size => 100, :auto_reconnect => true, :pull => false, :update_frequency => 33 do |sample, name|
        sonarfeatureviz.updatePointCloud(sample.pointCloud)
        wallviz.updateWallData(sample.wall)
        sample
    end 

    con = Vizkit.connect_port_to 'sonardetector', 'position_command', :type => :buffer, :size => 100, :auto_reconnect => true, :pull => false, :update_frequency => 33 do |sample, name|
        auv_avalon.updateDesiredPosition(sample)
        sample
    end 

    log.orientation_estimator.orientation_samples :type => :buffer, :size => 100  do |sample|
        auv_avalon.updateRigidBodyState(sample)
        #sonarbeamviz.updateBodyState(sample)
        sample
    end 
    
    #log.sonar.BaseScan :type => :buffer, :size => 100  do |sample|
    #    sonarbeamviz.updateSonarScan(sample)
    #    sample
    #end 

    sonardetector.configure
    sonardetector.start
    viewer = Vizkit.default_loader.StructViewer
    log.sonar.BaseScan.connect_to viewer
    viewer.show

    Vizkit.display sonardetector.position_command
    task_inspector = Vizkit.default_loader.task_inspector
    task_inspector.config(sonardetector)
    task_inspector.show
    Vizkit.control log
    Vizkit.exec
end


