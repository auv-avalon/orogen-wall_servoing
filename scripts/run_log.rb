require 'vizkit'

Orocos.initialize
log = Orocos::Log::Replay.open(ARGV)

Orocos.run 'sonardetector', 'sonar_vizkit' do

    sonardetector = Orocos::TaskContext.get 'sonardetector'

    # sonarbeam settings
    sonardetector.enable_beam_threshold = true
    sonardetector.beam_threshold_min = 1
    sonardetector.beam_threshold_max = 5
    sonardetector.min_response_value = 10

    # wall estimation settings
    sonardetector.wall_estimation_start_angle = 2.36
    sonardetector.wall_estimation_end_angle = 3.93
    sonardetector.wall_estimation_ransac_threshold = 0.5
    sonardetector.wall_estimation_ransac_min_inliers = 0.7

    # controller settings
    sonardetector.wall_distance = 2
    sonardetector.fixed_depth = 1.5
    sonardetector.y_distance = 0
    sonardetector.heading_modulation = 0

    sonar_vizkit = Orocos::TaskContext.get 'sonar_vizkit'
    # sonarbeam settings
    sonar_vizkit.enable_beam_threshold = true
    sonar_vizkit.beam_threshold_min = 1
    sonar_vizkit.beam_threshold_max = 9
    sonar_vizkit.min_response_value = 2
    sonar_vizkit.show_all_beam_entries = false
    # wall estimation settings
    sonar_vizkit.enable_wall_estimation = true
    sonar_vizkit.wall_estimation_start_angle = 2.36
    sonar_vizkit.wall_estimation_end_angle = 3.93
    sonar_vizkit.wall_estimation_ransac_threshold = 0.4
    sonar_vizkit.wall_estimation_ransac_min_inliers = 0.5

    # write debug output
    sonardetector.debug_output = false

    log.sonar.BaseScan.connect_to sonardetector.sonar_input
    log.orientation_estimator.orientation_samples.connect_to sonardetector.orientation_sample
    log.sonar.BaseScan.connect_to sonar_vizkit.sonar_input
    log.orientation_estimator.orientation_samples.connect_to sonar_vizkit.body_state

    sonardetector.configure
    sonardetector.start
    sonar_vizkit.configure
    sonar_vizkit.start
    viewer = Vizkit.default_loader.StructViewer
    log.sonar.BaseScan.connect_to viewer
    viewer.show

    Vizkit.display sonardetector.position_command
    Vizkit.control log
    Vizkit.exec
end


