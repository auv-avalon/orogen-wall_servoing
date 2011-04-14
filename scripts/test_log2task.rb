require 'vizkit'

Orocos.initialize

Orocos.run 'sonarVizkit_test' do

sonarlog = Orocos::Log::Replay.open('/home/sirius/avalon/log/wall/20100701_MH_4_Wall_Approaching/sonar.0.new.0.log',Typelib::Registry.new)
poslog = Orocos::Log::Replay.open('/home/sirius/avalon/log/wall/20100701_MH_4_Wall_Approaching/pose_estimator.0.log',Typelib::Registry.new)

mytask = Orocos::TaskContext.get 'sonarvizkit'

sonarlog.sonar.SonarScan.connect_to mytask.sonar_input
#sonarlog.sonar.CurrentGroundDistance.connect_to mytask.ground_distance_input
#poslog.pose_estimator.pose_samples.connect_to mytask.body_state

mytask.configure
mytask.start

Vizkit.control sonarlog

Vizkit.exec

end
