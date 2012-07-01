require 'vizkit'
require 'orocos'
include Orocos

Orocos::CORBA.name_service = "192.168.128.50"
#Orocos::CORBA.name_service = "127.0.0.1"
Orocos.initialize

## get wall_servoing
wall_servoing = Orocos::TaskContext.get 'wall_servoing'
wall_servoing.enable_debug_output = true

## run visualizations
view3d = Vizkit.vizkit3d_widget
view3d.show()
sonarfeatureviz = Vizkit.default_loader.SonarFeatureVisualization
wallviz = Vizkit.default_loader.WallVisualization

## connect to debug port
Vizkit.connect_port_to 'wall_servoing', 'wall_servoing_debug', :pull => false, :update_frequency => 100 do |sample, name|
    sonarfeatureviz.updatePointCloud(sample.pointCloud)
    wallviz.updateWallData(sample.wall)
end

Vizkit.display wall_servoing
#Vizkit.display Orocos::TaskContext.get 'sonar_feature_estimator'
Vizkit.display Orocos::TaskContext.get 'sonar'

begin
    Vizkit.exec
rescue Interrupt => e
    wall_servoing.enable_debug_output = false
end
