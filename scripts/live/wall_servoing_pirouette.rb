require 'orocos'
include Orocos

#Orocos::CORBA.name_service = "192.168.128.50"
Orocos::CORBA.name_service = "127.0.0.1"
Orocos.initialize

## get wall_servoing
wall_servoing = Orocos::TaskContext.get 'wall_servoing'
wall_servoing.fading_out_factor = 0.028
sonar = Orocos::TaskContext.get 'sonar'
config = sonar.config
config.continous = true
sonar.config = config

while 1
    sleep 1
    servoing_wall_direction = wall_servoing.servoing_wall_direction + 0.1
    if servoing_wall_direction > Math::PI
        servoing_wall_direction = servoing_wall_direction - 2 * Math::PI
    end
    if servoing_wall_direction <= -Math::PI
        servoing_wall_direction = servoing_wall_direction + 2 * Math::PI
    end
    puts servoing_wall_direction
    wall_servoing.servoing_wall_direction = servoing_wall_direction
end
