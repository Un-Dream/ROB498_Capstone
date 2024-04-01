for flight test 3  

#in terminal 1:
#no_sim=1 make px4_sitl_default gazebo drone sample with no camera
cd /path/to/PX4-Autopilot
make px4_sitl gazebo_iris_opt_flow

#in termianl 2:
cd <PX4-Autopilot_clone>
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

#in terminal 3:
#roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world

roslaunch gazebo_ros yosemite.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/irisiris_opt_flow.world

#for real test case 
#make px4_sitl gazebo_rover__yosemite

#to add wind 
<plugin name='wind_plugin' filename='libgazebo_wind_plugin.so'>
      <frameId>base_link</frameId>
      <robotNamespace/>
      <xyzOffset>1 0 0</xyzOffset>
      <windDirectionMean>0 1 0</windDirectionMean>
      <windVelocityMean>SET_YOUR_WIND_SPEED</windVelocityMean>
      <windGustDirection>0 0 0</windGustDirection>
      <windGustDuration>0</windGustDuration>
      <windGustStart>0</windGustStart>
      <windGustVelocityMean>0</windGustVelocityMean>
      <windPubTopic>world_wind</windPubTopic>
    </plugin>
