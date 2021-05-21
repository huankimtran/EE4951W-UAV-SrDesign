GASS_DIR = ../GASS/

setup:
	bash ./initsetup.bash
	cd catkin_ws && catkin_make
	make install

install:
	-yes | cp ./GASS_EXTENSION/models/solo.sdf $(shell rospack find px4)/Tools/sitl_gazebo/models/solo/solo.sdf
	-yes | cp ./GASS_EXTENSION/launch/niko_qr_launch.launch $(shell rospack find px4)/launch/niko_qr_launch.launch
	-yes | cp ./GASS_EXTENSION/worlds/niko_qr_landing.world $(shell rospack find mavlink_sitl_gazebo)/worlds/niko_qr_landing.world

simulation.launch:
	roslaunch px4 niko_qr_launch.launch

vision.launch:
	python ./GASS_EXTENSION/ros/camera_visual.py

landing.launch:
	python ./GASS_EXTENSION/ros/landing_controller.py

drone.takeoff:
	python ./GASS_EXTENSION/ros/px4_mavros_run.py

drone.send_target:
	rostopic pub /uav/goal std_msgs/String "data: '5,5,5'"

clean:
	-rm -r ./catkin_ws/devel
	-rm -r ./catkin_ws/build
