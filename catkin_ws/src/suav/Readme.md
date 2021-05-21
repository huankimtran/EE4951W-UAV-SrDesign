place the gazebo_sim directory in the src director of your catkin workspace. 
then from the cat_wks directory run catkin_make

to run, launch the file in the launch folder from the cat_wks directory using.

$roslaunch suav Quad_gazebo1.launch

motor commands are on the ros topic:

/Quad/joint_motor_controller/command \

which is a float64 array of values. 
