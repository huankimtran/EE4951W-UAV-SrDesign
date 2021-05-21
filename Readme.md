# UAV project
## If you are not using this repository on the virtual machine attached with the hand off folder:
- Follow the instructions in the file No_VM_SetUP_Instructions.pdf
- PLEASE , in the "ROS-Kinetic" section, install ROS melodic and its dependency, not Kinetic
- PLEASE create a folder named Working on your Desktop. (mkdir -p ~/Desktop/Working), the run the git clone in the "OFFBOARD Control" section inside the Working folder so that the GAAS folder in cloned inside the Working folder
- Once you finished setup your environment as the instructions in the website above
- Add everything in the file AddToBashRc.txt in this folder to the end of your ~/.bashrc file
## How launch simulation and landing strategy (Make sure to execute these line in order)
- FOR ALL THE NEW TERMINAL DOWN HERE, PLEASE CHANGE THE DIRECTORY TO THE DIRECTORY CONTAINING THIS README FILE FIRST
- If this is the first time you use this repository, run "make setup" once, then run "make install"
- Launch the backend of the simulation by: New terminal -> make simulation.launch
- Launch the backend of the simulation by: New terminal -> gzclient
- Launch the landing strategy: New terminal -> make landing.launch 
- Take off the drone by: New terminal -> make drone.takeoff
- Send a landing target to the drone by: New terminal -> make drone.send -> CTRL +C to cancel reapeating sending -> Go back to the terminal where you ran "make landing.launch" to see landing strategy update status. (You can also look at the "Camera viewport" window poped up when you launched "make landing.launch" to see the landing identification process.
## Modifying the drone model or the simulated wolrd or the launch file
- The simulated world is in GASS_EXTENSION/worlds/niko_qr_landing.world
- The model of of the drone used in the simulation is in GASS_EXTENSION/models/solo.sdf
- The launch file is in GASS_EXTENSION/launch/niko_qr_launch.launch
- MAKE SURE AFTER ANY MODIFICATIONS MADE TO THESE FILES, YOU RUN: "make install". Otherwise the changes will not be recognized by the framework
## The GASS_EXTENSION/ros folder
- This folder contains all the ROS module used to implement the landing strategy testing pipeline
### Files
- px4_mavros_run.py - helps you take off the drone
- commander.py - contains the class Commander that could be used to control the drone once it is took off
- landing_controller.py - Implementing the current landing strategy
- qr_code.py - A lot of classes helping to process, locate QR code.
## The catkin_ws folder
- This folder contains artifacts our team produced during our first approach.
- We have completely migrated to use GAAS so this folder might not be useful, however, it contains working ROS package that could be useful for the future.
### How to use some of the packages in the folder
#### The MovingViewPort package
- Open a terminal -> run "source /home/uav/Desktop/Working/EE4951W-UAV/catkin_ws/devel/setup.sh"
- run "roslaunch uav_px4_sim px4_sim.launch" to launch the MovingViewPort application
- You might encounter error regarding cv2.qrcodedetector not avaialbe.
- This could be resolve by installing the cv2 library
- However, installation might affect the runability of the packages in GASS_EXTENSION folder
- SO PLEASE, BE AWARE OF THE RISK BEFORE YOU ALTER THE CV2 library of this virtual machine.