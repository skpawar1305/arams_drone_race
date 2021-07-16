#Instructions:
#In your home directory clone arams_ drone_race using following command:
git clone https://git.fh-aachen.de/skpawar1305/arams_drone_race.git

#In the next step build the cloned package
colcon build

#source it 
source ~/arams_drone_race/install/setup.bash

#after launching the gazebo in another terminal launch the flight control file:
ros2 launch offboard_ctrl fly.launch.py 


----------------------------------------------------------------------------------------------------------
#In order to fly the UAV through manual control
#first you need to install pygame 
pip install pygame

# run the manual control node
ros2 run offboard_ctrl manual_ctrl
