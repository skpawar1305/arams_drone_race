# ARAMS project instructions

## Installation

Clone repository:

```
git clone https://git.fh-aachen.de/skpawar1305/arams_drone_race.git
```

Build ros2 package:

```
cd arams_drone_race
colcon build
```

Source it:

```
source ~/arams_drone_race/install/setup.bash
``` 

## Usage

Launch the project world and make sure PX4 micrortps client is working

In another terminal, initialise automatic control launch file using:

```
ros2 launch offboard_ctrl fly.launch.py
```

To launch it with manual control using keys W,S,A,D,Q,E:

```
pip install pygame
ros2 run offboard_ctrl manual_ctrl
```
