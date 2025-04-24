#!/bin/bash

# Grant read/write access to the Arduino serial port
sudo chmod a+rw /dev/ttyACM0

# Navigate to your ROS 2 workspace
cd ~/Documents/the_gaurdian || exit
source install/setup.bash

# Launch teleop_twist_joy in a new terminal
gnome-terminal -- bash -c "source ~/Documents/the_gaurdian/install/setup.bash; ros2 launch teleop_twist_joy teleop-launch.py; exec bash"

# Launch joy_to_serial_node in another terminal
gnome-terminal -- bash -c "source ~/Documents/the_gaurdian/install/setup.bash; ros2 run guardian_control joy_to_serial_node; exec bash"
