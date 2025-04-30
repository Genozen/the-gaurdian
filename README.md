# The Gaurdian Project
\A robotics project to help combat wildfires


## Setting up Jetson as Hotspot
`nmcli device wifi hotspot ifname wlP1p1s0 ssid JetsonNet password guardian123`

## microROS
```
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM1 -v6
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM0 -v4
```

## main Guardian scipts
```
ros2 run guardian_control joy_to_serial_node
ros2 run guardian_control guardian_nav_node 
```

## foxglove launch
`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
# then visualize on https://app.foxglove.dev/genozen/devices