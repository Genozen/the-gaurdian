# The Gaurdian Project
\A robotics project to help combat wildfires

## Quick Full Launch Order:
```
# starts GPS/IMU sensors
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM0 -v4

# starts Oak-D camera
ros2 launch depthai_ros_driver camera.launch.py params_file:=oak-d-rgb_only.yaml

# Starts major nodes
ros2 run guardian_control joy_to_serial_node
ros2 run guardian_control guardian_nav_node
ros2 run guardian_control fire_detector

# Remember to send Coordinates from Boron to initiate computation
```


## Setting up Jetson as Hotspot
`nmcli device wifi hotspot ifname wlP1p1s0 ssid JetsonNet password guardian123`

## microROS
```
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM1 -v6
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM0 -v4
```

## Luanch DepthAI Camera
```
ros2 launch depthai_ros_driver camera.launch.py params_file:=oak-d-rgb_only.yaml
```

## main Guardian scipts
```
ros2 run guardian_control joy_to_serial_node
ros2 run guardian_control guardian_nav_node
ros2 run guardian_control fire_detector 
```
!* # REMEMBER I HAVE TO TRIGGER SEND WAYPOINT FOR HEADING AND DISTANCE TO COMPUTE
*!

## foxglove launch
`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
# then visualize on https://app.foxglove.dev/genozen/devices

## Current Bug
-[ ] Must have new GPS points to trigger the IMU callback 


# Test GPS Points (Delete this later!!)
```
42.034752,-87.912656   (pt 1)
42.034752,-87.912473   (pt 2)
42.034494,-87.912473   (pt 3)
```
