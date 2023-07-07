# RPIPicoFreeRTOSuROSPub
Publishing Msg from uRos with FreeRTOS on Raspberry Pi Pico.

Project used for a Video Blog on my [YouTube Channel @DrJonEA](https://youtube.com/@DrJonEA)

## Cloning repo
Project uses submodules:
+ [FreeRTOS Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel)
+ [libmicroros](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk)

Clone using the --recurse-submodules switch

## Building Firmware
Firmware build using the normal Pico SDK build process:
```
mkdir build
cd build
cmake ..
make
```
Then can be flashed onto the pico using bootsel or SWD.

## Visualisation of Robot
A Robot visualisation, a simple Turntable, is writtent in URDF. This is include in the ros workspace. The package video_turntable will need to be build on your environment. You will require to have ROS2 isntalled.

```
cd ros_ws
colcon build
source install/local_setup.bash
ros2 launch video_turntable dispOnly.launch.py model:=urdf/turntable.urdf
```

## MicroROS Agent 
This will need to be run on the host connected to the Pico via USB. This tutorial talks about how to setup and run MicroROS Agent: https://robofoundry.medium.com/raspberry-pi-pico-ros2-via-micro-ros-actually-working-in-1-hr-9f7a3782d3e3



