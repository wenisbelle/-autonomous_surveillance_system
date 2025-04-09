# GOAL

This project aims to create an autonomous surveillance system. What I hope with this is to implement differents concepts as computer vision, LLM's, behavior trees, SLAM, multi robot cooperation and much more in a system that I think that has many different applications. 

We can have a system like this one for survaillance of civil or military installations, as well as larger forest areas, finding new fire spots and acting quickly.

For this prototype I will be using very simple robots as the turtlebot and the crazyflie 2.1 drone. Just as a concept proof.

## System Especifications

- Ubuntu 22.04
- ROS2 Humble


## System Requirements 

- Docker Engine
- NVIDIA Container Toolkit 
 

## Installation
First of all just clone this project into your repository.

    git clone https://github.com/wenisbelle/Tecdron.git

Thre are folders named burger and waffle, these two you need to put respectively into your robots workspaces. Basically these are the packages you get following the official instructions (https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) with the modifications made in order to have the two robots working on the same network.

Now you can build the docker image, because everything is running inside containers.

    docker build -f dockerfile_nvidia_turtlebot_humble -t turtlebot_humble .

In this image, I'm using as base an ubuntu 22.04 with cuda 12.4.0, perhaps you will need another cuda image depending on your gpu.

To create the container just run:

    docker-compose -f docker-compose.yml up

Note that this file requires a joystick connected in the port /dev/js0. If you have a different port or just don't wanna use the joystick you can simple remove that line from this file. 

## Usage
For now you still need to run the ros2 packages on the turtlebots using ssh, but I will study some way of bypass this on the future. And I still didn't put the drone inside the system. These are the robot configurations:

### Devices Access Info

| Device  | Username | Password | Address        |
|---------|----------|----------|----------------|
| burger  | pi       | 123456   | 192.168.1.11   |
| waffle  | svtrp    | svtrp    | 192.168.1.12   |

Now just ssh in the robots. And be sure that the host computer address is 192.168.1.10, this is important for the DDS communication that will be discussed.

### Running each robot

Inside each robot just run the following command:

    ros2 run launch_multiple_turtlebots turtlebot_bringup.launch.py

Now the local system of the robot must be running and you will be able to see the topics on the host. 

### Running the host

In the host container you can control both robots, sending the same command, using:

    ros2 launch teleop_twist_joy teleop-launch.py
    ros2 run control_turtlebots control_turtlebots

Thinking about an improvement of the odometry data, we applied an extended kalman filter. For that you can just run:

    ros2 launch ekf burger_waffle.launch.py

The slam still lack documentation. 

## Modifications in the robots

Some modifications are required in order to run the two turtlebots together, adding the proper namespaces. I first used this reference: https://discourse.ros.org/t/giving-a-turtlebot3-a-namespace-for-multi-robot-experiments/10756. But this modifications don't add namespaces to the links. Unfortunatelly, I didn't found a good way of doing so, instead I changed the names of the links inside the urdf files. All modifications can be found in the test folder of teh Turtlebot_Dev directory and you can follow the modifications with the new launch files in the launch_multiple_turtlebots package inside each robot.


## DDS configuration

Communication among more than one robot and a host requires some modifications in the DDS. I had two options:

- Changing the cyclone dds configuration file. 
- Using Zenoh

Fow now I follow the first one, but maybe I will change in the future if I found any trouble with this configuration. You can found all these files inside the respectively directory of this git. 

For that to work properly, some parameters must be set in all robots (turtlebots and host):

- ROS_DOMAIN_ID=30
- RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
- export CYCLONEDDS_URI = /robot_directory/src/dds_config/cyclonedds_unicast_wlan.xml

### The new launch files


## TODO
[   ] Documentation

[ x ] Create a folder in the main git for the files in the waffle and in the burger

[   ] Create a main launcher package to run everything smoothly

[   ] Find a way of launching everything from the host computer, without the ssh directly.

[   ] Changging the docker internet configurations, making that the container has the address 192.168.1.10, independently from the host network configurations.

[   ] Improve the documentation of the SLAM

[   ] Use nav2

[   ] Put the drone in the system

[   ] Control the system using my previous LLM project



