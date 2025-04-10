#!/bin/bash

# Start robot 1 in background
ssh pi@192.168.1.11 'bash -c "
  source /opt/ros/humble/setup.bash &&
  source ~/turtlebot3_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=30 &&
  export LDS_MODEL=LDS-01 &&
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  export CYCLONEDDS_URI=/home/pi/turtlebot3_ws/src/dds_config/cyclonedds_unicast_wlan_pi.xml &&
  export TURTLEBOT3_MODEL=burger &&
  ros2 launch launch_multiple_turtlebots turtlebot_bringup.launch.py
"' &
PID1=$!

echo "Waiting for Burger to come online..."
timeout=60  # seconds
interval=5
elapsed=0

while ! ros2 topic list | grep -q '/burger/odom'; do
  if [ $elapsed -ge $timeout ]; then
    echo "Timeout reached: Burger did not come online."
    exit 1
  fi
  sleep $interval
  elapsed=$((elapsed + interval))
done

echo "Burger is online, starting Waffle..."

ssh svtrp@192.168.1.12 'bash -c "
  source /opt/ros/humble/setup.bash &&
  source ~/turtlebot3_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=30 &&
  export LDS_MODEL=LDS-01 &&
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  export CYCLONEDDS_URI=/home/svtrp/turtlebot3_ws/src/dds_config/cyclonedds_unicast_wlan_pi_waffle.xml &&
  export TURTLEBOT3_MODEL=waffle &&
  ros2 launch launch_multiple_turtlebots turtlebot_bringup.launch.py
"' &
PID2=$!

wait $PID1
wait $PID2


