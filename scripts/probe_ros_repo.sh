#!/bin/bash
docker exec arm_ros bash -c '
grep -rn "LookAt\|lookAt\|optical\|-Z\|0, 0, 1\|0,0,-1" /opt/ros/jazzy/opt/gz_sensors_vendor/include/gz/sensors8/gz/sensors/CameraSensor.hh | head -10
echo "---"
grep -rn "class.*Camera\|renderTarget\|image" /opt/ros/jazzy/opt/gz_sensors_vendor/include/gz/sensors8/gz/sensors/CameraSensor.hh | head -10
'