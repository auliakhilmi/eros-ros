# EROS in ROS Platform

## **Documentation**
* Motion Controller using U2D2
* Feedback using 6-Axis MPU6050

## Setting Arduino
```
cd ~/sketchbook/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
rosrun rosserial_client make_library.py /home/eros/sketchbook/libraries/ros_lib/ /home/eros/eros_ws/src/robotcontrol/
```
