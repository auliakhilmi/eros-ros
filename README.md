# EROS in ROS Platform

* Semoga **Lulus 2019**
* Semoga **Tahun 2019 diterima S2 JSK Lab, The University of Tokyo**
* Semoga **EROS Juara 1 Robocup 2019, Sydney**
* Semoga **EROS Juara 1 KRSBI Nasional 2019**

## **Documentation**
* Motion Controller using U2D2
* Feedback using 9DoF AHRS IMU

## Setting Arduino
cd ~/sketchbook/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
rosrun rosserial_client make_library.py /home/eros/sketchbook/libraries/ros_lib/ /home/eros/eros_ws/src/robotcontrol/

